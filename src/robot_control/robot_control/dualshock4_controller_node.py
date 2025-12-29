#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import evdev
from evdev import InputDevice, ecodes
import threading
import array
import math
import time

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller_node')
        
        # Параметры управления
        self.declare_parameter('max_speed', 14.5)
        self.declare_parameter('max_angle', 30)
        self.declare_parameter('deadzone', 0.15)  # Увеличена мертвая зона
        self.declare_parameter('steering_scale', 1.0)
        self.declare_parameter('steering_exponent', 1.8)  # Увеличена для более плавного отклика
        
        # Параметры для плавности
        self.declare_parameter('angle_smoothing_factor', 0.15)  # Уменьшено для большей плавности
        self.declare_parameter('max_angle_velocity', 12.0)     # Снижена максимальная скорость
        self.declare_parameter('angle_acceleration', 15.0)     # Снижено ускорение
        
        # Параметры для плавного возврата
        self.declare_parameter('return_smoothing_factor', 0.08)   # Очень плавный возврат
        self.declare_parameter('return_max_velocity', 4.0)        # Медленная скорость возврата
        self.declare_parameter('auto_center_threshold', 1.0)      # Увеличен порог
        
        # Дополнительные параметры плавности
        self.declare_parameter('min_movement_threshold', 0.05)    # Порог для игнорирования микродвижений
        self.declare_parameter('double_filter_factor', 0.85)      # Коэффициент двойного фильтра
        
        # Публикатор команд управления
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/motion_commands',
            10
        )
        
        # Инициализация джойстика
        self.controller = self.init_controller()
        if not self.controller:
            self.get_logger().error("PS4 controller not found!")
            return
            
        self.get_logger().info(f"Controller found: {self.controller.name}")
        
        # Текущее состояние
        self.linear_speed = 0.0
        self.steering_angle = 0.0  # Текущий угол сервопривода
        self.target_angle = 0.0    # Целевой угол от джойстика
        self.filtered_angle = 0.0  # Сглаженный угол (для публикации)
        self.previous_target_angle = 0.0  # Предыдущий целевой угол
        
        # Получение параметров
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angle = self.get_parameter('max_angle').value
        self.deadzone = self.get_parameter('deadzone').value
        self.steering_scale = self.get_parameter('steering_scale').value
        self.steering_exponent = self.get_parameter('steering_exponent').value
        
        # Параметры плавности
        self.smoothing_factor = self.get_parameter('angle_smoothing_factor').value
        self.max_angle_velocity = self.get_parameter('max_angle_velocity').value
        self.angle_acceleration = self.get_parameter('angle_acceleration').value
        
        # Параметры возврата
        self.return_smoothing = self.get_parameter('return_smoothing_factor').value
        self.return_max_velocity = self.get_parameter('return_max_velocity').value
        self.auto_center_threshold = self.get_parameter('auto_center_threshold').value
        
        # Дополнительные параметры
        self.min_movement_threshold = self.get_parameter('min_movement_threshold').value
        self.double_filter_factor = self.get_parameter('double_filter_factor').value
        
        self.emergency_stop = False
        
        # Система плавности
        self.current_velocity = 0.0
        self.last_time = time.time()
        self.velocity_filter = 0.0  # Фильтр скорости
        
        # Переменные для плавного возврата
        self.return_to_center = False
        self.last_joystick_active = False
        self.return_start_time = 0.0
        self.return_start_angle = 0.0
        
        # Дополнительные переменные для плавности
        self.angle_history = [0.0] * 5  # История углов для дополнительного сглаживания
        self.history_index = 0
        
        # Поток для чтения событий джойстика
        self.thread = threading.Thread(target=self.read_loop)
        self.thread.daemon = True
        self.thread.start()
        
        # Таймеры
        self.smooth_timer = self.create_timer(0.01, self.smooth_angle_update)  # 50 Гц
        self.pub_timer = self.create_timer(0.05, self.publish_command)         # 20 Гц

    def init_controller(self):
        devices = [InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if "Wireless Controller" in device.name:
                return device
        return None

    def read_loop(self):
        try:
            for event in self.controller.read_loop():
                self.process_event(event)
        except Exception as e:
            self.get_logger().error(f"Controller read error: {str(e)}")

    def process_event(self, event):
        if event.type == ecodes.EV_KEY:
            self.process_buttons(event)
        elif event.type == ecodes.EV_ABS:
            self.process_axes(event)

    def process_buttons(self, event):
        if event.code == 305 and event.value == 1:  # Кнопка X
            self.emergency_stop = True
            self.get_logger().warn("EMERGENCY STOP activated!")
        elif event.code == 306 and event.value == 1:  # Кнопка O
            self.emergency_stop = False
            self.get_logger().info("Emergency stop released")
        elif event.code == 307 and event.value == 1:  # Кнопка △
            self.max_speed = min(self.max_speed + 0.1, 20.0)
            self.get_logger().info(f"Max speed increased to {self.max_speed:.1f} m/s")
        elif event.code == 304 and event.value == 1:  # Кнопка □
            self.max_speed = max(self.max_speed - 0.1, 0.1)
            self.get_logger().info(f"Max speed decreased to {self.max_speed:.1f} m/s")

    def process_axes(self, event):
        # Левый стик Y (ось 1) - линейная скорость
        if event.code == 1:  # Левый стик Y
            value = (event.value - 128) / 128.0
            if abs(value) < self.deadzone:
                self.linear_speed = 0.0
            else:
                signed_value = -value
                abs_value = abs(signed_value)
                # Очень плавная кривая для управления скоростью
                scaled_value = math.copysign(abs_value ** 1.5, signed_value)
                self.linear_speed = scaled_value * self.max_speed
        
        # Правый стик X (ось 2) - угол поворота
        elif event.code == 2:  # Правый стик X
            value = (event.value - 128) / 128.0
            
            # Определяем активность джойстика
            joystick_active = abs(value) >= self.deadzone
            
            # Если джойстик стал активным - отменяем возврат
            if joystick_active and self.return_to_center:
                self.return_to_center = False
                self.get_logger().info("Return to center cancelled - joystick active")
            
            # Если джойстик был активен и стал неактивен - начинаем плавный возврат
            if not joystick_active and self.last_joystick_active:
                self.start_smooth_return()
            
            # Обновляем состояние активности
            self.last_joystick_active = joystick_active
            
            if joystick_active:
                # Применяем нелинейную кривую с очень плавным откликом
                abs_value = abs(value)
                
                # Дополнительное сглаживание входного сигнала
                smooth_value = math.copysign(abs_value ** self.steering_exponent, value)
                
                # Рассчитываем целевой угол с ограничением скорости изменения
                desired_angle = smooth_value * self.max_angle * self.steering_scale
                desired_angle = max(-self.max_angle, min(desired_angle, self.max_angle))
                
                # ПЛАВНОЕ ИЗМЕНЕНИЕ ЦЕЛЕВОГО УГЛА - ключевое улучшение!
                angle_change = desired_angle - self.target_angle
                max_angle_change = 2.0  # Максимальное изменение целевого угла за один раз
                
                if abs(angle_change) > max_angle_change:
                    self.target_angle += math.copysign(max_angle_change, angle_change)
                else:
                    self.target_angle = desired_angle
                
                # Сохраняем предыдущее значение
                self.previous_target_angle = self.target_angle
            else:
                # В мертвой зоне - если не в режиме возврата, сохраняем текущее положение
                if not self.return_to_center:
                    # Не меняем target_angle - сервоприводы сохраняют положение
                    pass

    def start_smooth_return(self):
        """Начинает плавный возврат сервоприводов в центр"""
        if abs(self.target_angle) > self.auto_center_threshold:
            self.return_to_center = True
            self.return_start_time = time.time()
            self.return_start_angle = self.target_angle
            self.get_logger().info(f"Starting smooth return from {self.target_angle:.1f}° to 0°")

    def smooth_angle_update(self):
        """УЛУЧШЕННЫЙ алгоритм плавного изменения угла без рывков"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # ЕСЛИ АКТИВИРОВАН ПЛАВНЫЙ ВОЗВРАТ
        if self.return_to_center:
            # Рассчитываем прогресс возврата
            return_duration = current_time - self.return_start_time
            total_return_time = 1.5  # Увеличено время возврата до 1.5 секунд
            progress = min(return_duration / total_return_time, 1.0)
            
            # Очень плавная кривая для возврата (ease-in-out)
            if progress < 0.5:
                ease_progress = 2 * progress * progress
            else:
                ease_progress = 1 - pow(-2 * progress + 2, 2) / 2
            
            # Вычисляем целевой угол для плавного возврата
            self.target_angle = self.return_start_angle * (1.0 - ease_progress)
            
            # Если достигли центра - завершаем возврат
            if abs(self.target_angle) < 0.05:
                self.target_angle = 0.0
                self.return_to_center = False
                self.get_logger().info("Smooth return to center completed")
        
        # Вычисляем ошибку (разницу между целевым и текущим углом)
        error = self.target_angle - self.steering_angle
        
        # Если ошибка очень мала, ничего не делаем для предотвращения микродвижений
        if abs(error) < self.min_movement_threshold:
            return
        
        # ВЫБИРАЕМ ПАРАМЕТРЫ СГЛАЖИВАНИЯ В ЗАВИСИМОСТИ ОТ РЕЖИМА
        if self.return_to_center:
            # В режиме возврата используем более плавные параметры
            smoothing = self.return_smoothing
            max_velocity = self.return_max_velocity
        else:
            # В обычном режиме - стандартные параметры
            smoothing = self.smoothing_factor
            max_velocity = self.max_angle_velocity
        
        # ЭКСПОНЕНЦИАЛЬНОЕ СГЛАЖИВАНИЕ С ДОПОЛНИТЕЛЬНЫМ ФИЛЬТРОМ
        step = error * smoothing
        
        # Ограничиваем шаг максимальным ускорением
        max_acceleration_step = self.angle_acceleration * dt
        if abs(step) > max_acceleration_step:
            step = math.copysign(max_acceleration_step, step)
        
        # Ограничиваем максимальную скорость
        max_velocity_step = max_velocity * dt
        if abs(step) > max_velocity_step:
            step = math.copysign(max_velocity_step, step)
        
        # Обновляем угол
        self.steering_angle += step
        
        # Рассчитываем текущую скорость для отладки
        current_velocity = step / dt if dt > 0 else 0
        
        # ФИЛЬТРАЦИЯ СКОРОСТИ для дополнительной плавности
        self.velocity_filter = self.velocity_filter * 0.9 + current_velocity * 0.1
        self.current_velocity = self.velocity_filter
        
        # ИСТОРИЯ УГЛОВ для дополнительного сглаживания
        self.angle_history[self.history_index] = self.steering_angle
        self.history_index = (self.history_index + 1) % len(self.angle_history)
        
        # УСИЛЕННОЕ СГЛАЖИВАНИЕ ДЛЯ ПУБЛИКАЦИИ
        # Используем среднее значение из истории углов
        historical_avg = sum(self.angle_history) / len(self.angle_history)
        
        # Комбинируем текущий угол с историческим средним
        smoothed_angle = (historical_avg * 0.3 + self.steering_angle * 0.7)
        
        # Финальное сглаживание
        self.filtered_angle = (self.filtered_angle * self.double_filter_factor + 
                              smoothed_angle * (1.0 - self.double_filter_factor))

    def publish_command(self):
        msg = Float32MultiArray()
        
        if self.emergency_stop:
            msg.data = array.array('f', [0.0, 0.0])
            self.get_logger().warn("Emergency stop active! Motors disabled", throttle_duration_sec=1.0)
        else:
            # Используем максимально сглаженный угол для публикации
            msg.data = array.array('f', [self.linear_speed, self.filtered_angle])
        
        self.publisher.publish(msg)
        
        # Логирование с информацией о плавности
        mode = "RETURN" if self.return_to_center else "NORMAL"
        self.get_logger().info(
            f"Mode: {mode} | "
            f"Speed: {self.linear_speed:6.2f} m/s | "
            f"Angle: {self.filtered_angle:6.2f}° | "
            f"Target: {self.target_angle:6.2f}° | "
            f"Vel: {self.current_velocity:5.1f}°/s",
            throttle_duration_sec=0.3  # Увеличено время троттлинга
        )

def main(args=None):
    rclpy.init(args=args)
    node = PS4ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()