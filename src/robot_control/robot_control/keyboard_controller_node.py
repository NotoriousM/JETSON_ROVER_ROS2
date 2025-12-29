#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import select
import termios
import tty
import time

class KeyboardController(Node):
    def __init__(self):
        super().__init__('robot_keyboard_controller')
        self.publisher = self.create_publisher(
            Float32MultiArray, 
            '/motion_commands', 
            10)
        
        # ИЗМЕНЕНО: Для модели Аккермана
        self.speed = 0.0  # Скорость транспортного средства (м/с)
        self.steering_angle = 0.0  # Угол поворота передней оси (градусы)
        
        # Параметры управления
        self.declare_parameter('max_speed', 5.0)  # Уменьшено для безопасности
        self.declare_parameter('max_steering_angle', 30.0)  # Максимальный угол поворота
        self.declare_parameter('base_step_angle', 2.0)  # Шаг изменения угла в градусах
        self.declare_parameter('base_step_speed', 0.5)  # Шаг изменения скорости
        self.declare_parameter('initial_servo_pos', 90.0)  # Начальное положение сервопривода
        
        # Получаем параметры
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.base_step_angle = self.get_parameter('base_step_angle').value
        self.base_step_speed = self.get_parameter('base_step_speed').value
        self.initial_servo_pos = self.get_parameter('initial_servo_pos').value
        
        # Для ускорения при удержании клавиш
        self.acceleration_factor = 1.0
        self.max_acceleration = 3.0
        self.acceleration_rate = 0.15
        self.last_accel_update = time.time()
        self.active_keys = set()
        self.last_key_time = 0
        
        # Инициализация переменных шага
        self.current_step_angle = self.base_step_angle
        self.current_step_speed = self.base_step_speed
        
        self.get_logger().info("Keyboard controller READY for Ackermann model")
        self.print_instructions()
        
        # Сохраняем оригинальные настройки терминала
        self.settings = termios.tcgetattr(sys.stdin)

    def print_instructions(self):
        print("\n=== Управление для модели Аккермана ===")
        print("  W/S: Увеличить/Уменьшить скорость транспортного средства")
        print("  A/D: Поворот передней оси влево/вправо")
        print("  Space/K: Полная остановка (скорость=0, угол=0)")
        print("  C: Центрирование руля (угол=0)")
        print("  Q: Выход")
        print(f"Текущее состояние: Скорость={self.speed:.1f} м/с, Угол поворота={self.steering_angle:.1f}°")
        print(f"Параметры: Max скорость={self.max_speed:.1f} м/с, Max угол={self.max_steering_angle:.1f}°")
        print("=========================================\n")

    def send_command(self):
        msg = Float32MultiArray()
        # ИЗМЕНЕНО: Отправляем [speed, steering_angle] для модели Аккермана
        msg.data = [float(self.speed), float(self.steering_angle)]
        self.publisher.publish(msg)
        self.get_logger().info(f"Command: Speed={self.speed:.1f} m/s | Steering={self.steering_angle:.1f}°")

    def update_acceleration(self):
        current_time = time.time()
        if current_time - self.last_accel_update > 0.1:
            if self.active_keys:
                self.acceleration_factor = min(
                    self.acceleration_factor + self.acceleration_rate,
                    self.max_acceleration
                )
            else:
                self.acceleration_factor = 1.0
            self.last_accel_update = current_time
            
            # Обновляем параметры (на случай изменения во время работы)
            self.max_speed = self.get_parameter('max_speed').value
            self.max_steering_angle = self.get_parameter('max_steering_angle').value
            self.base_step_angle = self.get_parameter('base_step_angle').value
            self.base_step_speed = self.get_parameter('base_step_speed').value
            self.initial_servo_pos = self.get_parameter('initial_servo_pos').value
            
            # Рассчитываем текущий шаг с ускорением
            self.current_step_angle = self.base_step_angle * self.acceleration_factor
            self.current_step_speed = self.base_step_speed * self.acceleration_factor

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def key_listener(self):
        self.print_instructions()
        print("Нажимайте клавиши управления...")
        
        try:
            while True:
                key = self.get_key()
                current_time = time.time()
                
                if key:
                    # Обновляем состояние активных клавиш
                    if key in ['a', 'd', 'w', 's']:
                        self.active_keys.add(key)
                    else:
                        self.active_keys.clear()
                    
                    if key == 'q':
                        print("\nВыход...")
                        break
                        
                    elif key in [' ', 'k']:
                        # ИЗМЕНЕНО: Полная остановка - скорость и центрирование
                        self.speed = 0.0
                        self.steering_angle = 0.0
                        self.send_command()
                        self.active_keys.clear()
                        print("\n⚡ ПОЛНАЯ ОСТАНОВКА")
                        
                    elif key == 'c':
                        # НОВОЕ: Центрирование руля
                        self.steering_angle = 0.0
                        self.send_command()
                        print("\n🎯 Руль центрирован")
                        
                    elif key == 'w':
                        # Увеличение скорости
                        new_speed = self.speed + self.current_step_speed
                        self.speed = min(new_speed, self.max_speed)
                        self.send_command()
                        
                    elif key == 's':
                        # Уменьшение скорости (может быть отрицательной для заднего хода)
                        new_speed = self.speed - self.current_step_speed
                        self.speed = max(new_speed, -self.max_speed)
                        self.send_command()
                        
                    elif key == 'a':
                        # Поворот влево (отрицательный угол)
                        new_angle = self.steering_angle - self.current_step_angle
                        self.steering_angle = max(new_angle, -self.max_steering_angle)
                        self.send_command()
                        
                    elif key == 'd':
                        # Поворот вправо (положительный угол)
                        new_angle = self.steering_angle + self.current_step_angle
                        self.steering_angle = min(new_angle, self.max_steering_angle)
                        self.send_command()
                    
                    self.last_key_time = current_time
                else:
                    # Если клавиша отпущена
                    self.active_keys.clear()
                
                # Обновляем ускорение
                self.update_acceleration()
                
                # Выводим статус в терминал
                direction = "⬆️ ВПЕРЕД" if self.speed > 0 else "⬇️ НАЗАД" if self.speed < 0 else "⏹️ СТОП"
                turn = "⬅️ ЛЕВО" if self.steering_angle < 0 else "➡️ ПРАВО" if self.steering_angle > 0 else "⬆️ ПРЯМО"
                
                print(f"\rСкорость: {self.speed:5.1f} м/с {direction} | Угол: {self.steering_angle:5.1f}° {turn} | Ускорение: x{self.acceleration_factor:.1f}", 
                      end='', flush=True)
                
                # Ускоряем обработку ROS2
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка в цикле обработки: {str(e)}")
            
        finally:
            # Восстанавливаем настройки терминала
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\n\nЗавершение работы клавиатурного контроллера")

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    
    try:
        controller.key_listener()
    except KeyboardInterrupt:
        print("\nПрервано пользователем")
    except Exception as e:
        controller.get_logger().error(f"Критическая ошибка: {str(e)}")
    finally:
        # Отправляем команду остановки перед выходом
        controller.speed = 0.0
        controller.steering_angle = 0.0
        controller.send_command()
        time.sleep(0.1)
        
        controller.destroy_node()
        rclpy.shutdown()
        print("ROS2 узлы остановлены")

if __name__ == '__main__':
    main()