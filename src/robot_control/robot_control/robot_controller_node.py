#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, UInt8
import serial
import struct
import time
import math
import threading
import queue
from typing import Dict, List, Tuple

class SerialPortManager:
    """Класс для управления отдельным COM-портом с собственной очередью и потоком"""
    
    def __init__(self, port_name: str, baudrate: int, timeout: float, initial_servo_pos: float):
        self.port_name = port_name
        self.baudrate = baudrate
        self.timeout = timeout
        self.initial_servo_pos = initial_servo_pos
        self.ser = None
        self.command_queue = queue.Queue(maxsize=100)
        self.thread = None
        self.running = False
        self.connected = False
        
    def start(self):
        """Запуск потока для управления портом"""
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        
    def stop(self):
        """Остановка потока и закрытие порта"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self._close_port()
        
    def send_command(self, speed: float, angle: int):
        """Добавление команды в очередь (неблокирующее)"""
        try:
            self.command_queue.put_nowait((speed, angle))
            return True
        except queue.Full:
            return False
            
    def _connect_port(self) -> bool:
        """Подключение к COM-порту"""
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.connected = True
            
            # Отправка начальной позиции при подключении
            packed_data = struct.pack('<fH', 0.0, int(self.initial_servo_pos))
            self.ser.write(packed_data)
            self.ser.flush()
            
            return True
            
        except Exception as e:
            self.connected = False
            if self.ser:
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None
            return False
            
    def _close_port(self):
        """Закрытие COM-порта"""
        if self.ser and self.ser.is_open:
            try:
                # Отправка команды возврата в начальное положение
                packed_data = struct.pack('<fH', 0.0, int(self.initial_servo_pos))
                self.ser.write(packed_data)
                self.ser.flush()
                self.ser.close()
            except Exception:
                pass
        self.connected = False
        self.ser = None
        
    def _run_loop(self):
        """Основной цикл обработки команд для порта"""
        reconnect_delay = 1.0  # Задержка между попытками переподключения
        last_reconnect_attempt = 0
        
        while self.running:
            current_time = time.time()
            
            # Попытка подключения если порт не подключен
            if not self.connected:
                if current_time - last_reconnect_attempt >= reconnect_delay:
                    if self._connect_port():
                        print(f"Connected to {self.port_name}")
                    else:
                        print(f"Failed to connect to {self.port_name}, retrying in {reconnect_delay} sec")
                    last_reconnect_attempt = current_time
                time.sleep(0.1)
                continue
                
            # Обработка команд из очереди
            try:
                # Неблокирующее получение команды
                speed, angle = self.command_queue.get_nowait()
                
                # Отправка команды на порт
                if self.connected and self.ser and self.ser.is_open:
                    packed_data = struct.pack('<fH', speed, angle)
                    self.ser.write(packed_data)
                    self.ser.flush()
                    
            except queue.Empty:
                # Нет команд в очереди - небольшая пауза
                time.sleep(0.001)
            except serial.SerialException as e:
                print(f"Serial error on {self.port_name}: {e}")
                self._close_port()
            except Exception as e:
                print(f"Error processing command on {self.port_name}: {e}")
                
        # Завершение работы - закрытие порта
        self._close_port()


class ServoWheelController(Node):
    def __init__(self):
        super().__init__('robot_serial_controller')
        
        # Параметры робота
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('wheelbase', 0.807)
        self.declare_parameter('track_width', 0.779)
        self.declare_parameter('a_distance', 0.4035)
        self.declare_parameter('b_distance', 0.4035)
        self.declare_parameter('initial_pos_servo_deg', 90.0)
        
        # Параметры COM-портов (6 уникальных портов для каждого колеса)
        self.declare_parameter('port_front_right', '/dev/ttyROVER_WHEEL_1')   # WHEEL_1 - правое переднее поворотное
        self.declare_parameter('port_middle_right', '/dev/ttyROVER_WHEEL_2')  # WHEEL_2 - правое центральное неповоротное
        self.declare_parameter('port_rear_right', '/dev/ttyROVER_WHEEL_3')    # WHEEL_3 - правое заднее поворотное
        self.declare_parameter('port_front_left', '/dev/ttyROVER_WHEEL_4')    # WHEEL_4 - левое переднее поворотное
        self.declare_parameter('port_middle_left', '/dev/ttyROVER_WHEEL_5')   # WHEEL_5 - левое центральное неповоротное
        self.declare_parameter('port_rear_left', '/dev/ttyROVER_WHEEL_6')     # WHEEL_6 - левое заднее поворотное
        
        # Получение параметров робота
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.a = self.get_parameter('a_distance').value
        self.b = self.get_parameter('b_distance').value
        self.initial_servo_pos = self.get_parameter('initial_pos_servo_deg').value
        
        # Получение параметров портов
        port_configs = [
            ('front_right', self.get_parameter('port_front_right').value),    # WHEEL_1
            ('middle_right', self.get_parameter('port_middle_right').value),  # WHEEL_2  
            ('rear_right', self.get_parameter('port_rear_right').value),      # WHEEL_3
            ('front_left', self.get_parameter('port_front_left').value),      # WHEEL_4
            ('middle_left', self.get_parameter('port_middle_left').value),    # WHEEL_5
            ('rear_left', self.get_parameter('port_rear_left').value)         # WHEEL_6
        ]
        
        # Инициализация менеджеров портов
        self.port_managers: Dict[str, SerialPortManager] = {}
        baud = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        for port_name, port_path in port_configs:
            manager = SerialPortManager(port_path, baud, timeout, self.initial_servo_pos)
            self.port_managers[port_name] = manager
            manager.start()
            self.get_logger().info(f"Initialized port manager for {port_name} -> {port_path}")
        
        # Публикаторы для мониторинга
        self.speed_pub = self.create_publisher(Float32, '/motor_speed', 10)
        self.flag_pub = self.create_publisher(UInt8, '/motor_flag', 10)
        
        # Подписка на команды управления
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motion_commands',
            self.command_callback,
            10
        )
        
        # Таймер для мониторинга состояния портов
        self.create_timer(5.0, self._monitor_ports)
        
        self.get_logger().info("Controller initialized with multi-port Ackermann model")

    def _monitor_ports(self):
        """Мониторинг состояния COM-портов"""
        connected_ports = []
        disconnected_ports = []
        
        for name, manager in self.port_managers.items():
            if manager.connected:
                connected_ports.append(name)
            else:
                disconnected_ports.append(name)
                
        if disconnected_ports:
            self.get_logger().warn(
                f"Disconnected ports: {disconnected_ports} | "
                f"Connected: {connected_ports}",
                throttle_duration_sec=10.0
            )

    def ackermann_calculation(self, V: float, theta: float) -> Tuple[List[float], List[float]]:
        """
        Расчет параметров модели Аккермана
        Возвращает: скорости колес [VL1, VL2, VL3, VR1, VR2, VR3] 
                   и углы поворота [alpha_L1, alpha_L3, beta_R1, beta_R3]
        """
        # Если скорость близка к нулю - все колеса останавливаются
        if abs(V) < 0.001:
            wheel_speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            wheel_angles = [0.0, 0.0, 0.0, 0.0]
            return wheel_speeds, wheel_angles
        
        # Если угол близок к нулю - прямолинейное движение
        if abs(theta) < 0.001:
            wheel_speeds = [V, V, V, V, V, V]
            wheel_angles = [0.0, 0.0, 0.0, 0.0]
            return wheel_speeds, wheel_angles
        
        # Расчет радиуса поворота центра робота
        R = self.a / math.tan(math.radians(theta))
        
        # Расчет радиусов для каждого колеса
        R_L1 = math.sqrt(self.a**2 + (R + self.W/2)**2)  # Переднее левое
        R_R1 = math.sqrt(self.a**2 + (R - self.W/2)**2)  # Переднее правое
        R_L2 = R + self.W/2  # Среднее левое
        R_R2 = R - self.W/2  # Среднее правое  
        R_L3 = math.sqrt(self.b**2 + (R + self.W/2)**2)  # Заднее левое
        R_R3 = math.sqrt(self.b**2 + (R - self.W/2)**2)  # Заднее правоe
        
        # Расчет углов поворота колес (в градусах)
        alpha_L1 = math.degrees(math.atan(self.a / (R + self.W/2)))  # Переднее левое
        alpha_R1 = math.degrees(math.atan(self.a / (R - self.W/2)))  # Переднее правое
        beta_L3 = math.degrees(math.atan(self.b / (R + self.W/2)))   # Заднее левое
        beta_R3 = math.degrees(math.atan(self.b / (R - self.W/2)))   # Заднее правое
        
        # Расчет скоростей колес с проверкой знака
        # ИСПРАВЛЕНИЕ: Учитываем знак скорости V и направления поворота
        sign_V = 1.0 if V >= 0 else -1.0
        sign_R = 1.0 if R >= 0 else -1.0
        
        # Основное правило: все колеса должны двигаться в направлении скорости V
        # с поправкой на радиус поворота
        V_L1 = sign_V * abs(V) * R_L1 / abs(R) if abs(R) > 0.001 else V
        V_R1 = sign_V * abs(V) * R_R1 / abs(R) if abs(R) > 0.001 else V
        V_L2 = sign_V * abs(V) * R_L2 / abs(R) if abs(R) > 0.001 else V
        V_R2 = sign_V * abs(V) * R_R2 / abs(R) if abs(R) > 0.001 else V
        V_L3 = sign_V * abs(V) * R_L3 / abs(R) if abs(R) > 0.001 else V
        V_R3 = sign_V * abs(V) * R_R3 / abs(R) if abs(R) > 0.001 else V
        
        # ИСПРАВЛЕНИЕ: Проверяем, что все скорости имеют правильный знак относительно V
        # Если V положительный, все скорости должны быть положительными (и наоборот)
        wheel_speeds = [V_L1, V_L2, V_L3, V_R1, V_R2, V_R3]
        for i in range(len(wheel_speeds)):
            if abs(wheel_speeds[i]) < 0.001:  # Очень маленькие скорости
                wheel_speeds[i] = 0.0
            elif (V >= 0 and wheel_speeds[i] < 0) or (V < 0 and wheel_speeds[i] > 0):
                # Исправляем неправильный знак
                wheel_speeds[i] = -wheel_speeds[i] if abs(wheel_speeds[i]) > 0.001 else 0.0
        
        wheel_angles = [alpha_L1, alpha_R1, beta_L3, beta_R3]
        
        return wheel_speeds, wheel_angles

    def command_callback(self, msg):
        """Обработка входящих команд управления"""
        if len(msg.data) != 2:
            self.get_logger().error("Need [speed, angle]", throttle_duration_sec=2.0)
            return
            
        try:
            vehicle_speed = float(msg.data[0])
            steering_angle = float(msg.data[1])
            
            # Расчет параметров Аккермана
            wheel_speeds, wheel_angles = self.ackermann_calculation(vehicle_speed, steering_angle)
            
            # Логирование расчетов (с ограничением частоты)
            self.get_logger().info(
                f"Ackermann: V={vehicle_speed:.2f}, θ={steering_angle:.1f}° -> "
                f"Speeds: {[f'{v:.3f}' for v in wheel_speeds]}",
                throttle_duration_sec=1.0
            )
            
            # Распределение команд по портам в соответствии с колесами
            # Порядок соответствует: [VL1, VL2, VL3, VR1, VR2, VR3]
            port_commands = [
                # (manager_name, speed, angle, is_steering, wheel_description)
                ('front_left', wheel_speeds[0], wheel_angles[0], True, "WHEEL_4 - левое переднее поворотное"),
                ('middle_left', wheel_speeds[1], 0.0, False, "WHEEL_5 - левое центральное неповоротное"),
                ('rear_left', wheel_speeds[2], wheel_angles[2], True, "WHEEL_6 - левое заднее поворотное"),
                ('front_right', wheel_speeds[3], wheel_angles[1], True, "WHEEL_1 - правое переднее поворотное"),
                ('middle_right', wheel_speeds[4], 0.0, False, "WHEEL_2 - правое центральное неповоротное"),
                ('rear_right', wheel_speeds[5], wheel_angles[3], True, "WHEEL_3 - правое заднее поворотное")
            ]
            
            # Отправка команд в соответствующие порты
            successful_commands = 0
            for manager_name, speed, angle, is_steering, wheel_desc in port_commands:
                if manager_name in self.port_managers:
                    # Расчет конечного угла с учетом начального положения
                    if is_steering:
                        # ИСПРАВЛЕНИЕ: Учитываем направление поворота
                        if steering_angle >= 0:  # Поворот вправо
                            # Правое колесо поворачивается внутрь, левое - наружу
                            if 'right' in manager_name:
                                final_angle = int(self.initial_servo_pos + angle)  # Внутрь
                            else:
                                final_angle = int(self.initial_servo_pos + angle)  # Наружу
                        else:  # Поворот влево
                            # Левое колесо поворачивается внутрь, правое - наружу
                            if 'left' in manager_name:
                                final_angle = int(self.initial_servo_pos + angle)  # Внутрь
                            else:
                                final_angle = int(self.initial_servo_pos + angle)  # Наружу
                        
                        final_angle = max(0, min(180, final_angle))
                    else:
                        final_angle = int(self.initial_servo_pos)  # Неповоротные колеса
                    
                    # ИСПРАВЛЕНИЕ: Убеждаемся, что скорость имеет правильный знак
                    corrected_speed = speed
                    if abs(corrected_speed) < 0.001:
                        corrected_speed = 0.0
                    
                    # Отправка команды в очередь порта
                    if self.port_managers[manager_name].send_command(corrected_speed, final_angle):
                        successful_commands += 1
                        # Детальное логирование для отладки
                        self.get_logger().debug(
                            f"{wheel_desc}: speed={corrected_speed:.3f}, angle={final_angle}°",
                            throttle_duration_sec=2.0
                        )
                    else:
                        self.get_logger().warn(f"Queue full for {manager_name} ({wheel_desc})")
            
            # Логирование статуса отправки
            if successful_commands < len(port_commands):
                self.get_logger().warn(
                    f"Sent {successful_commands}/{len(port_commands)} commands",
                    throttle_duration_sec=1.0
                )
                        
        except Exception as e:
            self.get_logger().error(f"Ackermann calculation error: {str(e)}")

    def destroy_node(self):
        """Корректное завершение работы всех портов"""
        self.get_logger().info("Shutting down - stopping all port managers...")
        
        # Остановка всех менеджеров портов
        for name, manager in self.port_managers.items():
            try:
                manager.stop()
                self.get_logger().info(f"Stopped port manager: {name}")
            except Exception as e:
                self.get_logger().error(f"Error stopping {name}: {str(e)}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = ServoWheelController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print(f"\nЗавершение работы... Сервоприводы возвращаются в начальное положение")
    except Exception as e:
        controller.get_logger().error(f"Critical error: {str(e)}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()