#!/usr/bin/env python3
#robot_driver_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct
import time 
import math

class RobotDriverNode(Node):
    def __init__(self):
        super().__init__('robot_driver_node')
        
        # Параметры подключения
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        
        # Подписчики - получаем готовые команды для колес
        self.wheel_commands_sub = self.create_subscription(
            Float32MultiArray,
            '/wheel_commands',
            self.wheel_commands_callback,
            10)
        
        # Резервный подписчик для обратной совместимости
        self.motion_sub = self.create_subscription(
            Float32MultiArray,
            '/motion_commands',
            self.motion_commands_callback,
            10)
        
        # Serial порты для 6 колес - ИСПРАВЛЕННЫЕ ПУТИ
        self.steering_ports = {
            'front_right': '/dev/ttyROVER_WHEEL_1',
            'rear_right': '/dev/ttyROVER_WHEEL_3', 
            'front_left': '/dev/ttyROVER_WHEEL_4',
            'rear_left': '/dev/ttyROVER_WHEEL_6'
        }
        
        self.center_ports = {
            'right_center': '/dev/ttyROVER_WHEEL_2',
            'left_center': '/dev/ttyROVER_WHEEL_5'
        }
        
        self.serial_connections = {}
        self.center_connections = {}
        
        # Статистика
        self.command_count = 0
        self.error_count = 0
        
        # Подключение портов
        self.connect_serial_ports()
        
        # Таймер безопасности
        self.last_command_time = time.time()
        self.command_timeout = 1.0
        self.create_timer(0.05, self.safety_check)
        
        # Таймер проверки соединения
        self.create_timer(2.0, self.connection_check)
        
        self.get_logger().info("Robot driver node initialized")
        self.get_logger().info(f"Connected: {len(self.serial_connections)} steering, "
                             f"{len(self.center_connections)} center wheels")

    def connect_serial_ports(self):
        """Подключение ко всем serial портам"""
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        # Поворотные колеса
        for port_name, port_path in self.steering_ports.items():
            self.connect_single_port(port_name, port_path, self.serial_connections, baudrate, timeout)
        
        # Центральные колеса
        for port_name, port_path in self.center_ports.items():
            self.connect_single_port(port_name, port_path, self.center_connections, baudrate, timeout)

    def connect_single_port(self, port_name, port_path, connection_dict, baudrate, timeout):
        """Подключение одного serial порта"""
        if port_name not in connection_dict:
            try:
                ser = serial.Serial(port_path, baudrate=baudrate, 
                                  timeout=timeout, write_timeout=0.1)
                connection_dict[port_name] = {
                    'serial': ser,
                    'path': port_path,
                    'errors': 0
                }
                self.get_logger().info(f"✓ {port_name}: {port_path}")
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().warning(f"✗ {port_name} ({port_path}): {str(e)}")

    def wheel_commands_callback(self, msg):
        """
        Основной callback - получаем готовые команды для колес
        Ожидаемый формат: [speed_fr, angle_fr, speed_rr, angle_rr, speed_fl, angle_fl, speed_rl, angle_rl, speed_rc, speed_lc]
        """
        self.last_command_time = time.time()
        self.command_count += 1
        
        if len(msg.data) != 10:
            self.get_logger().error(f"Invalid wheel commands format: expected 10 values, got {len(msg.data)}")
            return
            
        try:
            # Распаковываем команды для каждого колеса
            commands = {
                'front_right': {'speed': float(msg.data[0]), 'angle': int(msg.data[1])},
                'rear_right': {'speed': float(msg.data[2]), 'angle': int(msg.data[3])},
                'front_left': {'speed': float(msg.data[4]), 'angle': int(msg.data[5])},
                'rear_left': {'speed': float(msg.data[6]), 'angle': int(msg.data[7])},
                'right_center': {'speed': float(msg.data[8]), 'angle': 0},
                'left_center': {'speed': float(msg.data[9]), 'angle': 0}
            }
            
            self.send_to_wheels(commands)
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"Wheel commands processing error: {str(e)}")

    def motion_commands_callback(self, msg):
        """
        Резервный callback для обратной совместимости
        Преобразует старый формат [speed, angle] в команды для колес
        """
        self.last_command_time = time.time()
        
        if len(msg.data) != 2:
            return
            
        try:
            V = float(msg.data[0])
            steering_angle = float(msg.data[1])
            
            # Простое преобразование для тестирования
            # В реальной системе это должно делаться отдельной нодой
            commands = {
                'front_right': {'speed': V, 'angle': int(math.degrees(steering_angle))},
                'rear_right': {'speed': V, 'angle': -int(math.degrees(steering_angle))},
                'front_left': {'speed': V, 'angle': int(math.degrees(steering_angle))},
                'rear_left': {'speed': V, 'angle': -int(math.degrees(steering_angle))},
                'right_center': {'speed': V, 'angle': 0},
                'left_center': {'speed': V, 'angle': 0}
            }
            
            self.send_to_wheels(commands)
            self.get_logger().warn("Using legacy motion commands - consider switching to wheel_commands")
            
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"Motion commands processing error: {str(e)}")

    def send_to_wheels(self, commands):
        """Отправка команд на все колеса"""
        success_count = 0
        
        # Поворотные колеса (скорость + угол)
        for port_name, connection_info in self.serial_connections.items():
            if port_name in commands:
                try:
                    ser = connection_info['serial']
                    if ser.is_open:
                        cmd = commands[port_name]
                        packed_data = struct.pack('<fh', cmd['speed'], cmd['angle'])
                        ser.write(packed_data)
                        ser.flush()
                        success_count += 1
                        connection_info['errors'] = 0
                except Exception as e:
                    connection_info['errors'] += 1
                    if connection_info['errors'] <= 3:
                        self.get_logger().warning(f"Steering wheel {port_name}: {str(e)}")
        
        # Центральные колеса (только скорость)
        for port_name, connection_info in self.center_connections.items():
            if port_name in commands:
                try:
                    ser = connection_info['serial']
                    if ser.is_open:
                        cmd = commands[port_name]
                        packed_data = struct.pack('<fh', cmd['speed'], cmd['angle'])
                        ser.write(packed_data)
                        ser.flush()
                        success_count += 1
                        connection_info['errors'] = 0
                except Exception as e:
                    connection_info['errors'] += 1
                    if connection_info['errors'] <= 3:
                        self.get_logger().warning(f"Center wheel {port_name}: {str(e)}")
        
        # Логирование каждые 10 команд
        if self.command_count % 10 == 0:
            self.get_logger().info(
                f"Commands sent: {success_count}/6 wheels | "
                f"Total: {self.command_count} | Errors: {self.error_count}"
            )

    def safety_check(self):
        """Проверка безопасности - остановка при потере связи"""
        time_since_last_command = time.time() - self.last_command_time
        if time_since_last_command > self.command_timeout:
            # Отправляем нулевые команды
            zero_commands = {
                'front_right': {'speed': 0.0, 'angle': 0},
                'rear_right': {'speed': 0.0, 'angle': 0},
                'front_left': {'speed': 0.0, 'angle': 0},
                'rear_left': {'speed': 0.0, 'angle': 0},
                'right_center': {'speed': 0.0, 'angle': 0},
                'left_center': {'speed': 0.0, 'angle': 0}
            }
            self.send_to_wheels(zero_commands)
            
            if self.command_count % 20 == 0:
                self.get_logger().warning("Safety stop: no commands received")

    def connection_check(self):
        """Периодическая проверка и восстановление соединений"""
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        # Проверка поворотных колес
        for port_name in list(self.serial_connections.keys()):
            conn = self.serial_connections[port_name]
            if conn['errors'] > 5 or not conn['serial'].is_open:
                self.get_logger().warning(f"Reconnecting {port_name}...")
                try:
                    conn['serial'].close()
                except:
                    pass
                del self.serial_connections[port_name]
                self.connect_single_port(port_name, self.steering_ports[port_name], 
                                       self.serial_connections, baudrate, timeout)
        
        # Проверка центральных колес
        for port_name in list(self.center_connections.keys()):
            conn = self.center_connections[port_name]
            if conn['errors'] > 5 or not conn['serial'].is_open:
                self.get_logger().warning(f"Reconnecting {port_name}...")
                try:
                    conn['serial'].close()
                except:
                    pass
                del self.center_connections[port_name]
                self.connect_single_port(port_name, self.center_ports[port_name],
                                       self.center_connections, baudrate, timeout)

    def destroy_node(self):
        """Корректное завершение работы"""
        self.get_logger().info("Shutting down driver node...")
        
        # Остановка всех колес
        zero_commands = {
            'front_right': {'speed': 0.0, 'angle': 0},
            'rear_right': {'speed': 0.0, 'angle': 0},
            'front_left': {'speed': 0.0, 'angle': 0},
            'rear_left': {'speed': 0.0, 'angle': 0},
            'right_center': {'speed': 0.0, 'angle': 0},
            'left_center': {'speed': 0.0, 'angle': 0}
        }
        self.send_to_wheels(zero_commands)
        time.sleep(0.1)
        
        # Закрытие соединений
        for conn_dict in [self.serial_connections, self.center_connections]:
            for port_name, connection_info in conn_dict.items():
                try:
                    connection_info['serial'].close()
                    self.get_logger().info(f"Closed {port_name}")
                except Exception as e:
                    self.get_logger().error(f"Error closing {port_name}: {str(e)}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    driver = RobotDriverNode()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.get_logger().info("Driver stopped by user")
    except Exception as e:
        driver.get_logger().error(f"Driver error: {str(e)}")
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()