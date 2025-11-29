#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        # /sensor_value konusuna Float32 tipinde yayıncı oluşturuyoruz
        self.publisher_ = self.create_publisher(Float32, '/sensor_value', 10)
        
        # 0.1 saniyede bir çalışacak zamanlayıcı (10 Hz)
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Sensor Publisher baslatildi (0.1s periyot)')

    def timer_callback(self):
        msg = Float32()
        # 0 ile 20 arasında rastgele bir sayı üret
        msg.data = random.uniform(0.0, 20.0)
        
        self.publisher_.publish(msg)
        # Log çıktısı (opsiyonel ama takip için iyi)
        self.get_logger().info(f'Yayinlanan Veri: {msg.data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
