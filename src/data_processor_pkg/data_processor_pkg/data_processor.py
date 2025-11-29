#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        
        # /sensor_value konusuna abone ol (Dinleyici)
        self.subscription = self.create_subscription(
            Float32,
            '/sensor_value',
            self.listener_callback,
            10)
        self.subscription  # unused variable warning engellemek icin
        
        # /processed_value konusuna yayıncı (Publisher)
        self.publisher_ = self.create_publisher(Float32, '/processed_value', 10)
        
        self.get_logger().info('Data Processor baslatildi.')

    def listener_callback(self, msg):
        # Gelen veriyi al
        original_data = msg.data
        
        # VERİ İŞLEME ADIMI: Değeri 2 ile çarp
        processed_data = original_data * 2.0
        
        # Yeni mesajı oluştur ve yayınla
        new_msg = Float32()
        new_msg.data = processed_data
        
        self.publisher_.publish(new_msg)
        
        # Log çıktısı
        self.get_logger().info(f'Alinan: {original_data:.2f} -> Islenen: {processed_data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
