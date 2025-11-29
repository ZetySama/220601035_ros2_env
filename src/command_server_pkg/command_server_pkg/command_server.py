#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Ozel servis mesajimizi import ediyoruz
from my_robot_interfaces.srv import ComputeCommand

class CommandServer(Node):
    def __init__(self):
        super().__init__('command_server')
        # /compute_command adinda bir servis olusturuyoruz
        self.srv = self.create_service(
            ComputeCommand, 
            'compute_command', 
            self.compute_callback
        )
        self.get_logger().info('Command Server hazir. Istek bekleniyor...')

    def compute_callback(self, request, response):
        # Mantik: Girdi 10'dan buyukse HIGH, degilse LOW
        if request.input > 10.0:
            response.output = "HIGH"
        else:
            response.output = "LOW"
            
        self.get_logger().info(f'Istek: {request.input} -> Cevap: {response.output}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CommandServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
