#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import LEDRingColor

class LEDRingColorClient(Node):
    def __init__(self):
        super().__init__('led_ring_color_client')
        self.client = self.create_client(LEDRingColor, '/led_service/set_led_ring_color')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def send_request(self, color):
        request = LEDRingColor.Request()
        request.color = color
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Service call successful: {future.result().message}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    
    # Create the client node
    client = LEDRingColorClient()
    
    # Example: Set the ring to red
    client.send_request('red')
    
    # Clean up
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 