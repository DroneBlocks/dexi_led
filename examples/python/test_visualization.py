#!/usr/bin/env python3
"""
Test script for LED visualization bridge
Demonstrates various LED effects and colors for RViz testing
"""

import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import LEDRingColor, LEDPixelColor, LEDEffect
import time

class LEDVisualizationTest(Node):
    def __init__(self):
        super().__init__('led_visualization_test')
        
        # Create clients for the visualization bridge services
        self.ring_client = self.create_client(LEDRingColor, '/led_visualization_bridge/set_led_ring_color')
        self.pixel_client = self.create_client(LEDPixelColor, '/led_visualization_bridge/set_led_pixel_color')
        self.effect_client = self.create_client(LEDEffect, '/led_visualization_bridge/set_led_effect')
        
        # Wait for services
        self.get_logger().info('Waiting for LED visualization services...')
        self.ring_client.wait_for_service(timeout_sec=10.0)
        self.pixel_client.wait_for_service(timeout_sec=10.0)
        self.effect_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('All services connected!')
    
    def test_ring_colors(self):
        """Test various ring colors"""
        colors = ['red', 'green', 'blue', 'purple', 'white', 'gold']
        
        self.get_logger().info('Testing ring colors...')
        for color in colors:
            self.get_logger().info(f'Setting ring to {color}')
            request = LEDRingColor.Request()
            request.color = color
            future = self.ring_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(f'Response: {response.message}')
            time.sleep(2)
    
    def test_individual_pixels(self):
        """Test individual pixel control"""
        self.get_logger().info('Testing individual pixels...')
        
        # Set every 10th LED to a different color
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), 
                 (255, 0, 255), (0, 255, 255), (255, 255, 255)]
        
        for i, color in enumerate(colors):
            pixel_index = i * 10
            if pixel_index < 78:  # Stay within LED count
                request = LEDPixelColor.Request()
                request.index = pixel_index
                request.r = color[0]
                request.g = color[1] 
                request.b = color[2]
                
                future = self.pixel_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                self.get_logger().info(f'Set pixel {pixel_index} to RGB{color}: {response.message}')
                time.sleep(1)
    
    def test_effects(self):
        """Test various LED effects"""
        effects = ['rainbow', 'galaxy', 'comet', 'red_flash', 'loading']
        
        self.get_logger().info('Testing LED effects...')
        for effect in effects:
            self.get_logger().info(f'Starting {effect} effect')
            request = LEDEffect.Request()
            request.effect_name = effect
            
            future = self.effect_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(f'Effect response: {response.message}')
            time.sleep(5)  # Let effect run for 5 seconds
    
    def run_full_test(self):
        """Run the complete test sequence"""
        self.get_logger().info('Starting LED visualization test sequence')
        
        try:
            # Test 1: Ring colors
            self.test_ring_colors()
            time.sleep(2)
            
            # Test 2: Individual pixels  
            self.test_individual_pixels()
            time.sleep(3)
            
            # Test 3: Effects
            self.test_effects()
            
            self.get_logger().info('Test sequence completed!')
            
        except Exception as e:
            self.get_logger().error(f'Test failed: {str(e)}')

def main():
    rclpy.init()
    
    try:
        tester = LEDVisualizationTest()
        tester.run_full_test()
    except KeyboardInterrupt:
        pass
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()