#!/usr/bin/env python3
"""
LED Visualization Bridge for RViz
Mirrors DEXI LED service calls to RViz markers for SITL simulation visualization
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from dexi_interfaces.srv import LEDPixelColor, LEDRingColor, LEDEffect
import math
import threading
import time

class LEDVisualizationBridge(Node):
    def __init__(self):
        super().__init__('led_visualization_bridge')
        
        # Declare parameters to match your LED service
        self.declare_parameter('led_count', 78)
        self.declare_parameter('brightness', 0.2)
        self.declare_parameter('visualization_radius', 1.0)  # meters
        self.declare_parameter('visualization_height', 2.0)  # meters above ground
        self.declare_parameter('led_marker_size', 0.04)     # 4cm diameter spheres
        
        self.led_count = self.get_parameter('led_count').value
        self.brightness = self.get_parameter('brightness').value
        self.viz_radius = self.get_parameter('visualization_radius').value
        self.viz_height = self.get_parameter('visualization_height').value
        self.marker_size = self.get_parameter('led_marker_size').value
        
        # Publisher for LED markers
        self.marker_pub = self.create_publisher(MarkerArray, '/led_visualization', 10)
        
        # Track current LED states - RGB tuples
        self.led_states = [(0, 0, 0)] * self.led_count
        
        # Color mapping from your LED service
        self.color_map = {
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'yellow': (255, 255, 0),
            'purple': (255, 0, 255),
            'cyan': (0, 255, 255),
            'white': (255, 255, 255),
            'orange': (255, 127, 0),
            'teal': (0, 128, 128),
            'magenta': (255, 0, 128),
            'gold': (255, 215, 0),
            'pink': (255, 192, 203),
            'aqua': (0, 255, 255),
            'jade': (0, 168, 107),
            'amber': (255, 191, 0),
            'old_lace': (253, 245, 230),
            'black': (0, 0, 0)
        }
        
        # Effect control
        self.current_effect = None
        self.effect_running = False
        self.effect_thread = None
        
        # Services to mirror your LED service
        self.led_pixel_service = self.create_service(
            LEDPixelColor,
            '~/set_led_pixel_color',
            self.set_led_pixel_callback
        )
        
        self.led_ring_service = self.create_service(
            LEDRingColor,
            '~/set_led_ring_color',
            self.set_led_ring_callback
        )
        
        self.effect_service = self.create_service(
            LEDEffect,
            '~/set_led_effect',
            self.set_led_effect_callback
        )
        
        # Timer to publish markers at 10Hz
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        # Start with galaxy effect like your real LED service
        self.start_effect('galaxy')
        
        self.get_logger().info(f'LED Visualization Bridge started - {self.led_count} LEDs in {self.viz_radius}m radius circle at {self.viz_height}m height')
        
    def publish_markers(self):
        """Publish LED markers to RViz"""
        marker_array = MarkerArray()
        
        for i in range(self.led_count):
            marker = Marker()
            marker.header.frame_id = "map"  # Use map frame for fixed position
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dexi_led_ring"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position LEDs in circle
            angle = (i / self.led_count) * 2 * math.pi
            marker.pose.position.x = self.viz_radius * math.cos(angle)
            marker.pose.position.y = self.viz_radius * math.sin(angle)
            marker.pose.position.z = self.viz_height
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Size
            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size
            
            # Color from LED state with brightness applied
            r, g, b = self.led_states[i]
            marker.color.r = (r / 255.0) * self.brightness
            marker.color.g = (g / 255.0) * self.brightness
            marker.color.b = (b / 255.0) * self.brightness
            marker.color.a = 1.0 if (r + g + b) > 0 else 0.1  # dim when off
            
            marker.lifetime.sec = 0  # persistent
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def set_led_pixel_callback(self, request, response):
        """Mirror your LED pixel service"""
        self.get_logger().info(f'Setting pixel {request.index} to RGB({request.r}, {request.g}, {request.b})')
        
        if request.index >= self.led_count:
            response.success = False
            response.message = "LED index out of bounds"
            return response
        
        # Stop any running effect
        self.stop_current_effect()
        
        # Update LED state
        self.led_states[request.index] = (request.r, request.g, request.b)
        
        response.success = True
        response.message = "Successfully set LED pixel color"
        return response
    
    def set_led_ring_callback(self, request, response):
        """Mirror your LED ring service"""
        self.get_logger().info(f'Setting ring color to: {request.color}')
        
        if request.color not in self.color_map:
            response.success = False
            response.message = "Color not available"
            return response
        
        # Stop any running effect
        self.stop_current_effect()
        
        # Set all LEDs to the specified color
        color_rgb = self.color_map[request.color]
        self.led_states = [color_rgb] * self.led_count
        
        response.success = True
        response.message = "Successfully set LED ring color"
        return response
    
    def set_led_effect_callback(self, request, response):
        """Mirror your LED effect service"""
        self.get_logger().info(f'Setting LED effect to: {request.effect_name}')
        
        try:
            self.start_effect(request.effect_name.lower())
            response.success = True
            response.message = f"Successfully started {request.effect_name} effect"
        except Exception as e:
            response.success = False
            response.message = f"Failed to start effect: {str(e)}"
            self.get_logger().error(f"Failed to start effect: {str(e)}")
        
        return response
    
    def start_effect(self, effect_name):
        """Start the specified effect"""
        self.stop_current_effect()
        
        effect_methods = {
            'rainbow': self.rainbow_effect,
            'meteor': self.meteor_effect,
            'loading': self.loading_effect,
            'ripple': self.ripple_effect,
            'comet': self.comet_effect,
            'galaxy': self.galaxy_spiral_effect,
            'red_flash': self.red_flash_effect,
        }
        
        if effect_name in effect_methods:
            self.current_effect = effect_name
            self.effect_running = True
            self.effect_thread = threading.Thread(target=effect_methods[effect_name])
            self.effect_thread.daemon = True
            self.effect_thread.start()
            self.get_logger().info(f"Started {effect_name} effect")
        elif effect_name == 'stop':
            self.get_logger().info("Stopped current effect")
        else:
            self.get_logger().warning(f"Unknown effect: {effect_name}")
    
    def stop_current_effect(self):
        """Stop any currently running effect"""
        self.effect_running = False
        if self.effect_thread is not None:
            self.effect_thread.join(timeout=1.0)
            self.effect_thread = None
    
    # Effect implementations matching your LED service
    def rainbow_effect(self):
        """Rainbow effect"""
        colors = [
            (255, 0, 0), (255, 127, 0), (255, 255, 0), (0, 255, 0),
            (0, 0, 255), (75, 0, 130), (148, 0, 211)
        ]
        num_colors = len(colors)
        
        while self.effect_running:
            for offset in range(self.led_count):
                if not self.effect_running:
                    break
                for i in range(self.led_count):
                    self.led_states[i] = colors[(i + offset) % num_colors]
                time.sleep(0.05)
    
    def galaxy_spiral_effect(self):
        """Galaxy spiral effect matching your implementation"""
        arms = [
            (255, 0, 255),    # Purple
            (0, 100, 255),    # Light Blue  
            (255, 255, 255),  # White
            (255, 50, 0)      # Orange-Red
        ]
        num_arms = len(arms)
        spacing = self.led_count // num_arms
        fade_length = spacing // 2
        
        import random
        
        while self.effect_running:
            for offset in range(self.led_count):
                if not self.effect_running:
                    break
                
                # Clear all LEDs
                self.led_states = [(0, 0, 0)] * self.led_count
                
                # Create multiple rotating arms
                for arm_idx, color in enumerate(arms):
                    arm_offset = (offset + (arm_idx * spacing)) % self.led_count
                    
                    for i in range(fade_length):
                        pos_forward = (arm_offset + i) % self.led_count
                        pos_backward = (arm_offset - i) % self.led_count
                        
                        intensity = 1 - (i / fade_length)
                        intensity *= (0.85 + random.random() * 0.15)
                        
                        r = int(color[0] * intensity)
                        g = int(color[1] * intensity)
                        b = int(color[2] * intensity)
                        
                        self.led_states[pos_forward] = (r, g, b)
                        self.led_states[pos_backward] = (r, g, b)
                
                time.sleep(0.05)
    
    def red_flash_effect(self):
        """Red flash effect"""
        red_color = (255, 0, 0)
        
        while self.effect_running:
            if not self.effect_running:
                break
            
            # All red
            self.led_states = [red_color] * self.led_count
            time.sleep(1.0)
            
            if not self.effect_running:
                break
            
            # All off
            self.led_states = [(0, 0, 0)] * self.led_count
            time.sleep(1.0)
    
    # Simplified versions of other effects for brevity
    def meteor_effect(self):
        """Simplified meteor effect"""
        import random
        while self.effect_running:
            color = (random.randint(100, 255), random.randint(100, 255), random.randint(100, 255))
            for i in range(self.led_count + 10):
                if not self.effect_running:
                    break
                self.led_states = [(0, 0, 0)] * self.led_count
                for j in range(10):
                    if 0 <= i - j < self.led_count:
                        intensity = 1 - (j / 10)
                        self.led_states[i - j] = (int(color[0] * intensity), 
                                                int(color[1] * intensity), 
                                                int(color[2] * intensity))
                time.sleep(0.1)
    
    def loading_effect(self):
        """Loading bar effect"""
        green = (0, 255, 0)
        while self.effect_running:
            for i in range(self.led_count):
                if not self.effect_running:
                    break
                self.led_states = [(0, 0, 0)] * self.led_count
                self.led_states[i] = green
                time.sleep(0.1)
    
    def ripple_effect(self):
        """Simplified ripple effect"""
        blue = (0, 0, 255)
        center = self.led_count // 2
        while self.effect_running:
            for radius in range(self.led_count // 2 + 10):
                if not self.effect_running:
                    break
                self.led_states = [(0, 0, 0)] * self.led_count
                for j in range(10):
                    intensity = 1 - (j / 10)
                    color = (int(blue[0] * intensity), int(blue[1] * intensity), int(blue[2] * intensity))
                    if 0 <= center - (radius - j) < self.led_count:
                        self.led_states[center - (radius - j)] = color
                    if 0 <= center + (radius - j) < self.led_count:
                        self.led_states[center + (radius - j)] = color
                time.sleep(0.1)
    
    def comet_effect(self):
        """Comet effect"""
        blue = (0, 0, 255)
        while self.effect_running:
            for i in range(self.led_count + 10):
                if not self.effect_running:
                    break
                self.led_states = [(0, 0, 0)] * self.led_count
                for j in range(10):
                    if 0 <= i - j < self.led_count:
                        intensity = 1 - (j / 10)
                        self.led_states[i - j] = (int(blue[0] * intensity), 
                                                int(blue[1] * intensity), 
                                                int(blue[2] * intensity))
                time.sleep(0.1)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.stop_current_effect()
        super().destroy_node()

def main():
    rclpy.init()
    
    try:
        bridge = LEDVisualizationBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()