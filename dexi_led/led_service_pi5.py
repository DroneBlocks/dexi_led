#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import LEDPixelColor, LEDRingColor, LEDEffect
from std_msgs.msg import String
from pi5neo import Pi5Neo
import threading
import time
import random
import sys

class LEDService(Node):
    def __init__(self):
        super().__init__('led_service')
        
        # Declare parameters
        self.declare_parameter('led_count', 78)
        self.declare_parameter('brightness', 0.2)
        self.declare_parameter('spi_speed', 800)
        self.declare_parameter('simulation_mode', False)
        
        self.led_count = self.get_parameter('led_count').value
        self.brightness = self.get_parameter('brightness').value
        self.spi_speed = self.get_parameter('spi_speed').value
        self.simulation_mode = self.get_parameter('simulation_mode').value

        # Add effect control variables
        self.effect_thread = None
        self.effect_running = False
        self._is_shutting_down = False
        
        if not self.simulation_mode:
            self.strip = Pi5Neo('/dev/spidev1.0', self.led_count, self.spi_speed)
            self.strip.fill_strip(0, 0, 0)
            self.strip.update_strip()
        else:
            self.get_logger().info('Simulation mode not implemented for Python version')
            
        # Create services and subscribers
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

        # Create effect service
        self.effect_service = self.create_service(
            LEDEffect,
            '~/set_led_effect',
            self.effect_callback
        )

        # Start galaxy spiral effect automatically
        self.effect_running = True
        self.effect_thread = threading.Thread(target=self.galaxy_spiral_effect)
        self.effect_thread.start()
        self.get_logger().info("Started initial galaxy spiral effect")

        # Register shutdown callback
        self.get_logger().info('LED Service initialized successfully')

    def cleanup(self):
        """Clean up resources"""
        if self._is_shutting_down:
            return
            
        self._is_shutting_down = True
        print('Cleaning up LED resources...')
        
        # Stop any running effect
        self.stop_current_effect()
        
        # Turn off LEDs
        if not self.simulation_mode and hasattr(self, 'strip'):
            try:
                self.strip.fill_strip(0, 0, 0)
                self.strip.update_strip()
            except Exception as e:
                print(f'Error during LED cleanup: {str(e)}')

    def rainbow_effect(self):
        colors = [
            (255, 0, 0),    # Red
            (255, 127, 0),  # Orange
            (255, 255, 0),  # Yellow
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (75, 0, 130),   # Indigo
            (148, 0, 211)   # Violet
        ]
        
        num_colors = len(colors)
        while self.effect_running:
            for offset in range(self.led_count):
                if not self.effect_running:
                    break
                for i in range(self.led_count):
                    self.strip.set_led_color(i, *colors[(i + offset) % num_colors])
                self.strip.update_strip()
                time.sleep(0.05)

    def meteor_effect(self):
        """Creates a meteor shower effect with random colors and positions"""
        meteor_length = 10
        delay = 0.1
        
        while self.effect_running:
            meteor_start = random.randint(0, self.led_count - 1)
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            
            # Meteor falling animation
            for i in range(meteor_start, meteor_start + meteor_length):
                if not self.effect_running:
                    break
                    
                self.strip.fill_strip(0, 0, 0)
                for j in range(meteor_length):
                    if i - j >= 0 and i - j < self.led_count:
                        # Calculate fading intensity based on position
                        fade = 1 - (j / meteor_length)
                        r = int(color[0] * fade)
                        g = int(color[1] * fade)
                        b = int(color[2] * fade)
                        self.strip.set_led_color(i - j, r, g, b)
                
                self.strip.update_strip()
                time.sleep(delay)

    def ripple_effect(self):
        """Creates a ripple effect with color waves moving outward from center"""
        delay = 0.1
        color = (0, 0, 255)  # Blue color
        center = self.led_count // 2  # Center of the strip
        ripple_length = 10  # Length of the ripple wave
        
        while self.effect_running:
            for radius in range(self.led_count // 2 + ripple_length):
                if not self.effect_running:
                    break
                    
                self.strip.fill_strip(0, 0, 0)
                # Create fading ripple on both sides of center
                for j in range(ripple_length):
                    # Calculate fading intensity
                    intensity = 1 - (j / ripple_length)
                    r = int(color[0] * intensity)
                    g = int(color[1] * intensity)
                    b = int(color[2] * intensity)
                    
                    # Set LEDs symmetrically from center with fade
                    if center - (radius - j) >= 0:
                        self.strip.set_led_color(center - (radius - j), r, g, b)
                    if center + (radius - j) < self.led_count:
                        self.strip.set_led_color(center + (radius - j), r, g, b)
                
                self.strip.update_strip()
                time.sleep(delay)

    def comet_effect(self):
        """Creates a comet effect with a trailing fade"""
        delay = 0.1
        color = (0, 0, 255)  # Blue color
        trail_length = 10  # Length of the comet trail
        
        while self.effect_running:
            for i in range(self.led_count + trail_length):
                if not self.effect_running:
                    break
                    
                self.strip.fill_strip(0, 0, 0)  # Clear the strip
                for j in range(trail_length):
                    if 0 <= i - j < self.led_count:
                        # Calculate fading intensity
                        intensity = 1 - (j / trail_length)
                        r = int(color[0] * intensity)
                        g = int(color[1] * intensity)
                        b = int(color[2] * intensity)
                        self.strip.set_led_color(i - j, r, g, b)
                
                self.strip.update_strip()
                time.sleep(delay)

    def loading_effect(self):
        """Creates a loading bar effect with a single moving LED"""
        delay = 0.1
        color = (0, 255, 0)  # Green color
        
        while self.effect_running:
            for i in range(self.led_count):
                if not self.effect_running:
                    break
                    
                self.strip.fill_strip(0, 0, 0)  # Clear the strip
                self.strip.set_led_color(i, *color)  # Set current LED to green
                self.strip.update_strip()
                time.sleep(delay)

    def galaxy_spiral_effect(self):
        """Creates a spiral galaxy effect with multiple rotating color arms"""
        delay = 0.05
        # Define multiple color arms for the spiral
        arms = [
            (255, 0, 255),    # Purple
            (0, 100, 255),    # Light Blue
            (255, 255, 255),  # White
            (255, 50, 0)      # Orange-Red
        ]
        num_arms = len(arms)
        spacing = self.led_count // num_arms
        fade_length = spacing // 2
        
        while self.effect_running:
            for offset in range(self.led_count):
                if not self.effect_running:
                    break
                
                self.strip.fill_strip(0, 0, 0)
                
                # Create multiple rotating arms
                for arm_idx, color in enumerate(arms):
                    # Calculate the starting point for each arm
                    arm_offset = (offset + (arm_idx * spacing)) % self.led_count
                    
                    # Create the spiral arm with fading
                    for i in range(fade_length):
                        # Calculate two positions: forward and backward from the arm center
                        pos_forward = (arm_offset + i) % self.led_count
                        pos_backward = (arm_offset - i) % self.led_count
                        
                        # Calculate fade intensity based on distance from arm center
                        intensity = 1 - (i / fade_length)
                        # Add some shimmer with random variation
                        intensity *= (0.85 + random.random() * 0.15)
                        
                        r = int(color[0] * intensity)
                        g = int(color[1] * intensity)
                        b = int(color[2] * intensity)
                        
                        # Set LEDs on both sides of the arm center
                        self.strip.set_led_color(pos_forward, r, g, b)
                        self.strip.set_led_color(pos_backward, r, g, b)
                
                self.strip.update_strip()
                time.sleep(delay)

    def red_flash_effect(self):
        """Creates a red flashing effect with 1-second delays"""
        self.blink_effect("red", 1.0)

    def blink_effect(self, color_name, delay=0.5):
        """Creates a blinking effect with specified color and delay"""
        color_map = {
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
        
        if color_name not in color_map:
            self.get_logger().error(f"Unknown color: {color_name}")
            return
            
        color = color_map[color_name]
        
        while self.effect_running:
            if not self.effect_running:
                break
                
            # Turn all LEDs to specified color
            self.strip.fill_strip(*color)
            self.strip.update_strip()
            time.sleep(delay)
            
            if not self.effect_running:
                break
                
            # Turn all LEDs off
            self.strip.fill_strip(0, 0, 0)
            self.strip.update_strip()
            time.sleep(delay)

    def stop_current_effect(self):
        self.effect_running = False
        if self.effect_thread is not None:
            self.effect_thread.join()
            self.effect_thread = None

    def effect_callback(self, request, response):
        """Service callback for setting LED effects"""
        self.get_logger().info(f'Setting LED effect to: {request.effect_name}')

        try:
            # Stop any currently running effect
            self.stop_current_effect()

            if request.effect_name.lower() == 'rainbow':
                self.effect_running = True
                self.effect_thread = threading.Thread(target=self.rainbow_effect)
                self.effect_thread.start()
                self.get_logger().info("Started rainbow effect")
                response.success = True
                response.message = "Successfully started rainbow effect"
            elif request.effect_name.lower() == 'meteor':
                self.effect_running = True
                self.effect_thread = threading.Thread(target=self.meteor_effect)
                self.effect_thread.start()
                self.get_logger().info("Started meteor effect")
                response.success = True
                response.message = "Successfully started meteor effect"
            elif request.effect_name.lower() == 'loading':
                self.effect_running = True
                self.effect_thread = threading.Thread(target=self.loading_effect)
                self.effect_thread.start()
                self.get_logger().info("Started loading effect")
                response.success = True
                response.message = "Successfully started loading effect"
            elif request.effect_name.lower() == 'ripple':
                self.effect_running = True
                self.effect_thread = threading.Thread(target=self.ripple_effect)
                self.effect_thread.start()
                self.get_logger().info("Started ripple effect")
                response.success = True
                response.message = "Successfully started ripple effect"
            elif request.effect_name.lower() == 'comet':
                self.effect_running = True
                self.effect_thread = threading.Thread(target=self.comet_effect)
                self.effect_thread.start()
                self.get_logger().info("Started comet effect")
                response.success = True
                response.message = "Successfully started comet effect"
            elif request.effect_name.lower() == 'galaxy':
                self.effect_running = True
                self.effect_thread = threading.Thread(target=self.galaxy_spiral_effect)
                self.effect_thread.start()
                self.get_logger().info("Started galaxy spiral effect")
                response.success = True
                response.message = "Successfully started galaxy spiral effect"
            elif request.effect_name.lower() == 'red_flash':
                self.effect_running = True
                self.effect_thread = threading.Thread(target=self.red_flash_effect)
                self.effect_thread.start()
                self.get_logger().info("Started red flash effect")
                response.success = True
                response.message = "Successfully started red flash effect"
            elif request.effect_name.lower().startswith('blink_'):
                # Extract color from effect name (e.g., "blink_cyan" -> "cyan")
                color_name = request.effect_name.lower().replace('blink_', '')
                self.effect_running = True
                self.effect_thread = threading.Thread(target=self.blink_effect, args=(color_name,))
                self.effect_thread.start()
                self.get_logger().info(f"Started {color_name} blink effect")
                response.success = True
                response.message = f"Successfully started {color_name} blink effect"
            elif request.effect_name.lower() == 'stop':
                self.get_logger().info("Stopped current effect")
                response.success = True
                response.message = "Successfully stopped effect"
            else:
                self.get_logger().warning("Unknown effect requested")
                response.success = False
                response.message = "Unknown effect requested"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to set effect: {str(e)}"
            self.get_logger().error(f"Failed to set effect: {str(e)}")
        
        return response

    def set_led_pixel_callback(self, request, response):
        self.get_logger().info(f'Setting pixel {request.index} to RGB({request.r}, {request.g}, {request.b})')

        try:
            if request.index >= self.led_count:
                response.success = False
                response.message = "LED index out of bounds"
                return response

            self.strip.set_led_color(request.index, request.r, request.g, request.b)
            self.strip.update_strip()
            
            response.success = True
            response.message = "Successfully set LED pixel color"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to update LED strip: {str(e)}"
            
        return response

    def set_led_ring_callback(self, request, response):
        self.get_logger().info(f'Setting ring color to: {request.color}')

        color_map = {
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

        if request.color not in color_map:
            response.success = False
            response.message = "Color not available"
            return response

        try:
            # Stop any running effect first
            self.stop_current_effect()
            
            r, g, b = color_map[request.color]
            self.strip.fill_strip(r, g, b)
            self.strip.update_strip()
            
            response.success = True
            response.message = "Successfully set LED ring color"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to update LED strip: {str(e)}"
            
        return response

    def destroy_node(self):
        """Override destroy_node to ensure cleanup"""
        self.cleanup()
        super().destroy_node()

def main():
    try:
        rclpy.init()
        node = LEDService()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print('Keyboard interrupt received, shutting down...')
            node.cleanup()
            node.destroy_node()
            sys.exit(0)
        except Exception as e:
            print(f'Error during execution: {str(e)}')
            node.cleanup()
            node.destroy_node()
            sys.exit(1)
    except Exception as e:
        print(f'Error initializing node: {str(e)}')
        sys.exit(1)

if __name__ == '__main__':
    main()