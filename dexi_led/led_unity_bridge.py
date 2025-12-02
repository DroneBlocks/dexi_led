#!/usr/bin/env python3
"""
LED Unity Bridge for DEXI Drone
Mirrors DEXI LED service state to a topic that Unity can subscribe to
This allows real-time LED visualization in Unity for testing effects
"""

import rclpy
from rclpy.node import Node
from dexi_interfaces.msg import LEDStateArray, LEDState
from dexi_interfaces.srv import LEDPixelColor, LEDRingColor, LEDEffect
import threading
import time
import colorsys

class LEDUnityBridge(Node):
    def __init__(self):
        super().__init__('led_unity_bridge')

        # Declare parameters to match your LED service
        self.declare_parameter('led_count', 45)
        self.declare_parameter('brightness', 0.2)
        self.declare_parameter('publish_rate', 15.0)  # Hz

        self.led_count = self.get_parameter('led_count').value
        self.global_brightness = int(self.get_parameter('brightness').value * 255)
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publisher for LED state to Unity
        self.led_state_pub = self.create_publisher(
            LEDStateArray,
            '/dexi/led_state',
            10
        )

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
        self.state_lock = threading.Lock()

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

        # Timer to publish LED state at regular intervals
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_led_state)

        # Start with galaxy effect like your real LED service
        self.start_effect('galaxy')

        self.get_logger().info(f'LED Unity Bridge started - {self.led_count} LEDs publishing at {self.publish_rate} Hz')

    def publish_led_state(self):
        """Publish current LED state to Unity"""
        msg = LEDStateArray()
        msg.leds = []

        with self.state_lock:
            for i in range(self.led_count):
                led_state = LEDState()
                led_state.index = i
                led_state.r = self.led_states[i][0]
                led_state.g = self.led_states[i][1]
                led_state.b = self.led_states[i][2]
                led_state.brightness = self.global_brightness
                msg.leds.append(led_state)

        self.led_state_pub.publish(msg)

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
        with self.state_lock:
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
        with self.state_lock:
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
            'gradient': self.gradient_effect,
            'wave': self.wave_effect,
            'snake': self.snake_effect,
            'festive': self.festive_effect,
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
                with self.state_lock:
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

                with self.state_lock:
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
            with self.state_lock:
                self.led_states = [red_color] * self.led_count
            time.sleep(1.0)

            if not self.effect_running:
                break

            # All off
            with self.state_lock:
                self.led_states = [(0, 0, 0)] * self.led_count
            time.sleep(1.0)

    def meteor_effect(self):
        """Simplified meteor effect"""
        import random
        while self.effect_running:
            color = (random.randint(100, 255), random.randint(100, 255), random.randint(100, 255))
            for i in range(self.led_count + 10):
                if not self.effect_running:
                    break
                with self.state_lock:
                    self.led_states = [(0, 0, 0)] * self.led_count
                    for j in range(10):
                        if 0 <= i - j < self.led_count:
                            intensity = 1 - (j / 10)
                            self.led_states[i - j] = (int(color[0] * intensity),
                                                    int(color[1] * intensity),
                                                    int(color[2] * intensity))
                time.sleep(0.05)

    def loading_effect(self):
        """Loading bar effect"""
        green = (0, 255, 0)
        while self.effect_running:
            for i in range(self.led_count):
                if not self.effect_running:
                    break
                with self.state_lock:
                    self.led_states = [(0, 0, 0)] * self.led_count
                    self.led_states[i] = green
                time.sleep(0.05)

    def ripple_effect(self):
        """Simplified ripple effect"""
        blue = (0, 0, 255)
        center = self.led_count // 2
        while self.effect_running:
            for radius in range(self.led_count // 2 + 10):
                if not self.effect_running:
                    break
                with self.state_lock:
                    self.led_states = [(0, 0, 0)] * self.led_count
                    for j in range(10):
                        intensity = 1 - (j / 10)
                        color = (int(blue[0] * intensity), int(blue[1] * intensity), int(blue[2] * intensity))
                        if 0 <= center - (radius - j) < self.led_count:
                            self.led_states[center - (radius - j)] = color
                        if 0 <= center + (radius - j) < self.led_count:
                            self.led_states[center + (radius - j)] = color
                time.sleep(0.05)

    def comet_effect(self):
        """Comet effect"""
        blue = (0, 0, 255)
        while self.effect_running:
            for i in range(self.led_count + 10):
                if not self.effect_running:
                    break
                with self.state_lock:
                    self.led_states = [(0, 0, 0)] * self.led_count
                    for j in range(10):
                        if 0 <= i - j < self.led_count:
                            intensity = 1 - (j / 10)
                            self.led_states[i - j] = (int(blue[0] * intensity),
                                                    int(blue[1] * intensity),
                                                    int(blue[2] * intensity))
                time.sleep(0.05)

    def gradient_effect(self):
        """Creates a gradient effect"""
        hue_increment = 0.01
        hue = 0.0
        delay = 0.05

        while self.effect_running: 
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            color = (int(r*255), int(g*255), int(b*255))

            with self.state_lock:
                self.led_states = [color] * self.led_count

            hue += hue_increment   
            hue %= 1.0            

            time.sleep(delay)

    def wave_effect(self):
        """Creates a wave the propagates from the center"""
        delay = 0.1
        colors = [
            (255, 0, 0),    # Red
            (255, 127, 0),  # Orange
            (255, 255, 0),  # Yellow
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (75, 0, 130),   # Indigo
            (148, 0, 211)   # Violet
        ]

        while self.effect_running:
            for i in range(len(colors)):
                for j in range((self.led_count // 2) + 1): 
                    if not self.effect_running:
                        break   
                    
                    self.led_states[j] = colors[i]
                    if not j == 0:
                        self.led_states[self.led_count - j] = colors[i]

                    time.sleep(delay)
                

    def snake_effect(self):
        """Creates a gradient snake effect"""
        hue_increment = 0.00667
        hue = 0.0
        delay = 0.05
        
        while self.effect_running:
            for i in range(self.led_count):
                if not self.effect_running:
                    break
                r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
                color = (int(r*255), int(g*255), int(b*255))
                
                self.led_states[i] = color

                hue += hue_increment   
                hue %= 1.0            

                time.sleep(delay)

    def festive_effect(self):
        """Creates alterating festive lights"""
        delay = 0.4
        ring_state_even = []
        ring_state_odd = []
        colors = [
            (0, 255, 0),    # Green,
            (255, 0, 0),    # Red
        ]
        
        for i in range(self.led_count):
            if (i // 2) % 2 == 0: ring_state_even.append(colors[0])
            else: ring_state_even.append(colors[1])

        ring_state_odd = ring_state_even[2:] + ring_state_even[:2]

        while self.effect_running:
            with self.state_lock:
                for i in range(self.led_count):
                    self.led_states[i] = ring_state_even[i]

            time.sleep(delay)

            if not self.effect_running:
                break

            with self.state_lock:
                for i in range(self.led_count):
                    self.led_states[i] = ring_state_odd[i]

            time.sleep(delay)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.stop_current_effect()
        super().destroy_node()

def main():
    rclpy.init()

    try:
        bridge = LEDUnityBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
