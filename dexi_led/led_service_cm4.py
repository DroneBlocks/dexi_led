import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import LEDPixelColor, LEDRingColor, LEDEffect
import board
import neopixel
import threading
import time
from enum import Enum

from adafruit_led_animation.animation.solid import Solid
from adafruit_led_animation.color import (
    RED as red,
    YELLOW as yellow,
    ORANGE as orange,
    GREEN as green,
    TEAL as teal,
    CYAN as cyan,
    BLUE as blue,
    PURPLE as purple,
    MAGENTA as magenta,
    WHITE as white,
    BLACK as black,
    GOLD as gold,
    PINK as pink,
    AQUA as aqua,
    JADE as jade,
    AMBER as amber,
    OLD_LACE as old_lace
)

class LEDService(Node):

    def __init__(self):
        super().__init__('led_service')
        self.led_pixel_service = self.create_service(LEDPixelColor, '~/set_led_pixel_color', self.set_led_pixel_callback)
        self.led_color_service = self.create_service(LEDRingColor, '~/set_led_ring_color', self.set_led_ring_callback)
        self.effect_service = self.create_service(LEDEffect, '~/set_led_effect', self.effect_callback)
        
        self.pixel_pin = board.D12
        self.num_pixels = 45
        self.pixel_order = neopixel.GRB
        self.pixels = neopixel.NeoPixel(self.pixel_pin, self.num_pixels, brightness=0.2, auto_write=False, pixel_order=self.pixel_order)
        
        # Add effect control variables
        self.effect_thread = None
        self.effect_running = False
        self._is_shutting_down = False


    def set_led_pixel_callback(self, request, response):
        self.get_logger().info(str(request))
        self.pixels[request.index] = (request.r, request.g, request.b)
        self.pixels.show()
        response.success = True
        return response

    def set_led_ring_callback(self, request, response):
        self.get_logger().info(str(request))
        try:
            # Stop any running effect first
            self.stop_current_effect()
            
            color = eval(request.color)
            solid = Solid(self.pixels, color=color)
            solid.animate()
            response.success = True
            response.message = "Successfully set LED ring color"
        except:
            response.success = False
            response.message = "LED ring color not available"

        return response

    def blink_effect(self, color_name, delay=0.5):
        """Creates a blinking effect with specified color and delay"""
        color_map = {
            'red': red,
            'green': green,
            'blue': blue,
            'yellow': yellow,
            'purple': purple,
            'cyan': cyan,
            'white': white,
            'orange': orange,
            'teal': teal,
            'magenta': magenta,
            'gold': gold,
            'pink': pink,
            'aqua': aqua,
            'jade': jade,
            'amber': amber,
            'old_lace': old_lace,
            'black': black
        }
        
        if color_name not in color_map:
            self.get_logger().error(f"Unknown color: {color_name}")
            return
            
        color = color_map[color_name]
        
        while self.effect_running:
            if not self.effect_running:
                break
                
            # Turn all LEDs to specified color
            self.pixels.fill(color)
            self.pixels.show()
            time.sleep(delay)
            
            if not self.effect_running:
                break
                
            # Turn all LEDs off
            self.pixels.fill(black)
            self.pixels.show()
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

            if request.effect_name.lower().startswith('blink_'):
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

    def cleanup(self):
        """Clean up resources"""
        if self._is_shutting_down:
            return
            
        self._is_shutting_down = True
        print('Cleaning up LED resources...')
        
        # Stop any running effect
        self.stop_current_effect()
        
        # Turn off LEDs
        try:
            self.pixels.fill(black)
            self.pixels.show()
        except Exception as e:
            print(f'Error during LED cleanup: {str(e)}')

    def destroy_node(self):
        """Override destroy_node to ensure cleanup"""
        self.cleanup()
        super().destroy_node()

    def rainbow(self):
        return

    def fill (self):
        return


def main(args=None):
    try:
        rclpy.init(args=args)
        led_service = LEDService()
        
        try:
            rclpy.spin(led_service)
        except KeyboardInterrupt:
            print('Keyboard interrupt received, shutting down...')
            led_service.cleanup()
        finally:
            led_service.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except Exception as e:
        print(f'Error initializing node: {str(e)}')


if __name__ == '__main__':
    main()