#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus
from dexi_interfaces.srv import LEDRingColor, LEDEffect
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from .utils.flight_mode import FlightMode

# Helper function to get the message version suffix for backwards compatibility
def get_message_name_version(msg_class):
    if hasattr(msg_class, 'MESSAGE_VERSION'):
        if msg_class.MESSAGE_VERSION == 0:
            return ""
        return f"_v{msg_class.MESSAGE_VERSION}"
    return ""

class LEDFlightModeStatus(Node):
    def __init__(self):
        super().__init__('led_flight_mode_status')
        
        # Create a client for the LED ring color service
        self.led_client = self.create_client(LEDRingColor, '/dexi/led_service/set_led_ring_color')
        while not self.led_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LED service not available, waiting...')
        
        # Create a client for the LED effect service
        self.effect_client = self.create_client(LEDEffect, '/dexi/led_service/set_led_effect')
        while not self.effect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LED effect service not available, waiting...')
        
        # Subscribe to battery state
        self.battery_state_sub = self.create_subscription(
            String,
            'battery_state',
            self.battery_state_callback,
            10
        )
        
        # Set up QoS profile for PX4 messages
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to PX4 vehicle status with version-aware topic name
        vehicle_status_topic = f"/fmu/out/vehicle_status{get_message_name_version(VehicleStatus)}"
        
        self.subscription = self.create_subscription(
            VehicleStatus,
            vehicle_status_topic,
            self.vehicle_status_callback,
            qos_profile
        )
        
        self.previous_nav_state = None
        self.battery_state = "NORMAL"  # NORMAL, LOW, CRITICAL
        
    def battery_state_callback(self, msg):
        self.battery_state = msg.data
        
    def vehicle_status_callback(self, msg):
        # Only update if the nav state has changed
        if msg.nav_state == self.previous_nav_state:
            return
            
        self.previous_nav_state = msg.nav_state
        
        # Don't change LED color if battery is in warning state
        if self.battery_state != "NORMAL":
            return
        
        # Map PX4 flight modes to LED color names
        color_map = {
            FlightMode.STABILIZED: "white",
            FlightMode.ALTITUDE: "yellow",
            FlightMode.POSITION: "green",
            FlightMode.OFFBOARD: "purple",
            FlightMode.TAKEOFF: "blue",
            FlightMode.LAND: "pink",
            FlightMode.HOLD: "cyan"
        }
        
        try:
            flight_mode = FlightMode(msg.nav_state)
            # Get color for current flight mode, default to black if unknown
            color = color_map.get(flight_mode, "black")
            
            # Create and send request to LED service
            request = LEDRingColor.Request()
            request.color = color
            
            future = self.led_client.call_async(request)
            future.add_done_callback(self.led_response_callback)
            
        except ValueError:
            self.get_logger().warn(f'Unknown flight mode: {msg.nav_state}')
            # Use cyan blinking effect for unknown flight modes
            effect_request = LEDEffect.Request()
            effect_request.effect_name = "blink_cyan"
            
            future = self.effect_client.call_async(effect_request)
            future.add_done_callback(self.effect_response_callback)
        
    def led_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f'Failed to set LED color: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def effect_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f'Failed to set LED effect: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Effect service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LEDFlightModeStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 