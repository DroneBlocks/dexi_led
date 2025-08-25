#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus
from dexi_interfaces.srv import LEDRingColor
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from .utils.flight_mode import FlightMode

class LEDFlightModeStatus(Node):
    def __init__(self):
        super().__init__('led_flight_mode_status')
        
        # Create a client for the LED ring color service
        self.led_client = self.create_client(LEDRingColor, '/dexi/led_service/set_led_ring_color')
        while not self.led_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LED service not available, waiting...')
        
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
        
        # Subscribe to PX4 vehicle status
        self.subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
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
            FlightMode.LAND: "pink"
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
        
    def led_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f'Failed to set LED color: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LEDFlightModeStatus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 