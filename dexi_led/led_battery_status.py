#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import BatteryStatus
from dexi_interfaces.srv import LEDRingColor
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time

class LEDBatteryStatus(Node):
    def __init__(self):
        super().__init__('led_battery_status')
        
        # Create a client for the LED ring color service
        self.led_client = self.create_client(LEDRingColor, 'led_service/set_led_ring_color')
        while not self.led_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LED service not available, waiting...')
        
        # Create a publisher for battery state
        self.battery_state_pub = self.create_publisher(
            String,
            'battery_state',
            10
        )
        
        # Set up QoS profile for PX4 messages
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to PX4 battery status
        self.subscription = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_status_callback,
            qos_profile
        )
        
        # Initialize variables
        self.last_flash_time = 0
        self.flash_state = False
        self.current_battery_level = 100.0
        self.flash_interval = 1.0  # seconds
        self.current_state = "NORMAL"  # NORMAL, LOW, CRITICAL
        
        # Create a timer for LED flashing
        self.timer = self.create_timer(0.5, self.flash_callback)
        
    def battery_status_callback(self, msg):
        # Update current battery level
        if msg.remaining >= 0:  # -1 means unknown
            self.current_battery_level = msg.remaining * 100.0  # Convert to percentage
            
            # Update battery state
            if self.current_battery_level < 15.0:
                new_state = "CRITICAL"
            elif self.current_battery_level < 30.0:
                new_state = "LOW"
            else:
                new_state = "NORMAL"
                
            # Only publish if state changed
            if new_state != self.current_state:
                self.current_state = new_state
                state_msg = String()
                state_msg.data = self.current_state
                self.battery_state_pub.publish(state_msg)
            
    def flash_callback(self):
        current_time = time.time()
        
        # Determine if we should be flashing based on battery level
        if self.current_battery_level < 15.0:
            # Critical battery - flash red
            color = "red" if self.flash_state else "black"
        elif self.current_battery_level < 30.0:
            # Low battery - flash yellow
            color = "yellow" if self.flash_state else "black"
        else:
            # Normal battery - no flashing needed
            return
            
        # Toggle flash state if enough time has passed
        if current_time - self.last_flash_time >= self.flash_interval:
            self.flash_state = not self.flash_state
            self.last_flash_time = current_time
            
            # Send LED color request
            request = LEDRingColor.Request()
            request.color = color
            
            future = self.led_client.call_async(request)
            future.add_done_callback(self.led_response_callback)
            
    def led_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f'Failed to set LED color: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LEDBatteryStatus()
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