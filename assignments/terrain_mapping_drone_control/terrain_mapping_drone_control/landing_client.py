#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import sys
import threading

class LandingClient(Node):
    """
    Simple client to trigger the ArUco marker landing sequence
    """
    def __init__(self):
        super().__init__('landing_client')
        
        # Publisher for landing trigger
        self.landing_trigger_pub = self.create_publisher(
            Bool,
            '/drone/trigger_landing',
            10
        )
        
        # Subscribe to landing status
        self.create_subscription(
            String,
            '/drone/landing_status',
            self.landing_status_callback,
            10
        )
        
        self.get_logger().info('Landing client initialized. Use the following commands:')
        self.get_logger().info('  land - Start the landing sequence')
        self.get_logger().info('  abort - Abort the landing sequence')
        self.get_logger().info('  exit - Exit this client')
    
    def landing_status_callback(self, msg):
        """Handle landing status messages"""
        self.get_logger().info(f'Landing Status: {msg.data}')
    
    def start_landing(self):
        """Send command to start landing sequence"""
        self.get_logger().info('Sending command to start landing sequence')
        msg = Bool()
        msg.data = True
        self.landing_trigger_pub.publish(msg)
    
    def abort_landing(self):
        """Send command to abort landing sequence"""
        self.get_logger().info('Sending command to abort landing sequence')
        msg = Bool()
        msg.data = False
        self.landing_trigger_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    client = LandingClient()
    
    # Create a thread to process messages
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,))
    spin_thread.start()
    
    try:
        while True:
            cmd = input('Enter command (land/abort/exit): ').strip().lower()
            
            if cmd == 'land':
                client.start_landing()
            elif cmd == 'abort':
                client.abort_landing()
            elif cmd == 'exit':
                break
            else:
                print('Unknown command. Please use land, abort, or exit.')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
