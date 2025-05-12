#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_msgs.msg import String, Empty, Bool
from tf2_ros import TransformListener, Buffer
from tf2_ros.exceptions import TransformException
import numpy as np
import math
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ArucoLandingController(Node):
    def __init__(self):
        super().__init__('aruco_landing_controller')
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Landing parameters
        self.target_marker_id = 0  # Target ArUco marker ID (default 0)
        self.target_altitude = 1.0  # Meters to hover above marker before landing
        self.landing_speed = 0.3   # Meters per second for landing
        self.position_tolerance = 0.05  # XY position tolerance in meters
        self.altitude_tolerance = 0.05  # Z position tolerance in meters
        self.descent_step = 0.05   # Meters to descend each step
        self.is_landing = False
        self.marker_sighted = False
        self.last_marker_sighting = None
        self.marker_lost_timeout = 3.0  # Seconds to wait before considering marker lost
        
        # Create velocity command publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )
        
        # Create landing status publisher
        self.landing_pub = self.create_publisher(
            String,
            '/drone/landing_status',
            10
        )
        
        # Create takeoff publisher (Empty message to trigger takeoff)
        self.takeoff_pub = self.create_publisher(
            Empty,
            '/drone/takeoff',
            10
        )
        
        # Create landing publisher (Empty message to trigger internal landing procedure)
        self.land_pub = self.create_publisher(
            Empty,
            '/drone/land',
            10
        )
        
        # Subscribe to marker pose messages
        self.create_subscription(
            String,
            '/aruco/marker_pose',
            self.marker_pose_callback,
            10
        )
        
        # Subscribe to landing commands
        self.create_subscription(
            Bool,
            '/drone/trigger_landing',
            self.trigger_landing_callback,
            10
        )
        
        # Timer for position control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('ArUco Landing Controller initialized')
    
    def marker_pose_callback(self, msg):
        """Handle marker pose data from ArUco tracker"""
        if not self.is_landing:
            return
            
        # Update marker sighting timestamp
        self.marker_sighted = True
        self.last_marker_sighting = self.get_clock().now()

    def trigger_landing_callback(self, msg):
        """Handle trigger landing command"""
        if msg.data:
            self.start_landing_sequence()
        else:
            self.abort_landing()
    
    def start_landing_sequence(self):
        """Start the landing sequence"""
        self.get_logger().info(f'Starting landing sequence targeting ArUco marker ID: {self.target_marker_id}')
        self.publish_status("LANDING_SEQUENCE_STARTED")
        self.is_landing = True
        
    def abort_landing(self):
        """Abort the landing sequence"""
        self.get_logger().info('Landing sequence aborted')
        self.publish_status("LANDING_ABORTED")
        self.is_landing = False
        # Send zero velocity command to stop the drone
        self.send_velocity_command(0, 0, 0, 0)
        
    def complete_landing(self):
        """Complete the landing sequence"""
        self.get_logger().info('Landing completed successfully')
        self.publish_status("LANDING_COMPLETED")
        self.is_landing = False
        
        # Send land command
        self.land_pub.publish(Empty())
    
    def check_marker_lost(self):
        """Check if marker has been lost for too long"""
        if not self.marker_sighted:
            return True
            
        if self.last_marker_sighting is None:
            return True
            
        elapsed = (self.get_clock().now() - self.last_marker_sighting).nanoseconds / 1e9
        if elapsed > self.marker_lost_timeout:
            self.get_logger().warn(f'Marker lost for {elapsed:.2f} seconds')
            return True
            
        return False
    
    def control_loop(self):
        """Main control loop for landing sequence"""
        if not self.is_landing:
            return
        
        try:
            # Check if marker is lost
            if self.check_marker_lost():
                self.get_logger().warn('Marker lost - hovering in place')
                self.send_velocity_command(0, 0, 0, 0)
                self.publish_status("MARKER_LOST")
                return
            
            # Get transform from camera to ArUco marker
            transform = self.tf_buffer.lookup_transform(
                'camera_frame',
                f'aruco_marker_{self.target_marker_id}',
                rclpy.time.Time())
            
            # Extract position information
            marker_x = transform.transform.translation.x
            marker_y = transform.transform.translation.y
            marker_z = transform.transform.translation.z
            
            # Debug marker position
            self.get_logger().debug(f'Marker position: x={marker_x:.2f}, y={marker_y:.2f}, z={marker_z:.2f}')
            
            # Calculate required velocities (simplified PID)
            # Negative values because camera frame and drone movement frame are often inverted
            vx = -1.0 * marker_y  # Camera y-axis is drone's x-axis (usually inverted)
            vy = -1.0 * marker_x  # Camera x-axis is drone's y-axis (usually inverted)
            
            # Scale velocities
            max_lateral_speed = 0.3  # m/s
            vx = self.clamp(vx, -max_lateral_speed, max_lateral_speed)
            vy = self.clamp(vy, -max_lateral_speed, max_lateral_speed)
            
            # Determine vertical velocity based on landing phase
            vz = 0.0
            
            # Check if we're in position for descent
            if abs(marker_x) < self.position_tolerance and abs(marker_y) < self.position_tolerance:
                # We're centered, so start descending
                self.get_logger().info('Centered over marker, descending')
                vz = -self.landing_speed
                self.publish_status(f"DESCENDING z={marker_z:.2f}")
                
                # If we're close enough to complete landing
                if marker_z < 0.3:  # Final approach threshold
                    self.get_logger().info('Close to landing target, completing landing')
                    self.complete_landing()
                    return
            else:
                # We're not centered, maintain altitude and center first
                vz = 0.0
                self.publish_status(f"CENTERING x={marker_x:.2f} y={marker_y:.2f}")
            
            # Send velocity command
            self.send_velocity_command(vx, vy, vz, 0.0)  # No yaw rotation
            
        except TransformException as e:
            self.get_logger().warn(f'Transform lookup failed: {str(e)}')
            # If transform lookup fails, hover in place
            self.send_velocity_command(0, 0, 0, 0)
    
    def send_velocity_command(self, vx, vy, vz, yaw_rate):
        """Send velocity command to the drone"""
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)
        cmd.angular.z = float(yaw_rate)
        self.cmd_vel_pub.publish(cmd)
    
    def publish_status(self, status_msg):
        """Publish landing status message"""
        msg = String()
        msg.data = status_msg
        self.landing_pub.publish(msg)
    
    def clamp(self, value, min_value, max_value):
        """Clamp a value between min and max"""
        return max(min(value, max_value), min_value)

def main():
    rclpy.init()
    node = ArucoLandingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
