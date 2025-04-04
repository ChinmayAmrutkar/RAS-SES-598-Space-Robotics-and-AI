#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry
from drone_mission.msg import DroneCommand
import math
import numpy as np

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # State variables
        self.current_mode = 'IDLE'
        self.previous_mode = 'IDLE'
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        
        # Lawnmower pattern state
        self.lawnmower_r = 10.0  # Initial radius
        self.lawnmower_theta = 0.0
        self.lawnmower_spacing = 5.0  # Space between concentric circles
        self.lawnmower_speed = 0.02  # Angular speed adjustment
        
        # Spiral descent state
        self.spiral_t = 0.0
        self.spiral_radius = 5.0
        self.spiral_descent_rate = 1.0
        self.spiral_rotation_speed = 5.0
        
        # Current position tracking
        self.current_x, self.current_y, self.current_z = 0.0, 0.0, 0.0
        
        # Publishers
        self.control_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Subscribers
        self.create_subscription(
            DroneCommand, '/drone_command', self.command_callback, 10)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, 10)
        
        # Timer for control loop
        self.create_timer(0.02, self.control_loop)  # 50 Hz
        
        self.get_logger().info('Drone controller initialized')

    def command_callback(self, msg):
        """Handle incoming drone commands"""
        # Store previous mode for state transition handling
        self.previous_mode = self.current_mode
        self.current_mode = msg.mode
        
        # Update target position
        self.target_x, self.target_y, self.target_z = msg.target_x, msg.target_y, msg.target_z
        
        # Handle parameters
        if msg.param1 > 0.0:
            self.spiral_radius = msg.param1
        
        if msg.param2 > 0.0:
            self.spiral_descent_rate = msg.param2
            
        # Reset spiral timer on new spiral command
        if self.current_mode == 'SPIRAL_DESCENT' and self.previous_mode != 'SPIRAL_DESCENT':
            self.spiral_t = 0.0
            
        self.get_logger().info(f"New command: {self.current_mode} to ({self.target_x}, {self.target_y}, {self.target_z})")

    def odometry_callback(self, msg):
        """Track drone position for accurate movement"""
        # Convert NED to ENU
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = -msg.position[2]  # NED to ENU conversion

    def control_loop(self):
        """Main control loop for drone movement"""
        # Publish offboard control mode
        control_msg = OffboardControlMode()
        control_msg.position = True
        control_msg.velocity = False
        control_msg.acceleration = False
        control_msg.attitude = False
        control_msg.body_rate = False
        control_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.control_pub.publish(control_msg)
        
        # Create setpoint message
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = control_msg.timestamp
        
        # Generate trajectory based on current mode
        if self.current_mode == 'LAWNMOWER':
            self._generate_lawnmower_setpoint(setpoint)
        elif self.current_mode == 'MOVE_TO':
            self._generate_move_to_setpoint(setpoint)
        elif self.current_mode == 'SPIRAL_DESCENT':
            self._generate_spiral_descent_setpoint(setpoint)
        elif self.current_mode == 'LAND':
            self._generate_landing_setpoint(setpoint)
        else:  # IDLE or unknown mode
            # Hover at current position
            setpoint.position = [self.current_x, self.current_y, self.current_z]
            
        # Publish setpoint
        self.setpoint_pub.publish(setpoint)

    def _generate_lawnmower_setpoint(self, setpoint):
        """Generate concentric circular lawnmower pattern"""
        # Update angle for circular motion
        self.lawnmower_theta += self.lawnmower_speed
        
        # Check if we've completed a full circle
        if self.lawnmower_theta >= 2 * math.pi:
            self.lawnmower_theta = 0.0
            self.lawnmower_r += self.lawnmower_spacing  # Increase radius for next circle
            
        # Calculate position on the circle
        x = self.lawnmower_r * math.cos(self.lawnmower_theta)
        y = self.lawnmower_r * math.sin(self.lawnmower_theta)
        
        # Set position
        setpoint.position = [x, y, self.target_z]
        
        # Optionally set velocity for smoother motion
        tangent_speed = self.lawnmower_r * self.lawnmower_speed
        vx = -tangent_speed * math.sin(self.lawnmower_theta)
        vy = tangent_speed * math.cos(self.lawnmower_theta)
        setpoint.velocity = [vx, vy, 0.0]

    def _generate_move_to_setpoint(self, setpoint):
        """Direct movement to target position"""
        setpoint.position = [self.target_x, self.target_y, self.target_z]
        
        # Calculate direction vector
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dz = self.target_z - self.current_z
        
        # Calculate distance
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # If close to target, slow down
        if distance < 2.0:
            setpoint.velocity = [0.0, 0.0, 0.0]
        else:
            # Normalize and scale
            speed = min(3.0, distance * 0.5)  # Cap speed at 3 m/s
            setpoint.velocity = [
                dx / distance * speed,
                dy / distance * speed,
                dz / distance * speed
            ]

    def _generate_spiral_descent_setpoint(self, setpoint):
        """Spiral descent pattern for rock inspection"""
        # Update time
        self.spiral_t += 0.02
        
        # Calculate shrinking radius (linear decrease)
        radius = max(0.5, self.spiral_radius * (1 - self.spiral_t / 20.0))
        
        # Calculate descent (linear from starting altitude to target altitude)
        current_altitude = -20.0 + self.spiral_t * self.spiral_descent_rate
        z = max(self.target_z, current_altitude)
        
        # Calculate position on spiral
        angle = self.spiral_t * self.spiral_rotation_speed
        x = self.target_x + radius * math.cos(angle)
        y = self.target_y + radius * math.sin(angle)
        
        # Set position
        setpoint.position = [x, y, z]
        
        # Calculate velocity for smooth motion
        vx = -radius * self.spiral_rotation_speed * math.sin(angle) - (self.spiral_radius / 20.0) * math.cos(angle)
        vy = radius * self.spiral_rotation_speed * math.cos(angle) - (self.spiral_radius / 20.0) * math.sin(angle)
        vz = self.spiral_descent_rate
        
        # Set velocity
        setpoint.velocity = [vx, vy, vz]
        
        # Check if we've reached target altitude
        if z <= self.target_z:
            # Reset spiral state for future use
            self.spiral_t = 0.0

    def _generate_landing_setpoint(self, setpoint):
        """Controlled landing at target position"""
        # First move to position above landing point
        if abs(self.current_x - self.target_x) > 0.5 or abs(self.current_y - self.target_y) > 0.5:
            # Still need to move horizontally
            setpoint.position = [self.target_x, self.target_y, max(self.current_z, -2.0)]
        else:
            # Once positioned correctly, descend slowly
            descent_rate = 0.5  # m/s
            target_z = max(self.target_z, self.current_z - 0.02 * descent_rate)
            setpoint.position = [self.target_x, self.target_y, target_z]
            
            # Slow descent velocity
            setpoint.velocity = [0.0, 0.0, -descent_rate]

def main():
    rclpy.init()
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()