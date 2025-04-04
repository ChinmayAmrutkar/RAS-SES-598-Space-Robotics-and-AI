#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Point
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import PointCloud2
import tf2_ros
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from drone_mission.msg import DroneCommand

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State tracking
        self.processed_markers = []
        self.rocks = []  # List of (id, x, y, height, diameter)
        self.current_lawnmower_pos = None
        self.last_command = ""
        self.current_cloud = None
        self.drone_position = [0.0, 0.0, 0.0]  # [x, y, z]
        self.lawnmower_complete = False
        self.search_area = {
            'min_x': -50.0, 'max_x': 50.0,
            'min_y': -50.0, 'max_y': 50.0,
            'spacing': 10.0
        }
        self.lawnmower_waypoints = self._generate_lawnmower_waypoints()
        self.current_waypoint_index = 0
        
        # Publishers
        self.cmd_pub = self.create_publisher(DroneCommand, '/drone_command', 10)
        
        # Subscribers
        self.create_subscription(String, '/aruco/marker_pose', self.aruco_callback, 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, 10)
        self.create_subscription(PointCloud2, '/cloud_map', self.cloud_callback, 10)
        
        # Status checking timer
        self.create_timer(1.0, self.mission_status_check)
        
        # Start lawnmower pattern
        self.send_command('LAWNMOWER', 0.0, 0.0, -20.0)
        self.get_logger().info("Mission Planner initialized, starting lawnmower pattern")

    def _generate_lawnmower_waypoints(self):
        """Generate waypoints for a lawnmower search pattern"""
        waypoints = []
        y_values = np.arange(
            self.search_area['min_y'], 
            self.search_area['max_y'] + self.search_area['spacing'], 
            self.search_area['spacing']
        )
        
        for i, y in enumerate(y_values):
            if i % 2 == 0:  # Even rows go left to right
                x_values = np.arange(
                    self.search_area['min_x'], 
                    self.search_area['max_x'] + self.search_area['spacing'], 
                    self.search_area['spacing']
                )
            else:  # Odd rows go right to left
                x_values = np.arange(
                    self.search_area['max_x'], 
                    self.search_area['min_x'] - self.search_area['spacing'], 
                    -self.search_area['spacing']
                )
                
            for x in x_values:
                waypoints.append((x, y, -20.0))  # 20m altitude for search
                
        return waypoints

    def send_command(self, mode, x, y, z, param1=0.0, param2=0.0):
        """Send command to the drone"""
        msg = DroneCommand()
        msg.mode = mode
        msg.target_x, msg.target_y, msg.target_z = x, y, z
        msg.param1, msg.param2 = param1, param2
        self.cmd_pub.publish(msg)
        self.last_command = mode
        self.get_logger().info(f"Sent command: {mode} to ({x}, {y}, {z})")

    def aruco_callback(self, msg):
        """Process ArUco marker detections"""
        parts = msg.data.split()
        if len(parts) < 2:
            return
            
        marker_id = int(parts[1])
        if marker_id not in self.processed_markers:
            try:
                transform = self.tf_buffer.lookup_transform('map', f'aruco_marker_{marker_id}', rclpy.time.Time())
                x, y = transform.transform.translation.x, transform.transform.translation.y
                
                self.get_logger().info(f"New marker {marker_id} detected at ({x}, {y})")
                self.processed_markers.append(marker_id)
                self.current_lawnmower_pos = self.get_drone_position()
                
                # Approach marker
                self.send_command('MOVE_TO', x, y, -20.0)
                
                # Schedule spiral descent after reaching the marker (5 seconds delay)
                self.create_timer(5.0, lambda: self.start_spiral_descent(x, y), one_shot=True)
                
            except Exception as e:
                self.get_logger().error(f'TF lookup failed: {e}')

    def start_spiral_descent(self, x, y):
        """Start spiral descent over a marker"""
        self.get_logger().info(f"Starting spiral descent at ({x}, {y})")
        self.send_command('SPIRAL_DESCENT', x, y, -1.0, 5.0, 1.0)  # param1=radius, param2=descent_rate

    def odom_callback(self, msg):
        """Process drone odometry"""
        # Update drone position (convert from NED to ENU)
        self.drone_position = [msg.position[0], msg.position[1], -msg.position[2]]
        
        # Check if we've reached the ground during spiral descent
        if self.drone_position[2] <= 1.0 and self.last_command == 'SPIRAL_DESCENT':
            self.get_logger().info("Reached minimum altitude during spiral, processing rock data")
            self.process_rock()
            
            # Return to lawnmower pattern
            x, y, z = self.current_lawnmower_pos
            self.send_command('MOVE_TO', x, y, -20.0)
            
            # Resume lawnmower after reaching previous position
            self.create_timer(5.0, self.resume_lawnmower, one_shot=True)

    def cloud_callback(self, msg):
        """Store point cloud data for processing"""
        self.current_cloud = msg

    def process_rock(self):
        """Process point cloud to estimate rock dimensions"""
        if not self.current_cloud:
            self.get_logger().warning("No point cloud data available for rock processing")
            return
            
        # Get the latest marker ID
        rock_id = self.processed_markers[-1]
        
        try:
            # Get marker position
            transform = self.tf_buffer.lookup_transform('map', f'aruco_marker_{rock_id}', rclpy.time.Time())
            rock_x = transform.transform.translation.x
            rock_y = transform.transform.translation.y
            
            # Process point cloud to calculate height and diameter
            # This is a simplified algorithm - in practice, you'd use clustering and statistical analysis
            points = list(pc2.read_points(self.current_cloud, field_names=("x", "y", "z"), skip_nans=True))
            
            # Filter points near the marker
            marker_points = []
            for point in points:
                dx = point[0] - rock_x
                dy = point[1] - rock_y
                if dx*dx + dy*dy < 25.0:  # Within 5m radius
                    marker_points.append(point)
            
            if marker_points:
                # Calculate height (highest z-value)
                heights = [p[2] for p in marker_points]
                height = max(heights) if heights else 0.5  # Default if no points
                
                # Calculate diameter (max distance between any two points)
                diameter = 0.0
                for i, p1 in enumerate(marker_points):
                    for p2 in marker_points[i+1:]:
                        dist = np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
                        diameter = max(diameter, dist)
                
                diameter = max(diameter, 0.5)  # Minimum diameter
            else:
                # Fallback if no points found
                height = 0.5
                diameter = 0.5
            
            self.rocks.append((rock_id, rock_x, rock_y, height, diameter))
            self.get_logger().info(f"Processed rock {rock_id}: height={height:.2f}m, diameter={diameter:.2f}m")
            
        except Exception as e:
            self.get_logger().error(f"Error processing rock: {e}")
            # Fallback values
            height = 0.5
            diameter = 0.5
            rock_x, rock_y = self.drone_position[0], self.drone_position[1]
            self.rocks.append((rock_id, rock_x, rock_y, height, diameter))

    def resume_lawnmower(self):
        """Resume lawnmower pattern after processing a rock"""
        if self.is_lawnmower_complete():
            self.get_logger().info("Lawnmower pattern complete, moving to final landing phase")
            self.land_on_tallest()
        else:
            self.get_logger().info("Resuming lawnmower pattern")
            self.send_command('LAWNMOWER', 0.0, 0.0, -20.0)
            
            # Move to next waypoint in pattern
            x, y, z = self.lawnmower_waypoints[self.current_waypoint_index]
            self.send_command('MOVE_TO', x, y, z)

    def land_on_tallest(self):
        """Find and land on the tallest rock"""
        if not self.rocks:
            self.get_logger().error("No rocks found! Emergency landing at current position")
            self.send_command('LAND', self.drone_position[0], self.drone_position[1], 0.0)
            return
            
        # Find the tallest rock
        tallest = max(self.rocks, key=lambda r: r[3])
        rock_id, x, y, height, diameter = tallest
        
        self.get_logger().info(f"Landing on tallest rock (ID {rock_id}, height {height:.2f}m) at ({x}, {y})")
        self.send_command('MOVE_TO', x, y, -20.0)  # Move above rock
        
        # Create timer to initiate landing after reaching position
        self.create_timer(5.0, lambda: self.send_command('LAND', x, y, 0.0), one_shot=True)

    def get_drone_position(self):
        """Get current drone position"""
        return self.drone_position

    def is_lawnmower_complete(self):
        """Check if lawnmower pattern is complete"""
        if self.lawnmower_complete:
            return True
            
        # Consider pattern complete if:
        # 1. We've reached the last waypoint, or
        # 2. We've found a minimum number of rocks (e.g., 3)
        if self.current_waypoint_index >= len(self.lawnmower_waypoints) - 1 or len(self.rocks) >= 3:
            self.lawnmower_complete = True
            return True
        return False

    def mission_status_check(self):
        """Regular status check to ensure mission progresses"""
        # Update waypoint index based on position (find closest remaining waypoint)
        if self.last_command == 'LAWNMOWER':
            min_dist = float('inf')
            for i, (x, y, z) in enumerate(self.lawnmower_waypoints[self.current_waypoint_index:]):
                dist = (x - self.drone_position[0])**2 + (y - self.drone_position[1])**2
                if dist < min_dist:
                    min_dist = dist
                    self.current_waypoint_index = self.current_waypoint_index + i
            
            # If we're close to current waypoint, move to next
            if min_dist < 4.0:  # Within 2m
                self.current_waypoint_index = min(self.current_waypoint_index + 1, len(self.lawnmower_waypoints) - 1)
                
                if self.current_waypoint_index < len(self.lawnmower_waypoints):
                    x, y, z = self.lawnmower_waypoints[self.current_waypoint_index]
                    self.send_command('MOVE_TO', x, y, z)

def main():
    rclpy.init()
    node = MissionPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()