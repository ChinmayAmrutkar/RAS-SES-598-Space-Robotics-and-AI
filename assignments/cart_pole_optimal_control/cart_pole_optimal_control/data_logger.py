#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import csv
from datetime import datetime
import math

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Create a CSV file with a timestamped filename
        self.filename = f"cart_pole_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.get_logger().info(f"Logging data to {self.filename}")

        # Open CSV file and write header
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'Cart Position', 'Cart Velocity', 'Pole Angle', 'Pole Angular Velocity', 'Control Force'])

        # Subscribe to joint states and control force topics
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        self.force_sub = self.create_subscription(
            Float64,
            '/model/cart_pole/joint/cart_to_base/cmd_force',
            self.control_force_callback,
            10
        )

        self.latest_force = 0.0  # Store latest control force
        
        # Store logged data for post-processing:
        # Each row: [time, cart_pos, cart_vel, pole_angle, pole_ang_vel, control_force]
        self.data_rows = []

        # Flag to ensure shutdown is called only once
        self.logging_done = False

        # Create a one-shot timer for a maximum of 3 minutes (180 seconds)
        self.end_timer = self.create_timer(180.0, self.end_logging_callback)

    def joint_state_callback(self, msg):
        # Ensure valid data is available
        if len(msg.position) < 2 or len(msg.velocity) < 2:
            return

        cart_position = msg.position[0]
        cart_velocity = msg.velocity[0]
        pole_angle = msg.position[1]
        pole_angular_velocity = msg.velocity[1]
        current_time = self.get_clock().now().nanoseconds / 1e9  # seconds

        # Prepare and log the data row
        row = [current_time, cart_position, cart_velocity, pole_angle, pole_angular_velocity, self.latest_force]
        self.csv_writer.writerow(row)
        self.data_rows.append(row)
        self.get_logger().info(f"Logged: t={current_time:.2f}, x={cart_position:.2f}, θ={pole_angle:.2f}, u={self.latest_force:.2f}")

        # If cart position exceeds ±2.5 m, end logging immediately.
        if not self.logging_done and abs(cart_position) >= 2.5:
            self.get_logger().info("Cart reached ±2.5 m limit. Ending data logging...")
            self.end_logging_callback()

    def control_force_callback(self, msg):
        self.latest_force = msg.data  # Update the latest control force

    def end_logging_callback(self):
        if self.logging_done:
            return
        self.logging_done = True
        self.get_logger().info("Ending data logging...")
        rclpy.shutdown()  # Stop the spin

    def close_file(self):
        self.csv_file.close()

    def compute_metrics(self):
        """Compute and return various performance metrics."""
        if not self.data_rows:
            return None

        times = [row[0] for row in self.data_rows]
        cart_positions = [row[1] for row in self.data_rows]
        pole_angles = [row[3] for row in self.data_rows]
        control_forces = [row[5] for row in self.data_rows]

        N = len(self.data_rows)
        T_total = times[-1] - times[0] if N > 1 else 0.0

        # 1. Maximum Pole Angle Deviation (max absolute value)
        max_pole_deviation = max(abs(angle) for angle in pole_angles)

        # 2. RMS Cart Position Error (desired cart position is 0)
        rms_cart_error = math.sqrt(sum(x**2 for x in cart_positions) / N)

        # 3. Peak Control Force Used (max absolute control force)
        peak_control_force = max(abs(u) for u in control_forces)

        # 4. Recovery Time After Disturbances (using a threshold for pole angle)
        threshold = 0.1  # rad; adjust if needed
        recovery_times = []
        disturbed = False
        disturbance_start = None
        for row in self.data_rows:
            t, _, _, angle, _, _ = row
            if not disturbed and abs(angle) > threshold:
                disturbed = True
                disturbance_start = t
            elif disturbed and abs(angle) <= threshold:
                recovery_times.append(t - disturbance_start)
                disturbed = False
                disturbance_start = None
        max_recovery_time = max(recovery_times) if recovery_times else 0.0

        # 5. Maximum Cart Displacement (max absolute cart position)
        max_cart_displacement = max(abs(x) for x in cart_positions)

        # 6. RMS Pole Angle Deviation
        rms_pole_angle = math.sqrt(sum(angle**2 for angle in pole_angles) / N)

        # 7. Duration of Stable Operation (longest continuous period with "stable" states)
        stable_cart_threshold = 0.5   # m
        stable_angle_threshold = 0.05  # rad
        longest_stable_duration = 0.0
        current_stable_start = None
        for row in self.data_rows:
            t, x, _, angle, _, _ = row
            if abs(x) < stable_cart_threshold and abs(angle) < stable_angle_threshold:
                if current_stable_start is None:
                    current_stable_start = t
            else:
                if current_stable_start is not None:
                    stable_duration = t - current_stable_start
                    if stable_duration > longest_stable_duration:
                        longest_stable_duration = stable_duration
                    current_stable_start = None
        if current_stable_start is not None:
            stable_duration = times[-1] - current_stable_start
            if stable_duration > longest_stable_duration:
                longest_stable_duration = stable_duration

        # 8. Control Effort Analysis: Total and Average Control Effort
        total_control_effort = 0.0
        for i in range(N - 1):
            dt = times[i+1] - times[i]
            total_control_effort += abs(control_forces[i]) * dt
        average_control_effort = total_control_effort / T_total if T_total > 0 else 0.0

        return {
            'max_pole_deviation': max_pole_deviation,
            'rms_cart_error': rms_cart_error,
            'peak_control_force': peak_control_force,
            'max_recovery_time': max_recovery_time,
            'max_cart_displacement': max_cart_displacement,
            'rms_pole_angle': rms_pole_angle,
            'longest_stable_duration': longest_stable_duration,
            'total_control_effort': total_control_effort,
            'average_control_effort': average_control_effort
        }

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    finally:
        # Compute and output final metrics
        metrics = node.compute_metrics()
        if metrics:
            node.get_logger().info("Final Metrics:")
            node.get_logger().info(f"Maximum Pole Angle Deviation: {metrics['max_pole_deviation']:.4f} rad")
            node.get_logger().info(f"RMS Cart Position Error: {metrics['rms_cart_error']:.4f} m")
            node.get_logger().info(f"Peak Control Force Used: {metrics['peak_control_force']:.4f} N")
            node.get_logger().info(f"Max Recovery Time After Disturbances: {metrics['max_recovery_time']:.4f} s")
            node.get_logger().info(f"Maximum Cart Displacement: {metrics['max_cart_displacement']:.4f} m")
            node.get_logger().info(f"RMS Pole Angle Deviation: {metrics['rms_pole_angle']:.4f} rad")
            node.get_logger().info(f"Longest Stable Duration: {metrics['longest_stable_duration']:.4f} s")
            node.get_logger().info(f"Total Control Effort: {metrics['total_control_effort']:.4f}")
            node.get_logger().info(f"Average Control Effort: {metrics['average_control_effort']:.4f}")
        else:
            node.get_logger().info("No data was recorded.")
