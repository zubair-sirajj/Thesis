#!/usr/bin/env python3
"""
Simple autonomous exploration for TurtleBot3 + Cartographer.

Strategy: Move forward slowly, turn when obstacles detected.
Random turns prevent loops and ensure full coverage.
"""

from enum import Enum
import math
import random
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ExploreState(Enum):
    """Simple state machine."""
    MOVING_FORWARD = 1
    TURNING = 2
    FINISHED = 3


class SimpleExplorer(Node):
    """Simple exploration node - forward motion with obstacle avoidance."""

    def __init__(self):
        super().__init__('simple_explorer')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # State
        self.state = ExploreState.MOVING_FORWARD
        self.scan_data = None
        self.map_data = None
        self.current_pose = None
        self.current_yaw = 0.0

        # Timing
        self.start_time = time.time()
        self.max_exploration_time = 1800  # 30 minutes
        self.turn_start_time = None
        self.turn_duration = 0.0
        self.last_coverage_report = time.time()

        # Motion parameters - SLOW for good Cartographer mapping
        self.forward_speed = 0.06  # Slow forward speed to reduce scan-to-scan distortion
        self.turn_speed = 0.35  # Gentler turns for cleaner SLAM alignment
        
        # Obstacle detection - INCREASED for safety
        self.obstacle_distance = 0.65  # Stop if obstacle within 65cm
        self.warning_distance = 0.80   # Slow down at 80cm
        self.side_check_distance = 0.5  # Check side clearance
        
        # Turn behavior
        self.min_turn_angle = 60.0  # Minimum turn (degrees)
        self.max_turn_angle = 120.0  # Maximum turn (degrees)
        self.turn_direction = 1  # 1 = left, -1 = right

        # Exploration tracking
        self.coverage_target = 0.92  # Stop at 92% coverage
        self.min_unknown_cells = 200  # Very few unknown cells

        # Control loop at 5 Hz
        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info('Simple Explorer started!')
        self.get_logger().info('Strategy: Move forward slowly, turn at obstacles')

    def scan_callback(self, msg):
        """Store laser scan."""
        self.scan_data = msg

    def map_callback(self, msg):
        """Store map."""
        self.map_data = msg

    def odom_callback(self, msg):
        """Store odometry."""
        self.current_pose = msg.pose.pose
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """Convert quaternion to yaw."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_front_distance(self):
        """Get minimum distance in front (wider cone for better detection)."""
        if self.scan_data is None:
            return float('inf')

        ranges = np.array(self.scan_data.ranges)
        n = len(ranges)
        
        # Check wider front cone (±30 indices for better detection)
        cone_width = 30
        center = n // 2
        front_ranges = ranges[center - cone_width:center + cone_width]
        
        # Filter out invalid readings
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        front_ranges = front_ranges[front_ranges > 0.01]  # Ignore very close noise

        if len(front_ranges) == 0:
            return float('inf')

        return float(np.min(front_ranges))

    def get_side_distances(self):
        """Get average distances on left and right sides."""
        if self.scan_data is None:
            return float('inf'), float('inf')

        ranges = np.array(self.scan_data.ranges)
        n = len(ranges)

        # Left side (60-120 degrees from front)
        left_start = n // 2 + n // 6
        left_end = n // 2 + n // 3
        left_ranges = ranges[left_start:left_end]
        left_ranges = left_ranges[np.isfinite(left_ranges)]
        left_dist = np.mean(left_ranges) if len(left_ranges) > 0 else np.inf

        # Right side (240-300 degrees from front)
        right_start = n // 2 - n // 3
        right_end = n // 2 - n // 6
        right_ranges = ranges[right_start:right_end]
        right_ranges = right_ranges[np.isfinite(right_ranges)]
        right_dist = np.mean(right_ranges) if len(right_ranges) > 0 else np.inf

        return float(left_dist), float(right_dist)

    def get_coverage(self):
        """Calculate map coverage percentage."""
        if self.map_data is None:
            return 0.0, 0

        grid = np.array(self.map_data.data, dtype=np.int8)
        total = len(grid)
        unknown = int(np.sum(grid == -1))
        known = total - unknown

        coverage = known / total if total > 0 else 0.0
        return coverage, unknown

    def is_exploration_complete(self):
        """Check if exploration is done."""
        coverage, unknown = self.get_coverage()
        
        # Need BOTH high coverage AND few unknown cells
        if coverage >= self.coverage_target and unknown <= self.min_unknown_cells:
            self.get_logger().info(
                f'Exploration complete: {coverage:.1%} coverage, {unknown} unknown cells')
            return True

        return False

    def start_turn(self):
        """Initiate a random turn."""
        # Choose turn direction based on side clearance
        left_dist, right_dist = self.get_side_distances()
        
        if left_dist > right_dist:
            self.turn_direction = 1  # Turn left (more space)
        else:
            self.turn_direction = -1  # Turn right (more space)

        # Random turn angle for variety (prevents loops)
        turn_angle_deg = random.uniform(self.min_turn_angle, self.max_turn_angle)
        turn_angle_rad = math.radians(turn_angle_deg)
        
        # Calculate turn duration based on angle and speed
        self.turn_duration = abs(turn_angle_rad / self.turn_speed)
        self.turn_start_time = time.time()
        
        direction = "LEFT" if self.turn_direction > 0 else "RIGHT"
        self.get_logger().info(f'Turning {direction} ~{turn_angle_deg:.0f}°')

    def control_loop(self):
        """Main control loop."""
        # Report coverage periodically
        now = time.time()
        if now - self.last_coverage_report > 10.0:
            coverage, unknown = self.get_coverage()
            elapsed = now - self.start_time
            self.get_logger().info(
                f'Progress: {coverage:.1%} coverage, {unknown} unknown cells, '
                f'{elapsed:.0f}s elapsed')
            self.last_coverage_report = now
        
        # Check time limit
        if now - self.start_time > self.max_exploration_time:
            self.get_logger().info('Time limit reached!')
            self.state = ExploreState.FINISHED

        # Check completion
        if self.is_exploration_complete():
            self.state = ExploreState.FINISHED

        # Create velocity command
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # State machine logic
        if self.state == ExploreState.MOVING_FORWARD:
            front_dist = self.get_front_distance()
            
            if front_dist < self.obstacle_distance:
                # Obstacle too close - STOP and turn
                self.get_logger().info(f'Obstacle detected at {front_dist:.2f}m - stopping and turning')
                self.start_turn()
                self.state = ExploreState.TURNING
            elif front_dist < self.warning_distance:
                # Obstacle approaching - slow down
                cmd.twist.linear.x = self.forward_speed * 0.5
                self.get_logger().info(
                    f'Slowing down (obstacle at {front_dist:.2f}m)',
                    throttle_duration_sec=1.0)
            else:
                # Clear path - move forward at normal speed
                cmd.twist.linear.x = self.forward_speed
                self.get_logger().info(
                    f'Moving forward (clear: {front_dist:.2f}m)',
                    throttle_duration_sec=3.0)

        elif self.state == ExploreState.TURNING:
            # Safety check - if still too close to obstacle, keep turning
            front_dist = self.get_front_distance()
            if front_dist < self.obstacle_distance * 0.8:
                # Still too close - extend the turn
                self.turn_duration = time.time() - self.turn_start_time + 1.0
                self.get_logger().warn(
                    f'Still close to obstacle ({front_dist:.2f}m), extending turn')
            
            # Check if turn is complete
            if time.time() - self.turn_start_time >= self.turn_duration:
                # Verify path is clear before resuming
                if front_dist > self.obstacle_distance:
                    self.get_logger().info('Turn complete, path clear, resuming')
                    self.state = ExploreState.MOVING_FORWARD
                else:
                    self.get_logger().warn('Turn complete but still blocked, turning more')
                    self.start_turn()  # Turn again
            else:
                # Continue turning
                cmd.twist.angular.z = self.turn_direction * self.turn_speed
                self.get_logger().info('Turning...', throttle_duration_sec=0.5)

        elif self.state == ExploreState.FINISHED:
            # Stop the robot
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            
            coverage, unknown = self.get_coverage()
            self.get_logger().info(
                f'EXPLORATION COMPLETE! Coverage: {coverage:.1%}, Unknown: {unknown}')

        # Publish command
        self.cmd_vel_pub.publish(cmd)

    def stop(self):
        """Stop the robot."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    explorer = SimpleExplorer()

    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        explorer.get_logger().info('Stopping...')
    finally:
        explorer.stop()
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
