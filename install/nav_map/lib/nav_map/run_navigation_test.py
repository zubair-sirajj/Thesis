#!/usr/bin/env python3
import math
import os
import time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def read_pgm_size(pgm_path):
    """Read width/height from a PGM file header (P2/P5)."""
    with open(pgm_path, 'rb') as f:
        magic = f.readline().strip()
        if magic not in (b'P2', b'P5'):
            raise ValueError(f'Unsupported PGM format in {pgm_path}: {magic}')

        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                raise ValueError(f'Unexpected EOF while reading {pgm_path}')
            stripped = line.strip()
            if not stripped or stripped.startswith(b'#'):
                continue
            tokens.extend(stripped.split())

        width = int(tokens[0])
        height = int(tokens[1])
        return width, height


def load_map_bounds(yaml_path):
    """Load map bounds (xmin, xmax, ymin, ymax) from nav2 map yaml + image size."""
    image = None
    resolution = None
    origin = None

    with open(yaml_path, 'r') as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue

            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()

            if key == 'image':
                image = value
            elif key == 'resolution':
                resolution = float(value)
            elif key == 'origin':
                origin_str = value.strip().lstrip('[').rstrip(']')
                origin = [float(v.strip()) for v in origin_str.split(',')]

    if image is None or resolution is None or origin is None or len(origin) < 2:
        raise ValueError(f'Invalid map yaml: {yaml_path}')

    image_path = os.path.join(os.path.dirname(yaml_path), image)
    width, height = read_pgm_size(image_path)

    xmin = origin[0]
    ymin = origin[1]
    xmax = xmin + width * resolution
    ymax = ymin + height * resolution
    return xmin, xmax, ymin, ymax


def point_in_bounds(x, y, bounds):
    xmin, xmax, ymin, ymax = bounds
    return xmin <= x <= xmax and ymin <= y <= ymax


class NavigationMetrics:
    """Tracks traversed path length and plan updates while a goal is active."""

    def __init__(self, node):
        self._node = node
        self._pose_source = None
        self._last_pos = None
        self.path_length = 0.0
        self.plan_updates = 0
        self.goal_active = False

        self._node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10,
        )
        self._node.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
        )
        self._node.create_subscription(Path, '/plan', self._plan_callback, 10)

    def start_goal(self):
        self.path_length = 0.0
        self.plan_updates = 0
        self.goal_active = True
        self._last_pos = None

    def stop_goal(self):
        self.goal_active = False

    def get_replan_count(self):
        # First plan is initial planning; remaining updates are replans.
        return max(0, self.plan_updates - 1)

    def _plan_callback(self, _msg):
        if self.goal_active:
            self.plan_updates += 1

    def _amcl_pose_callback(self, msg):
        self._handle_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, 'amcl')

    def _odom_callback(self, msg):
        self._handle_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, 'odom')

    def _handle_pose(self, x, y, source):
        if not self.goal_active:
            return

        if self._pose_source is None:
            self._pose_source = source

        if source != self._pose_source:
            return

        if self._last_pos is not None:
            dx = x - self._last_pos[0]
            dy = y - self._last_pos[1]
            self.path_length += math.hypot(dx, dy)

        self._last_pos = (x, y)


def main():
    rclpy.init()
    navigator = BasicNavigator()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_yaml_name = os.environ.get('NAV_MAP_YAML', 'my_map.yaml')
    map_yaml_path = os.path.join(script_dir, 'maps', map_yaml_name)
    try:
        map_bounds = load_map_bounds(map_yaml_path)
        xmin, xmax, ymin, ymax = map_bounds
        print(f'Map bounds: x in [{xmin:.3f}, {xmax:.3f}], y in [{ymin:.3f}, {ymax:.3f}]')
    except Exception as e:
        print(f'WARNING: Could not load map bounds from {map_yaml_path}: {e}')
        map_bounds = None

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    # Use zero stamp so TF resolves against the latest available transform.
    initial_pose.header.stamp.sec = 0
    initial_pose.header.stamp.nanosec = 0
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0

    # Publish initial pose a few times to make AMCL initialization more reliable.
    for _ in range(3):
        navigator.setInitialPose(initial_pose)
        time.sleep(0.2)

    # Wait for Nav2 to be fully active with sim time support (handling amcl)
    try:
        print('Waiting for Nav2 to transition to active...')
        navigator.waitUntilNav2Active()
    except Exception as e:
        print(f'ERROR: Failed to initialize Nav2: {e}')
        rclpy.shutdown()
        return

    metrics = NavigationMetrics(navigator)

    waypoints = [
        (10.0, -3.04, 0.0),
        (-1.70, -0.5, 0.0),
        (5.0, 7.0, 0.0),
    ]
    output_path = os.path.join(script_dir, 'navigation_goal_metrics.txt')

    try:
        with open(output_path, 'a') as f:
            time.sleep(2)  # Short delay before starting navigation

            for idx, (goal_x, goal_y, goal_theta) in enumerate(waypoints, start=1):
                if map_bounds is not None and not point_in_bounds(goal_x, goal_y, map_bounds):
                    res_str = (
                        f'Waypoint {idx} Goal ({goal_x}, {goal_y}): Status = FAILED, '
                        'Reason = Goal outside map bounds, Time taken = 0.00 seconds, '
                        'Path length = 0.00 meters, Replans = 0\n'
                    )
                    print(res_str)
                    f.write(res_str)
                    continue

                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = navigator.get_clock().now().to_msg()

                goal.pose.position.x = float(goal_x)
                goal.pose.position.y = float(goal_y)
                # Convert planar yaw to quaternion (roll=pitch=0).
                goal.pose.orientation.z = float(math.sin(goal_theta / 2.0))
                goal.pose.orientation.w = float(math.cos(goal_theta / 2.0))

                print(f'Sending robot to waypoint {idx} goal ({goal_x}, {goal_y})...')
                metrics.start_goal()
                start_real_time = time.monotonic()
                navigator.goToPose(goal)

                while not navigator.isTaskComplete():
                    # Spin to process subscriptions used for path length and plan updates.
                    rclpy.spin_once(navigator, timeout_sec=0.1)
                    time.sleep(0.1)

                metrics.stop_goal()

                result = navigator.getResult()
                feedback = navigator.getFeedback()
                total_time = time.monotonic() - start_real_time

                if result == TaskResult.SUCCEEDED:
                    status = 'SUCCEEDED'
                elif result == TaskResult.CANCELED:
                    status = 'CANCELED'
                elif result == TaskResult.FAILED:
                    status = 'FAILED'
                else:
                    status = 'UNKNOWN'

                reason = ''
                if status == 'FAILED' and feedback is not None:
                    reason = f', Feedback = {feedback}'

                replans = metrics.get_replan_count()
                res_str = (
                    f'Waypoint {idx} Goal ({goal_x}, {goal_y}): Status = {status}{reason}, '
                    f'Time taken = {total_time:.2f} seconds, '
                    f'Path length = {metrics.path_length:.2f} meters, '
                    f'Replans = {replans}\n'
                )
                print(res_str)
                f.write(res_str)

                # Short pause between goals to let local planner settle.
                time.sleep(0.5)

    except Exception as e:
        print(f'ERROR during navigation test: {e}')
    finally:
        try:
            navigator.cancelTask()
        except Exception:
            pass
        rclpy.shutdown()

    print(f'Results saved to {output_path}')


if __name__ == '__main__':
    main()
