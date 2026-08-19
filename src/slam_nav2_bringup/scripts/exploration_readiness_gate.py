#!/usr/bin/env python3
"""Start frontier exploration only after the mapping/navigation stack is ready."""

from frontier_exploration_ros2.srv import ControlExploration
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ExplorationReadinessGate(Node):
    """Wait for map, costmaps, Nav2 and frontier control before starting motion."""

    def __init__(self):
        super().__init__('exploration_readiness_gate')
        self.declare_parameter('post_ready_delay', 2.0)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._ready = {'map': False, 'global': False, 'local': False}
        self._subscriptions = [
            self.create_subscription(
                OccupancyGrid, '/map',
                lambda _: self._mark_ready('map'), map_qos),
            self.create_subscription(
                OccupancyGrid, '/global_costmap/costmap',
                lambda _: self._mark_ready('global'), map_qos),
            self.create_subscription(
                OccupancyGrid, '/local_costmap/costmap',
                lambda _: self._mark_ready('local'), map_qos),
        ]
        self._navigate_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self._control_client = self.create_client(
            ControlExploration, '/control_exploration')
        self._request_sent = False
        self._ready_since = None
        self._last_waiting_log_ns = 0
        self._timer = self.create_timer(0.5, self._check_readiness)
        self.get_logger().info(
            'Holding exploration until map, costmaps, Nav2 and frontier service are ready')

    def _mark_ready(self, key):
        self._ready[key] = True

    def _check_readiness(self):
        if self._request_sent:
            return

        nav_ready = self._navigate_client.server_is_ready()
        service_ready = self._control_client.service_is_ready()
        all_ready = all(self._ready.values()) and nav_ready and service_ready

        now = self.get_clock().now()
        if not all_ready:
            self._ready_since = None
            if now.nanoseconds - self._last_waiting_log_ns > 5_000_000_000:
                self._last_waiting_log_ns = now.nanoseconds
                missing = [name for name, ready in self._ready.items() if not ready]
                if not nav_ready:
                    missing.append('navigate_to_pose')
                if not service_ready:
                    missing.append('control_exploration')
                self.get_logger().info('Waiting for: ' + ', '.join(missing))
            return

        if self._ready_since is None:
            self._ready_since = now
            self.get_logger().info('Stack is ready; applying final settling delay')
            return

        settle_delay = float(self.get_parameter('post_ready_delay').value)
        if (now - self._ready_since).nanoseconds < max(0.0, settle_delay) * 1e9:
            return

        request = ControlExploration.Request()
        request.action = ControlExploration.Request.ACTION_START
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        self._request_sent = True
        future = self._control_client.call_async(request)
        future.add_done_callback(self._start_done)

    def _start_done(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS future surfaces transport errors
            self.get_logger().error(f'Failed to start exploration: {exc}')
            self._request_sent = False
            return

        if response.accepted:
            self.get_logger().info('Frontier exploration started safely')
            self._timer.cancel()
        else:
            self.get_logger().error(
                f'Frontier explorer rejected start request: {response.message}')
            self._request_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationReadinessGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
