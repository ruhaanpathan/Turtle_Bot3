#!/usr/bin/env python3
"""Persist the SLAM map once frontier exploration reports completion."""

from datetime import datetime, timezone
import math
from pathlib import Path
import re
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import Empty, String


_PREFIX_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
_OUTPUT_SUFFIXES = ('.yaml', '.pgm', '.png', '.posegraph', '.data')


class AutoMapSaver(Node):
    """Save a uniquely named map after one exploration-complete event."""

    def __init__(self, **kwargs):
        super().__init__('auto_map_saver', **kwargs)

        self.declare_parameter('map_directory', '/home/ros/maps')
        self.declare_parameter('map_prefix', 'turtlebot_map')
        self.declare_parameter('completion_topic', '/exploration_complete')
        self.declare_parameter('save_map_service', '/slam_toolbox/save_map')
        self.declare_parameter(
            'serialize_pose_graph_service', '/slam_toolbox/serialize_map')
        self.declare_parameter('serialize_pose_graph', True)
        self.declare_parameter('service_wait_timeout', 60.0)
        self.declare_parameter('service_call_timeout', 120.0)

        self._map_prefix = self._validate_prefix(
            str(self.get_parameter('map_prefix').value))
        self._map_directory = self._prepare_directory(
            str(self.get_parameter('map_directory').value))
        self._serialize_enabled = bool(
            self.get_parameter('serialize_pose_graph').value)
        self._service_wait_timeout = float(
            self.get_parameter('service_wait_timeout').value)
        self._service_call_timeout = float(
            self.get_parameter('service_call_timeout').value)
        if (not math.isfinite(self._service_wait_timeout) or
                self._service_wait_timeout <= 0.0):
            raise ValueError('service_wait_timeout must be greater than zero')
        if (not math.isfinite(self._service_call_timeout) or
                self._service_call_timeout <= 0.0):
            raise ValueError('service_call_timeout must be greater than zero')

        completion_topic = str(self.get_parameter('completion_topic').value)
        self._save_service = str(self.get_parameter('save_map_service').value)
        self._serialize_service = str(
            self.get_parameter('serialize_pose_graph_service').value)

        self._save_client = self.create_client(SaveMap, self._save_service)
        self._serialize_client = self.create_client(
            SerializePoseGraph, self._serialize_service)

        completion_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._completion_subscription = self.create_subscription(
            Empty, completion_topic, self._on_exploration_complete,
            completion_qos)

        self._state = 'waiting_for_completion'
        self._wait_started = None
        self._call_started = None
        self._active_future = None
        self._output_basename = None
        self._timer = self.create_timer(0.5, self._poll_service)

        self.get_logger().info(
            f"Will save one completed map under '{self._map_directory}'")

    @staticmethod
    def _prepare_directory(raw_path):
        if not raw_path or '\x00' in raw_path:
            raise ValueError('map_directory must be a non-empty path')

        requested = Path(raw_path).expanduser()
        if not requested.is_absolute():
            raise ValueError('map_directory must be an absolute path')

        directory = requested.resolve(strict=False)
        if directory == Path(directory.anchor):
            raise ValueError('map_directory cannot be the filesystem root')

        directory.mkdir(parents=True, exist_ok=True)
        if not directory.is_dir():
            raise ValueError(f"map_directory is not a directory: '{directory}'")

        try:
            with tempfile.NamedTemporaryFile(
                    prefix='.auto_map_saver_', dir=directory):
                pass
        except OSError as exc:
            raise ValueError(
                f"map_directory is not writable: '{directory}': {exc}") from exc

        return directory

    @staticmethod
    def _validate_prefix(prefix):
        if not _PREFIX_PATTERN.fullmatch(prefix):
            raise ValueError(
                'map_prefix must contain 1-64 letters, digits, underscores, '
                'or hyphens, and must start with a letter or digit')
        return prefix

    @property
    def done(self):
        """Return true after the one requested save sequence has ended."""
        return self._state in ('complete', 'failed')

    @property
    def save_succeeded(self):
        """Return true when the occupancy map service reported success."""
        return self._state == 'complete'

    @property
    def output_basename(self):
        """Return the output basename selected for the current run."""
        return self._output_basename

    def _make_unique_basename(self):
        stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%fZ')
        stem = f'{self._map_prefix}_{stamp}'
        candidate = self._map_directory / stem
        suffix = 1
        while any(Path(f'{candidate}{extension}').exists()
                  for extension in _OUTPUT_SUFFIXES):
            candidate = self._map_directory / f'{stem}_{suffix}'
            suffix += 1

        candidate = candidate.resolve(strict=False)
        if candidate.parent != self._map_directory:
            raise RuntimeError('Generated map basename escaped map_directory')
        return candidate

    def _on_exploration_complete(self, _message):
        if self._state != 'waiting_for_completion':
            self.get_logger().info(
                'Ignoring duplicate exploration-complete event; map save is '
                'already active or finished')
            return

        self._output_basename = self._make_unique_basename()
        self._wait_started = time.monotonic()
        self._state = 'waiting_for_save_service'
        self.get_logger().info(
            'Exploration complete; preparing to save map as '
            f"'{self._output_basename}'")
        if not self._save_client.service_is_ready():
            self.get_logger().info(f"Waiting for '{self._save_service}'")
        self._poll_service()

    def _poll_service(self):
        if self._state == 'waiting_for_save_service':
            if self._save_client.service_is_ready():
                self._request_map_save()
            elif self._service_wait_expired():
                self.get_logger().error(
                    'Map was not saved: SLAM Toolbox save-map service did '
                    f'not become ready within {self._service_wait_timeout:.1f}s')
                self._finish(False)
        elif self._state == 'waiting_for_serialize_service':
            if self._serialize_client.service_is_ready():
                self._request_pose_graph_save()
            elif self._service_wait_expired():
                self.get_logger().warning(
                    'Occupancy map was saved, but pose graph service did not '
                    f'become ready within {self._service_wait_timeout:.1f}s')
                self._finish(True)
        elif self._state == 'saving_map' and self._service_call_expired():
            self.get_logger().error(
                'Map was not saved: save-map service call exceeded '
                f'{self._service_call_timeout:.1f}s')
            self._finish(False)
            self._active_future.cancel()
        elif (self._state == 'serializing_pose_graph' and
              self._service_call_expired()):
            self.get_logger().warning(
                'Occupancy map was saved, but pose graph service call exceeded '
                f'{self._service_call_timeout:.1f}s')
            self._finish(True)
            self._active_future.cancel()

    def _service_wait_expired(self):
        return (time.monotonic() - self._wait_started) >= self._service_wait_timeout

    def _service_call_expired(self):
        return (time.monotonic() - self._call_started) >= self._service_call_timeout

    def _request_map_save(self):
        request = SaveMap.Request()
        request.name = String(data=str(self._output_basename))
        self._state = 'saving_map'
        self._call_started = time.monotonic()
        self.get_logger().info(f"Calling '{self._save_service}'")
        self._active_future = self._save_client.call_async(request)
        self._active_future.add_done_callback(self._map_save_done)

    def _map_save_done(self, future):
        if self._state != 'saving_map':
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS futures surface transport errors
            self.get_logger().error(f'Map save service call failed: {exc}')
            self._finish(False)
            return

        if response.result != SaveMap.Response.RESULT_SUCCESS:
            self.get_logger().error(
                f'Map save failed with SLAM Toolbox result {response.result}')
            self._finish(False)
            return

        self.get_logger().info(
            f"Occupancy map saved as '{self._output_basename}.yaml' and image")
        if not self._serialize_enabled:
            self._finish(True)
            return

        self._state = 'waiting_for_serialize_service'
        self._wait_started = time.monotonic()
        if not self._serialize_client.service_is_ready():
            self.get_logger().info(f"Waiting for '{self._serialize_service}'")
        self._poll_service()

    def _request_pose_graph_save(self):
        request = SerializePoseGraph.Request()
        request.filename = str(self._output_basename)
        self._state = 'serializing_pose_graph'
        self._call_started = time.monotonic()
        self.get_logger().info(f"Calling '{self._serialize_service}'")
        self._active_future = self._serialize_client.call_async(request)
        self._active_future.add_done_callback(self._pose_graph_save_done)

    def _pose_graph_save_done(self, future):
        if self._state != 'serializing_pose_graph':
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS futures surface transport errors
            self.get_logger().warning(
                f'Occupancy map was saved, but pose graph call failed: {exc}')
            self._finish(True)
            return

        if response.result == SerializePoseGraph.Response.RESULT_SUCCESS:
            self.get_logger().info(
                f"Pose graph saved with basename '{self._output_basename}'")
        else:
            self.get_logger().warning(
                'Occupancy map was saved, but pose graph serialization '
                f'failed with result {response.result}')
        self._finish(True)

    def _finish(self, map_saved):
        self._state = 'complete' if map_saved else 'failed'
        self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutoMapSaver()
        rclpy.spin(node)
    except (OSError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(f'Invalid map saver configuration: {exc}')
        else:
            print(f'Invalid map saver configuration: {exc}')
        raise
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
