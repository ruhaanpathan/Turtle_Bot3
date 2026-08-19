"""
Self-contained teleop + mapping web server node.
- HTTP on port 8080: joystick + live map UI
- WebSocket on port 9090: cmd_vel, map stream, save/pause
No rosbridge dependency required.
"""

import os
import json
import socket
import asyncio
import threading
import base64
import subprocess
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from functools import partial
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_share_directory

import websockets
import websockets.asyncio.server


class WebServerNode(Node):

    def __init__(self):
        super().__init__('web_server')

        self.declare_parameter('http_port', 8080)
        self.declare_parameter('ws_port', 9090)
        self.declare_parameter('map_save_dir', os.path.expanduser('~/maps'))

        self.http_port = self.get_parameter('http_port').get_parameter_value().integer_value
        self.ws_port = self.get_parameter('ws_port').get_parameter_value().integer_value
        self.map_save_dir = self.get_parameter('map_save_dir').get_parameter_value().string_value

        os.makedirs(self.map_save_dir, exist_ok=True)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Map subscriber (SLAM publishes with transient-local durability)
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos
        )
        self.latest_map_msg = None
        self.latest_map_json = None
        self.last_map_broadcast = 0.0

        # WebSocket bookkeeping
        self.ws_clients = set()
        self.ws_loop = None
        self.mapping_paused = False

        # Locate web assets
        pkg_share = get_package_share_directory('mobile_teleop')
        self.web_dir = os.path.join(pkg_share, 'web')
        if not os.path.isdir(self.web_dir):
            self.get_logger().error(f'Web dir not found: {self.web_dir}')
            return

        # HTTP server
        handler = partial(QuietHTTPHandler, directory=self.web_dir)
        HTTPServer.allow_reuse_address = True
        self.httpd = HTTPServer(('0.0.0.0', self.http_port), handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

        # WebSocket server
        threading.Thread(target=self._run_ws, daemon=True).start()

        ip = self._get_local_ip()
        self.get_logger().info('=' * 60)
        self.get_logger().info('  📱  MOBILE TELEOP + MAPPING')
        self.get_logger().info(f'  Phone URL:   http://{ip}:{self.http_port}')
        self.get_logger().info(f'  WebSocket:   ws://{ip}:{self.ws_port}')
        self.get_logger().info(f'  Maps dir:    {self.map_save_dir}')
        self.get_logger().info('=' * 60)

    # ── Map handling ────────────────────────────────────────

    def _map_cb(self, msg):
        self.latest_map_msg = msg
        now = time.time()
        if now - self.last_map_broadcast < 1.0:
            return
        self.last_map_broadcast = now

        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return

        data = np.array(msg.data, dtype=np.int8)
        pixels = np.full(len(data), 30, dtype=np.uint8)
        pixels[data == 0] = 60
        occ = data > 0
        pixels[occ] = 255

        self.latest_map_json = json.dumps({
            'type': 'map',
            'width': w,
            'height': h,
            'resolution': float(msg.info.resolution),
            'origin_x': float(msg.info.origin.position.x),
            'origin_y': float(msg.info.origin.position.y),
            'data': base64.b64encode(pixels.tobytes()).decode('ascii'),
        })

        if self.ws_loop and self.ws_clients:
            asyncio.run_coroutine_threadsafe(self._broadcast_map(), self.ws_loop)

    async def _broadcast_map(self):
        msg = self.latest_map_json
        if not msg:
            return
        dead = set()
        for c in self.ws_clients.copy():
            try:
                await c.send(msg)
            except Exception:
                dead.add(c)
        self.ws_clients -= dead

    # ── WebSocket server ────────────────────────────────────

    def _run_ws(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self.ws_loop = loop
        loop.run_until_complete(self._ws_serve())

    async def _ws_serve(self):
        async with websockets.asyncio.server.serve(
            self._ws_handler, '0.0.0.0', self.ws_port
        ):
            self.get_logger().info(f'WebSocket listening on :{self.ws_port}')
            await asyncio.Future()

    async def _ws_handler(self, ws):
        addr = ws.remote_address[0] if ws.remote_address else '?'
        self.get_logger().info(f'📱 Connected: {addr}')
        self.ws_clients.add(ws)

        # Send cached map immediately
        if self.latest_map_json:
            try:
                await ws.send(self.latest_map_json)
            except Exception:
                pass

        try:
            async for raw in ws:
                try:
                    data = json.loads(raw)
                    action = data.get('action', '')

                    if action == 'save_map':
                        res = await asyncio.to_thread(self._save_map)
                        await ws.send(json.dumps({
                            'type': 'save_result', **res
                        }))

                    elif action == 'pause_map':
                        res = await asyncio.to_thread(self._toggle_pause)
                        await ws.send(json.dumps({
                            'type': 'pause_result',
                            'paused': self.mapping_paused,
                            **res,
                        }))

                    else:
                        # cmd_vel (with or without action field)
                        lx = float(data.get('linear_x', 0.0))
                        az = float(data.get('angular_z', 0.0))
                        t = Twist()
                        t.linear.x = lx
                        t.angular.z = az
                        self.cmd_vel_pub.publish(t)

                except (json.JSONDecodeError, ValueError, TypeError) as e:
                    self.get_logger().warn(f'Bad msg: {e}')
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(ws)
            self.get_logger().info(f'📱 Disconnected: {addr}')
            try:
                self.cmd_vel_pub.publish(Twist())
            except Exception:
                pass

    # ── Save map ────────────────────────────────────────────

    def _save_map(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fpath = os.path.join(self.map_save_dir, f'map_{ts}')

        if not self.latest_map_msg:
            return {'success': False, 'message': 'No map data yet'}

        try:
            self._write_pgm_yaml(fpath, self.latest_map_msg)
            self.get_logger().info(f'Map saved → {fpath}')
            return {'success': True, 'message': f'Saved map_{ts}'}
        except Exception as e:
            self.get_logger().error(f'Save failed: {e}')
            return {'success': False, 'message': str(e)}

    def _write_pgm_yaml(self, fpath, msg):
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full_like(data, 205, dtype=np.uint8)
        img[data == 0] = 254
        img[data > 0] = 0
        img = np.flipud(img)

        pgm = fpath + '.pgm'
        with open(pgm, 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            f.write(img.tobytes())

        with open(fpath + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(pgm)}\n')
            f.write(f'mode: trinary\nresolution: {res}\n')
            f.write(f'origin: [{ox}, {oy}, 0.0]\n')
            f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')

    # ── Pause SLAM ──────────────────────────────────────────

    def _toggle_pause(self):
        self.mapping_paused = not self.mapping_paused
        state = 'paused' if self.mapping_paused else 'resumed'
        try:
            subprocess.run(
                ['ros2', 'service', 'call',
                 '/slam_toolbox/pause_new_measurements',
                 'slam_toolbox/srv/Pause', '{}'],
                capture_output=True, timeout=5, env={**os.environ},
            )
        except Exception:
            pass
        self.get_logger().info(f'Mapping {state}')
        return {'message': f'Mapping {state}'}

    # ── Helpers ─────────────────────────────────────────────

    def _get_local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return '127.0.0.1'

    def destroy_node(self):
        if hasattr(self, 'httpd'):
            self.httpd.shutdown()
        super().destroy_node()


class QuietHTTPHandler(SimpleHTTPRequestHandler):
    def log_message(self, fmt, *a):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
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
