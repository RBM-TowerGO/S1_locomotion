"""ROS client for the optional map display. Its only command output is a service call."""
from collections import deque
import hashlib
import io
import json
import math
import secrets
import threading
import time

import numpy as np
from PIL import Image
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from astribot_nav_interfaces.srv import NavigationCommand

from . import FRAME, NS
from .geometry import cloud_xyz, scan_points


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_web_client', namespace=NS.rstrip('/'))
        self.commands = self.create_client(NavigationCommand, NS+'command')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.token = secrets.token_urlsafe(32)
        self.session = None
        self.localization_id = None
        self.trajectory = deque(maxlen=10000)
        self.scans = deque(maxlen=20)
        self.clouds = {}
        self.paths = {}
        self.map_version = None
        self.png = None
        self.snapshot = None
        self.received_at = 0.
        self.lock = threading.Lock()
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, NS+'map', self.on_map, map_qos)
        self.create_subscription(String, NS+'task_state', self.on_state, qos_profile_sensor_data)
        self.create_subscription(LaserScan, NS+'scan', self.on_scan, qos_profile_sensor_data)
        for side in ('front', 'back'):
            self.create_subscription(PointCloud2, NS+'obstacles/'+side,
                                     lambda msg, key=side: self.clouds.update({key: msg}), qos_profile_sensor_data)
        for key in ('plan', 'local_plan'):
            self.create_subscription(Path, NS+key, lambda msg, k=key: self.on_path(k, msg), 1)

    def on_map(self, msg):
        cells = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        pixels = np.full(cells.shape, 205, dtype=np.uint8)
        pixels[cells == 0] = 245
        pixels[cells > 0] = 35
        out = io.BytesIO()
        Image.fromarray(pixels[::-1]).save(out, format='PNG')
        png = out.getvalue()
        with self.lock:
            self.png = png
            self.map_version = hashlib.sha256(png).hexdigest()

    def project(self, points, source, target, stamp):
        if source == target:
            return points[:, :2].tolist()
        tf = self.tf.lookup_transform(target, source, stamp).transform
        q, p = tf.rotation, tf.translation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return (points @ rot.T + [p.x, p.y, p.z])[:, :2].tolist()

    def on_path(self, key, msg):
        points = np.array([[v.pose.position.x, v.pose.position.y, v.pose.position.z]
                           for v in msg.poses]).reshape(-1, 3)
        try:
            self.paths[key] = self.project(points, msg.header.frame_id, FRAME['map'], Time())
        except TransformException:
            self.paths[key] = []

    def on_scan(self, msg):
        self.scans.append((Time.from_msg(msg.header.stamp).nanoseconds, msg))

    def scan_overlay(self, state, now):
        overlay = dict(laser_scan=[], local_scan=None, laser_scan_age=None, laser_scan_status='等待雷达扫描')
        source = state.get('scan_source_stamp')
        if source is None:
            return overlay
        age = now - source
        overlay['laser_scan_age'] = round(age, 3)
        if not -.05 <= age <= state['scan_source_timeout'] or state['sensor_error']:
            overlay['laser_scan_status'] = '扫描过期或定位数据异常，暂不叠加'
            return overlay
        # Raw-pair scans can be projected at a later reference time. Only use the
        # exact scan associated with the source age reported by the supervisor.
        scan = next((m for ns, m in reversed(self.scans) if ns == state['scan_reference_ns']), None)
        if scan is None:
            return overlay
        stamp = Time.from_msg(scan.header.stamp)
        points = scan_points(scan.ranges, scan.range_min, scan.range_max)
        frame = scan.header.frame_id
        try:
            local = self.project(points, frame, FRAME['base'], stamp)
            overlay['local_scan'] = dict(frame=FRAME['base'], stamp=stamp.nanoseconds*1e-9,
                                         age=round(age, 3), points=local)
            overlay['laser_scan_status'] = '局部实时扫描可用；选点可预览地图对齐'
            overlay['laser_scan'] = self.project(points, frame, FRAME['map'], stamp)
            overlay['laser_scan_status'] = ('延迟水平扫描' if age > .5 else '实时水平扫描') + f' · {len(points)} 点'
        except TransformException:
            pass
        return overlay

    def obstacle_overlay(self, now):
        overlay = []
        for cloud in self.clouds.values():
            stamp = Time.from_msg(cloud.header.stamp)
            if not -.05 <= now - stamp.nanoseconds*1e-9 <= .5:
                continue
            points = cloud_xyz(cloud)
            try:
                overlay.extend(self.project(points[::max(1, len(points)//600)], cloud.header.frame_id,
                                            FRAME['map'], stamp))
            except TransformException:
                pass
        return overlay

    def on_state(self, msg):
        state = json.loads(msg.data)
        session = state['session']
        loc = state['localization']['id']
        if self.session is not None and session != self.session:
            self.paths.clear()
            self.scans.clear()
            self.clouds.clear()
            self.token = secrets.token_urlsafe(32)
        if session != self.session or loc != self.localization_id:
            self.trajectory.clear()
        self.session, self.localization_id = session, loc
        pose = state['pose']
        if pose and state['ready_checks']['tf'] and (not self.trajectory or math.dist(pose[:2], self.trajectory[-1][:2]) > .02):
            self.trajectory.append(pose)
        now = self.get_clock().now().nanoseconds * 1e-9
        state.update(self.scan_overlay(state, now), trajectory=list(self.trajectory),
                     obstacles=self.obstacle_overlay(now), map_version=self.map_version, **self.paths)
        with self.lock:
            self.snapshot = state
            self.received_at = time.monotonic()

    def state(self):
        with self.lock:
            if self.snapshot is None or time.monotonic() - self.received_at > 2:
                raise ConnectionError('等待导航节点的实时状态')
            return self.snapshot

    def call(self, op, obj):
        if not self.commands.service_is_ready():
            raise ConnectionError('导航命令服务尚未连接')
        request = NavigationCommand.Request()
        request.operation = op
        if op != 'stop':
            request.owner = obj['owner']
            keys = ('dx', 'dy', 'dyaw') if op == 'nudge' else ('x', 'y', 'yaw')
            request.x, request.y, request.yaw = (float(obj[k]) for k in keys)
            if op != 'localize':
                if isinstance(obj['speed'], bool):
                    raise ValueError('速度必须是数字')
                request.speed = float(obj['speed'])
        # The HTTP worker waits here; ROS callbacks keep spinning on the main thread.
        done = threading.Event()
        future = self.commands.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(3):
            self.commands.remove_pending_request(future)
            raise TimeoutError('尚未收到受理结果，请查看任务状态')
        reply = future.result()
        if not reply.accepted:
            raise ValueError(reply.reason)
        return dict(queued=True, accepted=True, task=reply.task_id or None, localization_id=reply.localization_id)
