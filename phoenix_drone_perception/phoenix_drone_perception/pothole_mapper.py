"""
pothole_mapper.py — Georeferenced Pothole Location Mapper
═══════════════════════════════════════════════════════════
Subscribes to:
  • Pothole detections (pixel bounding boxes) from the detector
  • Drone odometry and GPS

Projects each detection from image pixels onto the road plane, fuses
repeat sightings of the same pothole, and publishes a georeferenced
map with WGS-84 coordinates plus RViz markers.

Usage:
    ros2 run phoenix_drone_perception pothole_mapper
"""
import json
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from phoenix_drone_perception.geo_utils import (
    pixel_to_world,
    enu_to_geodetic,
    estimate_diameter_m,
    focal_from_fov,
)


def quat_to_yaw(q) -> float:
    """Extract yaw from a geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


@dataclass
class PotholeRecord:
    """A single georeferenced pothole, fused from one or more sightings."""
    id: int
    world_x: float
    world_y: float
    latitude: float
    longitude: float
    altitude: float
    confidence: float
    diameter_m: float
    sightings: int = 1
    first_seen: float = 0.0
    last_seen: float = 0.0
    severity: str = 'LOW'

    def to_dict(self):
        d = asdict(self)
        # Round for a readable report
        for k in ('world_x', 'world_y', 'diameter_m', 'confidence'):
            d[k] = round(d[k], 3)
        for k in ('latitude', 'longitude'):
            d[k] = round(d[k], 8)
        d['altitude'] = round(d['altitude'], 2)
        return d


class PotholeMapper(Node):
    """
    Aggregates pothole detections with drone pose to build a
    georeferenced map of road defects.
    """

    def __init__(self):
        super().__init__('pothole_mapper')

        # ── Parameters ──
        self.declare_parameter('min_separation_m', 1.5)
        self.declare_parameter('output_file', '/tmp/pothole_map.json')
        self.declare_parameter('report_file', '/tmp/pothole_report.txt')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('ref_latitude', 28.6139)
        self.declare_parameter('ref_longitude', 77.2090)
        self.declare_parameter('ref_altitude', 216.0)
        self.declare_parameter('road_plane_z', 0.02)
        self.declare_parameter('min_confidence', 0.30)
        self.declare_parameter('min_sightings_to_confirm', 3)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('horizontal_fov', 1.3962634)

        self._min_sep = self.get_parameter('min_separation_m').value
        self._output_file = self.get_parameter('output_file').value
        self._report_file = self.get_parameter('report_file').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self._ref_lat = self.get_parameter('ref_latitude').value
        self._ref_lon = self.get_parameter('ref_longitude').value
        self._ref_alt = self.get_parameter('ref_altitude').value
        self._road_z = self.get_parameter('road_plane_z').value
        self._min_conf = self.get_parameter('min_confidence').value
        self._min_sightings = self.get_parameter('min_sightings_to_confirm').value

        self._img_w = self.get_parameter('image_width').value
        self._img_h = self.get_parameter('image_height').value
        fov = self.get_parameter('horizontal_fov').value

        # Camera intrinsics — refined if CameraInfo arrives
        self._fx = focal_from_fov(self._img_w, fov)
        self._fy = self._fx
        self._cx = self._img_w / 2.0
        self._cy = self._img_h / 2.0
        self._have_caminfo = False

        # ── State ──
        self._potholes: List[PotholeRecord] = []
        self._next_id = 1
        self._drone_xyz = None
        self._drone_yaw = 0.0
        self._pose_history = deque()
        self._history_span = 3.0     # seconds of pose history to retain
        self._gps: Optional[NavSatFix] = None
        self._total_detections = 0

        # ── QoS ──
        # GPS (gazebo_ros_gps_sensor) and CameraInfo both publish RELIABLE.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──
        self.create_subscription(
            Odometry, '/phoenix/ground_truth/odom', self._odom_cb, 10)
        self.create_subscription(
            NavSatFix, '/phoenix/gps/fix', self._gps_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, '/phoenix/camera/camera_info',
            self._caminfo_cb, sensor_qos)
        # Detections arrive as JSON with pixel bboxes
        self.create_subscription(
            String, '/phoenix/perception/detections',
            self._detections_cb, 10)

        # ── Publishers ──
        self._map_pub = self.create_publisher(
            String, '/phoenix/pothole_map', 10)
        self._summary_pub = self.create_publisher(
            String, '/phoenix/pothole_map/summary', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/phoenix/pothole_markers', 10)

        self._timer = self.create_timer(1.0 / publish_rate, self._publish_map)

        self.get_logger().info(
            '╔══════════════════════════════════════════════╗\n'
            '║  PhoenixDrone Pothole Mapper Started         ║\n'
            f'║  Separation filter: {self._min_sep:.1f}m'
            f'{" " * 22}║\n'
            f'║  Ref GPS: {self._ref_lat:.4f}, {self._ref_lon:.4f}'
            f'{" " * 14}║\n'
            f'║  Output: {os.path.basename(self._output_file):35s}║\n'
            '╚══════════════════════════════════════════════╝'
        )

    # ═══════════════════════════════════════════════════════════
    #  CALLBACKS
    # ═══════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._drone_xyz = (p.x, p.y, p.z)
        self._drone_yaw = quat_to_yaw(msg.pose.pose.orientation)

        # Keep a short history so a detection can be projected with the pose
        # the drone actually had when its frame was captured.
        t = self.get_clock().now().nanoseconds * 1e-9
        self._pose_history.append((t, p.x, p.y, p.z, self._drone_yaw))
        cutoff = t - self._history_span
        while self._pose_history and self._pose_history[0][0] < cutoff:
            self._pose_history.popleft()

    def _pose_at(self, stamp_s):
        """Interpolate the drone pose at a past timestamp."""
        hist = self._pose_history
        if not hist:
            return None
        if stamp_s <= hist[0][0]:
            _, x, y, z, yaw = hist[0]
            return x, y, z, yaw
        if stamp_s >= hist[-1][0]:
            _, x, y, z, yaw = hist[-1]
            return x, y, z, yaw
        for i in range(len(hist) - 1):
            t0, x0, y0, z0, yaw0 = hist[i]
            t1, x1, y1, z1, yaw1 = hist[i + 1]
            if t0 <= stamp_s <= t1:
                span = t1 - t0
                a = 0.0 if span <= 0 else (stamp_s - t0) / span
                dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
                return (x0 + (x1 - x0) * a,
                        y0 + (y1 - y0) * a,
                        z0 + (z1 - z0) * a,
                        yaw0 + dyaw * a)
        _, x, y, z, yaw = hist[-1]
        return x, y, z, yaw

    def _gps_cb(self, msg: NavSatFix):
        self._gps = msg

    def _caminfo_cb(self, msg: CameraInfo):
        """Use the real projection matrix once Gazebo publishes it."""
        # msg.k is a fixed-size numpy array; test its length, not its truth
        if len(msg.k) >= 6 and msg.k[0] > 0:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]
            if not self._have_caminfo:
                self._have_caminfo = True
                self.get_logger().info(
                    f'Camera intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
                    f'cx={self._cx:.1f} cy={self._cy:.1f}')

    def _detections_cb(self, msg: String):
        """Project each detection onto the road plane and fuse it."""
        if self._drone_xyz is None:
            return

        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        detections = payload.get('detections', [])
        if not detections:
            return

        # Project with the pose the drone had when this FRAME was captured,
        # not the pose it has now — otherwise motion during inference smears
        # one pothole into a trail of duplicates.
        stamp = payload.get('stamp')
        pose = None
        if stamp:
            stamp_s = stamp.get('sec', 0) + stamp.get('nanosec', 0) * 1e-9
            pose = self._pose_at(stamp_s)
        if pose is None:
            pose = (self._drone_xyz[0], self._drone_xyz[1],
                    self._drone_xyz[2], self._drone_yaw)
        drone_x, drone_y, drone_z, drone_yaw = pose

        # Height of the camera above the road surface
        height = drone_z - self._road_z
        if height < 0.5:
            return  # too low / on the ground — projection is meaningless

        for det in detections:
            conf = float(det.get('confidence', 0.0))
            if conf < self._min_conf:
                continue
            bbox = det.get('bbox')
            if not bbox or len(bbox) != 4:
                continue

            u = (bbox[0] + bbox[2]) / 2.0
            v = (bbox[1] + bbox[3]) / 2.0

            wx, wy = pixel_to_world(
                u, v, self._cx, self._cy, self._fx, self._fy,
                drone_x, drone_y, height, drone_yaw)

            diameter = estimate_diameter_m(bbox, self._fx, self._fy, height)
            self._total_detections += 1
            self._fuse(wx, wy, conf, diameter)

    # ═══════════════════════════════════════════════════════════
    #  FUSION
    # ═══════════════════════════════════════════════════════════

    @staticmethod
    def _severity(diameter: float) -> str:
        if diameter >= 0.9:
            return 'HIGH'
        if diameter >= 0.5:
            return 'MEDIUM'
        return 'LOW'

    def _fuse(self, wx: float, wy: float, conf: float, diameter: float):
        """Merge a new sighting into an existing pothole, or create one."""
        now = time.time()

        nearest = None
        nearest_d = float('inf')
        for rec in self._potholes:
            d = math.hypot(rec.world_x - wx, rec.world_y - wy)
            if d < nearest_d:
                nearest_d, nearest = d, rec

        if nearest is not None and nearest_d < self._min_sep:
            # Running average weighted by sighting count
            n = nearest.sightings
            nearest.world_x = (nearest.world_x * n + wx) / (n + 1)
            nearest.world_y = (nearest.world_y * n + wy) / (n + 1)
            nearest.diameter_m = (nearest.diameter_m * n + diameter) / (n + 1)
            nearest.confidence = max(nearest.confidence, conf)
            nearest.sightings = n + 1
            nearest.last_seen = now
            nearest.severity = self._severity(nearest.diameter_m)
            lat, lon, alt = enu_to_geodetic(
                nearest.world_x, nearest.world_y, 0.0,
                self._ref_lat, self._ref_lon, self._ref_alt)
            nearest.latitude, nearest.longitude, nearest.altitude = lat, lon, alt
            return

        lat, lon, alt = enu_to_geodetic(
            wx, wy, 0.0, self._ref_lat, self._ref_lon, self._ref_alt)

        rec = PotholeRecord(
            id=self._next_id,
            world_x=wx, world_y=wy,
            latitude=lat, longitude=lon, altitude=alt,
            confidence=conf, diameter_m=diameter,
            sightings=1, first_seen=now, last_seen=now,
            severity=self._severity(diameter),
        )
        self._potholes.append(rec)
        self._next_id += 1
        self.get_logger().info(
            f'★ New pothole #{rec.id} at ({wx:.1f}, {wy:.1f})m  '
            f'⌀{diameter:.2f}m  {rec.severity}  '
            f'GPS {lat:.6f},{lon:.6f}')

    # ═══════════════════════════════════════════════════════════
    #  OUTPUT
    # ═══════════════════════════════════════════════════════════

    def _confirmed(self) -> List[PotholeRecord]:
        return [p for p in self._potholes if p.sightings >= self._min_sightings]

    def _publish_map(self):
        confirmed = self._confirmed()

        payload = {
            'timestamp': time.time(),
            'reference': {
                'latitude': self._ref_lat,
                'longitude': self._ref_lon,
                'altitude': self._ref_alt,
            },
            'total_raw_detections': self._total_detections,
            'candidate_count': len(self._potholes),
            'confirmed_count': len(confirmed),
            'potholes': [p.to_dict() for p in confirmed],
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._map_pub.publish(msg)

        by_sev = {'HIGH': 0, 'MEDIUM': 0, 'LOW': 0}
        for p in confirmed:
            by_sev[p.severity] = by_sev.get(p.severity, 0) + 1

        summary = String()
        summary.data = (
            f'Potholes confirmed: {len(confirmed)} '
            f'(HIGH {by_sev["HIGH"]} / MED {by_sev["MEDIUM"]} / '
            f'LOW {by_sev["LOW"]}) | candidates: {len(self._potholes)} '
            f'| raw detections: {self._total_detections}'
        )
        self._summary_pub.publish(summary)

        self._publish_markers(confirmed)
        self._write_files(payload, confirmed)

    def _publish_markers(self, confirmed: List[PotholeRecord]):
        """RViz markers: a disc + id label per confirmed pothole."""
        arr = MarkerArray()
        colours = {
            'HIGH': ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.85),
            'MEDIUM': ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.85),
            'LOW': ColorRGBA(r=1.0, g=0.95, b=0.2, a=0.85),
        }

        for p in confirmed:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'potholes'
            m.id = p.id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = p.world_x
            m.pose.position.y = p.world_y
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            d = max(0.3, p.diameter_m)
            m.scale = Vector3(x=d, y=d, z=0.12)
            m.color = colours.get(p.severity, colours['LOW'])
            arr.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = 'pothole_labels'
            t.id = p.id
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = p.world_x
            t.pose.position.y = p.world_y
            t.pose.position.z = 1.2
            t.pose.orientation.w = 1.0
            t.scale.z = 0.8
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            t.text = f'#{p.id} {p.diameter_m:.2f}m {p.severity}'
            arr.markers.append(t)

        self._marker_pub.publish(arr)

    def _write_files(self, payload: dict, confirmed: List[PotholeRecord]):
        """Persist the map as JSON and a human-readable report."""
        try:
            with open(self._output_file, 'w') as f:
                json.dump(payload, f, indent=2)
        except OSError as e:
            self.get_logger().warn(f'Could not write map file: {e}',
                                   throttle_duration_sec=10.0)

        try:
            lines = [
                '═' * 78,
                ' PHOENIX DRONE — ROAD POTHOLE SURVEY REPORT',
                '═' * 78,
                f' Generated     : {time.strftime("%Y-%m-%d %H:%M:%S")}',
                f' Reference GPS : {self._ref_lat:.6f}, {self._ref_lon:.6f}',
                f' Raw detections: {self._total_detections}',
                f' Confirmed     : {len(confirmed)}',
                '─' * 78,
                f'{"ID":>4} {"X(m)":>8} {"Y(m)":>8} {"⌀(m)":>7} '
                f'{"SEV":>7} {"SEEN":>5}  {"LATITUDE":>12} {"LONGITUDE":>12}',
                '─' * 78,
            ]
            for p in sorted(confirmed, key=lambda r: r.world_x):
                lines.append(
                    f'{p.id:>4} {p.world_x:>8.2f} {p.world_y:>8.2f} '
                    f'{p.diameter_m:>7.2f} {p.severity:>7} {p.sightings:>5}  '
                    f'{p.latitude:>12.6f} {p.longitude:>12.6f}')
            lines.append('═' * 78)
            with open(self._report_file, 'w') as f:
                f.write('\n'.join(lines) + '\n')
        except OSError as e:
            self.get_logger().warn(f'Could not write report: {e}',
                                   throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = PotholeMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pothole mapper stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
