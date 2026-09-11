"""
navigation_monitor.py — Flight Navigation Telemetry Monitor
═════════════════════════════════════════════════════════════
Subscribes to drone odometry, GPS, flight state, and survey topics to
produce a unified navigation telemetry stream for the PhoenixDrone.

Published topics:
  • /phoenix/navigation/telemetry  (std_msgs/String)  — JSON telemetry
  • /phoenix/navigation/phase      (std_msgs/String)  — current flight phase

Usage:
    ros2 run phoenix_drone_perception navigation_monitor
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


class NavigationMonitor(Node):
    """
    Real-time navigation telemetry monitor for the PhoenixDrone.

    Fuses ground-truth odometry, GPS fixes, flight-controller state,
    and survey progress into a single JSON telemetry stream.
    """

    # gazebo_ros_gps_sensor publishes RELIABLE; BEST_EFFORT would be an
    # incompatible-QoS match and silently receive nothing.
    SENSOR_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self):
        super().__init__('navigation_monitor')

        # ── Parameters ──
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('gps_topic', '/phoenix/gps/fix')
        self.declare_parameter('survey_topic', '/phoenix/survey/status')

        publish_rate = self.get_parameter('publish_rate_hz').value
        gps_topic = self.get_parameter('gps_topic').value
        survey_topic = self.get_parameter('survey_topic').value

        # ── Cached state ──
        self._position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
        self._heading_deg = 0.0
        self._altitude_agl = 0.0
        self._gps = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self._gps_valid = False
        self._flight_phase = 'INITIALIZING'
        self._survey_status = 'WAITING'
        self._arming_state = 'DISARMED'
        self._nav_state = 'UNKNOWN'
        self._start_time = time.time()
        self._pos_received = False
        self._pothole_summary = ''

        # ── Subscribers ──
        self.create_subscription(
            Odometry,
            '/phoenix/ground_truth/odom',
            self._odom_cb,
            10,
        )
        self.create_subscription(
            String,
            '/phoenix/flight_state',
            self._flight_state_cb,
            10,
        )
        self.create_subscription(
            NavSatFix,
            gps_topic,
            self._gps_cb,
            self.SENSOR_QOS,
        )
        self.create_subscription(
            String,
            survey_topic,
            self._survey_cb,
            10,
        )
        self.create_subscription(
            String,
            '/phoenix/pothole_map/summary',
            self._pothole_cb,
            10,
        )

        # ── Publishers ──
        self._telemetry_pub = self.create_publisher(
            String,
            '/phoenix/navigation/telemetry',
            10,
        )
        self._phase_pub = self.create_publisher(
            String,
            '/phoenix/navigation/phase',
            10,
        )

        # ── Periodic telemetry publisher ──
        self._timer = self.create_timer(
            1.0 / publish_rate, self._publish_telemetry
        )

        self.get_logger().info(
            '╔══════════════════════════════════════════════════╗\n'
            '║  PhoenixDrone Navigation Monitor Started         ║\n'
            f'║  Telemetry rate: {publish_rate:.1f} Hz'
            f'{" " * (31 - len(f"{publish_rate:.1f}"))}║\n'
            f'║  GPS topic: {gps_topic[:35]:35s}║\n'
            '╚══════════════════════════════════════════════════╝'
        )

    # ═══════════════════════════════════════════════════════════
    #  CALLBACKS
    # ═══════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        """Cache pose and velocity from ground-truth odometry."""
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._position['x'] = float(p.x)
        self._position['y'] = float(p.y)
        self._position['z'] = float(p.z)
        self._velocity['vx'] = float(v.x)
        self._velocity['vy'] = float(v.y)
        self._velocity['vz'] = float(v.z)

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._heading_deg = math.degrees(math.atan2(siny, cosy)) % 360.0

        self._altitude_agl = max(0.0, float(p.z))
        self._pos_received = True

    def _flight_state_cb(self, msg: String):
        """Map the flight controller's state onto a high-level phase."""
        state = msg.data.strip().upper()
        self._nav_state = state
        self._arming_state = 'DISARMED' if state == 'IDLE' else 'ARMED'
        self._flight_phase = self._resolve_phase()

    def _pothole_cb(self, msg: String):
        """Cache the latest pothole map summary."""
        self._pothole_summary = msg.data

    def _gps_cb(self, msg: NavSatFix):
        """Cache latest GPS fix."""
        self._gps['lat'] = msg.latitude
        self._gps['lon'] = msg.longitude
        self._gps['alt'] = msg.altitude
        self._gps_valid = (msg.latitude != 0.0 or msg.longitude != 0.0)

    def _survey_cb(self, msg: String):
        """Cache latest survey status string."""
        self._survey_status = msg.data

    # ═══════════════════════════════════════════════════════════
    #  PHASE RESOLUTION
    # ═══════════════════════════════════════════════════════════

    def _resolve_phase(self) -> str:
        """Determine the high-level flight phase from controller state."""
        return {
            'IDLE': 'GROUND_IDLE',
            'TAKEOFF': 'TAKEOFF',
            'HOVER': 'HOVERING',
            'NAVIGATE': 'NAVIGATING',
            'LAND': 'LANDING',
        }.get(self._nav_state, 'IN_FLIGHT' if self._pos_received
              else 'INITIALIZING')

    # ═══════════════════════════════════════════════════════════
    #  TELEMETRY PUBLISHING
    # ═══════════════════════════════════════════════════════════

    def _publish_telemetry(self):
        """Publish the unified navigation telemetry as JSON."""
        # Ground speed (horizontal)
        ground_speed = math.sqrt(
            self._velocity['vx'] ** 2 + self._velocity['vy'] ** 2
        )
        # Total 3D speed
        total_speed = math.sqrt(
            self._velocity['vx'] ** 2
            + self._velocity['vy'] ** 2
            + self._velocity['vz'] ** 2
        )

        elapsed = time.time() - self._start_time

        telemetry = {
            'timestamp': time.time(),
            'elapsed_s': round(elapsed, 1),
            'flight_phase': self._flight_phase,
            'arming_state': self._arming_state,
            'nav_state': self._nav_state,
            'position_enu': {
                'x_m': round(self._position['x'], 2),
                'y_m': round(self._position['y'], 2),
                'z_m': round(self._position['z'], 2),
            },
            'altitude_agl_m': round(self._altitude_agl, 2),
            'velocity': {
                'ground_speed_ms': round(ground_speed, 2),
                'vertical_speed_ms': round(abs(self._velocity['vz']), 2),
                'total_speed_ms': round(total_speed, 2),
            },
            'heading_deg': round(self._heading_deg, 1),
            'gps': {
                'valid': self._gps_valid,
                'latitude': round(self._gps['lat'], 7),
                'longitude': round(self._gps['lon'], 7),
                'altitude_m': round(self._gps['alt'], 2),
            },
            'survey_status': self._survey_status,
            'pothole_summary': self._pothole_summary,
        }

        # Publish telemetry JSON
        tel_msg = String()
        tel_msg.data = json.dumps(telemetry, indent=2)
        self._telemetry_pub.publish(tel_msg)

        # Publish flight phase
        phase_msg = String()
        phase_msg.data = self._flight_phase
        self._phase_pub.publish(phase_msg)

        # Periodic log (every ~5 seconds at 2 Hz = every 10th call)
        tick = int(elapsed * 2) % 10
        if tick == 0 and self._pos_received:
            self.get_logger().info(
                f'🛩  {self._flight_phase} | '
                f'Alt: {self._altitude_agl:.1f}m | '
                f'Speed: {ground_speed:.1f}m/s | '
                f'Hdg: {self._heading_deg:.0f}° | '
                f'ENU: ({self._position["x"]:.1f}, '
                f'{self._position["y"]:.1f}, '
                f'{self._position["z"]:.1f}) | '
                f'GPS: {"✓" if self._gps_valid else "✗"} | '
                f'Survey: {self._survey_status}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation monitor stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
