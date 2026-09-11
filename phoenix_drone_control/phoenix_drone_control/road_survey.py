#!/usr/bin/env python3
"""
road_survey.py — Autonomous Road Survey Node for PhoenixDrone
═══════════════════════════════════════════════════════════════
Generates a lawnmower (boustrophedon) survey pattern along the road
and drives the drone through it by publishing goal poses to the
flight controller, which handles the actual PID stabilisation.

The drone flies at a fixed altitude along the road, capturing camera
images for pothole detection while the LiDAR provides obstacle
awareness.

Usage:
    ros2 run phoenix_drone_control road_survey
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


def yaw_to_quaternion(yaw: float):
    """Return (x, y, z, w) for a rotation of `yaw` about Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class RoadSurvey(Node):
    """
    Autonomous road survey controller.

    Generates a systematic survey pattern along the pothole road and
    sequences the drone through it via /phoenix/goal_pose, advancing
    when ground-truth odometry shows the waypoint has been reached.
    """

    def __init__(self):
        super().__init__('road_survey')

        # ── Parameters ──
        self.declare_parameter('survey_altitude', 6.0)       # meters AGL
        self.declare_parameter('road_start_x', 5.0)          # meters (ENU)
        self.declare_parameter('road_end_x', 95.0)
        self.declare_parameter('road_width', 6.0)
        self.declare_parameter('lane_spacing', 3.0)
        self.declare_parameter('waypoint_spacing', 10.0)
        self.declare_parameter('arrival_radius', 1.5)        # meters
        self.declare_parameter('update_rate_hz', 5.0)
        self.declare_parameter('start_delay_s', 12.0)        # let takeoff finish
        self.declare_parameter('auto_start', True)

        alt = self.get_parameter('survey_altitude').value
        road_start = self.get_parameter('road_start_x').value
        road_end = self.get_parameter('road_end_x').value
        road_width = self.get_parameter('road_width').value
        lane_spacing = self.get_parameter('lane_spacing').value
        wp_spacing = self.get_parameter('waypoint_spacing').value
        rate_hz = self.get_parameter('update_rate_hz').value

        self._arrival_radius = self.get_parameter('arrival_radius').value
        self._start_delay = self.get_parameter('start_delay_s').value
        self._auto_start = self.get_parameter('auto_start').value
        self._altitude = alt

        # ── Generate lawnmower survey waypoints ──
        self._waypoints = self._generate_survey_pattern(
            road_start, road_end, road_width, lane_spacing, wp_spacing, alt
        )
        self._current_wp_idx = 0
        self._survey_active = False
        self._survey_complete = False
        self._elapsed = 0.0

        # ── Cached state ──
        self._position = None
        self._flight_state = 'UNKNOWN'

        # ── Publishers ──
        self._goal_pub = self.create_publisher(
            PoseStamped, '/phoenix/goal_pose', 10)
        self._status_pub = self.create_publisher(
            String, '/phoenix/survey/status', 10)
        self._path_pub = self.create_publisher(
            Path, '/phoenix/survey/path', 10)
        self._land_pub = self.create_publisher(
            String, '/phoenix/flight_command', 10)

        # ── Subscribers ──
        self.create_subscription(
            Odometry, '/phoenix/ground_truth/odom', self._odom_cb, 10)
        self.create_subscription(
            String, '/phoenix/flight_state', self._state_cb, 10)
        self.create_subscription(
            String, '/phoenix/survey/command', self._command_cb, 10)

        # ── Timers ──
        self._dt = 1.0 / rate_hz
        self._timer = self.create_timer(self._dt, self._survey_loop)
        # Publish the planned path periodically so RViz late-joiners see it
        self.create_timer(2.0, self._publish_path)

        self.get_logger().info(
            f'Road Survey initialized with {len(self._waypoints)} waypoints\n'
            f'  Altitude:     {alt:.1f}m\n'
            f'  Road range:   [{road_start:.0f}, {road_end:.0f}]m\n'
            f'  Lane spacing: {lane_spacing:.1f}m\n'
            f'  Auto-start:   {self._auto_start} (delay {self._start_delay:.0f}s)'
        )

    # ═══════════════════════════════════════════════════════════
    #  SURVEY PATTERN GENERATION
    # ═══════════════════════════════════════════════════════════

    def _generate_survey_pattern(
        self,
        x_start: float,
        x_end: float,
        road_width: float,
        lane_spacing: float,
        wp_spacing: float,
        altitude: float,
    ) -> list:
        """
        Generate a lawnmower (boustrophedon) survey pattern.

        The drone flies back and forth along the road, shifting
        laterally by lane_spacing each pass to cover the full width.

        Returns a list of waypoint dicts with x, y, z, yaw (ENU frame).
        """
        waypoints = []
        half_width = road_width / 2.0
        num_lanes = max(1, int(round(road_width / lane_spacing)) + 1)

        for lane_idx in range(num_lanes):
            y = min(-half_width + lane_idx * lane_spacing, half_width)

            # Alternate direction each lane (boustrophedon)
            if lane_idx % 2 == 0:
                xs = self._arange(x_start, x_end, wp_spacing)
                yaw = 0.0            # facing +X
            else:
                xs = self._arange(x_end, x_start, -wp_spacing)
                yaw = math.pi        # facing -X

            for x in xs:
                waypoints.append({'x': x, 'y': y, 'z': altitude, 'yaw': yaw})

        return waypoints

    @staticmethod
    def _arange(start: float, stop: float, step: float) -> list:
        """Generate a range of floats (inclusive of endpoints)."""
        result = []
        val = start
        if step > 0:
            while val <= stop + 1e-6:
                result.append(val)
                val += step
        else:
            while val >= stop - 1e-6:
                result.append(val)
                val += step
        # Guarantee the lane end is actually visited
        if result and abs(result[-1] - stop) > 1e-6:
            result.append(stop)
        return result

    # ═══════════════════════════════════════════════════════════
    #  CALLBACKS
    # ═══════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._position = (p.x, p.y, p.z)

    def _state_cb(self, msg: String):
        self._flight_state = msg.data

    def _command_cb(self, msg: String):
        """Manual control: START / STOP / RESTART."""
        cmd = msg.data.strip().upper()
        if cmd == 'START':
            self._survey_active = True
            self._survey_complete = False
            self.get_logger().info('Survey START commanded')
        elif cmd == 'STOP':
            self._survey_active = False
            self.get_logger().info('Survey STOP commanded')
        elif cmd == 'RESTART':
            self._current_wp_idx = 0
            self._survey_active = True
            self._survey_complete = False
            self.get_logger().info('Survey RESTART commanded')

    # ═══════════════════════════════════════════════════════════
    #  PUBLISHING HELPERS
    # ═══════════════════════════════════════════════════════════

    def _publish_goal(self, wp):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(wp['x'])
        msg.pose.position.y = float(wp['y'])
        msg.pose.position.z = float(wp['z'])
        qx, qy, qz, qw = yaw_to_quaternion(wp['yaw'])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._goal_pub.publish(msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_path(self):
        """Publish the full survey pattern for RViz visualisation."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for wp in self._waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(wp['x'])
            ps.pose.position.y = float(wp['y'])
            ps.pose.position.z = float(wp['z'])
            qx, qy, qz, qw = yaw_to_quaternion(wp['yaw'])
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self._path_pub.publish(path)

    # ═══════════════════════════════════════════════════════════
    #  MAIN SURVEY LOOP
    # ═══════════════════════════════════════════════════════════

    def _survey_loop(self):
        """Main sequencing loop for the road survey mission."""
        self._elapsed += self._dt

        if self._survey_complete:
            return

        # ── Wait for takeoff to finish before starting ──
        if not self._survey_active:
            if not self._auto_start:
                return
            if self._elapsed < self._start_delay:
                self._publish_status(
                    f'WAITING — takeoff ({self._elapsed:.0f}/'
                    f'{self._start_delay:.0f}s)')
                return
            if self._position is None:
                self._publish_status('WAITING — no odometry')
                return
            self._survey_active = True
            self.get_logger().info(
                f'▶ Starting survey: {len(self._waypoints)} waypoints')
            self._publish_status('STARTED')

        if self._position is None or not self._waypoints:
            return

        # ── Survey finished? ──
        if self._current_wp_idx >= len(self._waypoints):
            self._survey_complete = True
            self._survey_active = False
            self._publish_status('COMPLETE — Landing')
            self.get_logger().info('✓ Road survey complete — landing')
            cmd = String()
            cmd.data = 'LAND'
            self._land_pub.publish(cmd)
            return

        # ── Drive toward the current waypoint ──
        wp = self._waypoints[self._current_wp_idx]
        self._publish_goal(wp)

        dx = self._position[0] - wp['x']
        dy = self._position[1] - wp['y']
        dz = self._position[2] - wp['z']
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < self._arrival_radius:
            total = len(self._waypoints)
            progress = (self._current_wp_idx + 1) / total * 100.0
            self._publish_status(
                f'WP {self._current_wp_idx + 1}/{total} ({progress:.0f}%)')
            self.get_logger().info(
                f'  ✓ WP {self._current_wp_idx + 1}/{total} '
                f'[{wp["x"]:.1f}, {wp["y"]:.1f}] ({progress:.0f}%)')
            self._current_wp_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = RoadSurvey()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Road survey interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
