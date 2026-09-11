#!/usr/bin/env python3
"""
Phoenix Drone Flight Controller — cascade PID control for Gazebo Classic.
Uses Gazebo's apply_link_wrench service to control the drone via forces/torques.
Subscribes to /goal_pose from RViz for waypoint navigation.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from gazebo_msgs.srv import ApplyLinkWrench, LinkRequest
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
import numpy as np


def euler_from_quaternion(q):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class FlightController(Node):
    """Simple PID-based flight controller for a quadrotor in Gazebo Classic."""

    # States
    IDLE = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    HOVER = 'HOVER'
    NAVIGATE = 'NAVIGATE'
    LAND = 'LAND'

    def __init__(self):
        super().__init__('flight_controller')

        # Parameters
        self.declare_parameter('takeoff_altitude', 5.0)
        self.declare_parameter('mass', 1.5)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('model_name', 'phoenix_drone')
        self.declare_parameter('body_name', 'base_link')
        self.declare_parameter('auto_takeoff', True)
        self.declare_parameter('max_altitude', 40.0)
        self.declare_parameter('max_speed', 14.0)
        self.declare_parameter('cruise_speed', 6.0)
        self.declare_parameter('wrench_refresh_hz', 20.0)

        self.takeoff_alt = self.get_parameter('takeoff_altitude').value
        self.mass = self.get_parameter('mass').value
        self.rate = self.get_parameter('control_rate').value
        self.model_name = self.get_parameter('model_name').value
        self.body_name = self.get_parameter('body_name').value
        self.auto_takeoff = self.get_parameter('auto_takeoff').value
        self.max_altitude = self.get_parameter('max_altitude').value
        self.max_speed = self.get_parameter('max_speed').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.wrench_refresh_hz = self.get_parameter('wrench_refresh_hz').value

        self.gravity = 9.81
        self.hover_thrust = self.mass * self.gravity

        # State
        self.state = self.IDLE
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.target_pos = np.array([0.0, 0.0, self.takeoff_alt])
        self.target_yaw = 0.0
        self.odom_received = False
        self.startup_timer = 0
        self.takeoff_x = 0.0
        self.takeoff_y = 0.0
        self._envelope_tripped = False
        self._bad_odom = 0
        self.last_odom_time = None
        self.odom_timeout_s = 2.0

        # PID gains
        # Position
        # Cascade XY: position -> velocity (kp_xy), velocity -> force (kv_xy)
        self.kp_xy = 0.8
        self.kv_xy = 3.0
        self.ki_xy = 0.05
        self.kp_z = 5.0
        self.kd_z = 3.0
        self.ki_z = 0.15
        # Yaw
        self.kp_yaw = 2.0
        self.kd_yaw = 0.5
        # Integral accumulators
        self.int_xy = np.array([0.0, 0.0])
        self.int_z = 0.0
        # Max values
        self.max_force_xy = 6.0
        self.max_force_z = 32.0
        self.max_torque = 3.0
        # Keep the integral small: it only trims the steady-state droop.
        # A large one winds up during a climb and makes the loop unstable.
        self.max_integral = 2.0

        # QoS
        qos = QoSProfile(depth=10,
                          reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/phoenix/ground_truth/odom', self.odom_callback, qos)
        # RViz "2D Goal Pose" publishes here
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        # The survey node publishes full 3D goals here
        self.survey_goal_sub = self.create_subscription(
            PoseStamped, '/phoenix/goal_pose', self.goal_callback, 10)
        # TAKEOFF / LAND / HOVER commands
        self.cmd_sub = self.create_subscription(
            String, '/phoenix/flight_command', self.command_callback, 10)

        # Publishers
        self.state_pub = self.create_publisher(String, '/phoenix/flight_state', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/phoenix/target_pose', 10)

        # ── Gazebo service clients ──
        # Gazebo Classic drops wrenches whose duration is shorter than a
        # physics step, so we apply a CONTINUOUS wrench (duration = -1) and
        # clear + re-apply it every control cycle.  Clearing must use
        # LinkRequest; BodyRequest silently fails to cancel the force.
        self.wrench_client = self.create_client(
            ApplyLinkWrench, '/apply_link_wrench')
        self.clear_client = self.create_client(
            LinkRequest, '/clear_link_wrenches')
        self._link_name = f'{self.model_name}::{self.body_name}'
        self._warned_no_service = False
        self._refreshing = False
        self._last_refresh = None
        self._min_refresh_dt = (1.0 / self.wrench_refresh_hz
                                if self.wrench_refresh_hz > 0 else 0.0)

        # Control loop timer
        dt = 1.0 / self.rate
        self.control_timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info('Flight Controller started — waiting for odometry...')

    def odom_callback(self, msg: Odometry):
        """Update current position and velocity from ground truth."""
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        pos = np.array([p.x, p.y, p.z])
        vel = np.array([v.x, v.y, v.z])

        # A physics blow-up shows up here as NaN/inf.  Never feed that into
        # the PID — it would poison the integrators permanently.
        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(vel))):
            self._bad_odom += 1
            self.get_logger().error(
                f'Non-finite odometry (sample {self._bad_odom}) — holding '
                f'position, waiting for the simulator to recover',
                throttle_duration_sec=5.0)
            self._stop_wrench()
            # Drop the sample.  If they keep coming the staleness failsafe
            # takes over; a single glitch is simply ignored.
            return

        if self._bad_odom:
            self.get_logger().info(
                f'Odometry recovered after {self._bad_odom} bad sample(s)')
            self._bad_odom = 0

        self.current_pos = pos
        self.current_vel = vel
        _, _, self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True
        self.last_odom_time = self.get_clock().now()

    def goal_callback(self, msg: PoseStamped):
        """
        Receive a goal pose (RViz 2D Goal Pose, or the survey node).

        The survey republishes its current waypoint continuously, so this
        must be idempotent: re-sending the same goal must not reset the
        integral terms or bounce the state machine, or the controller
        oscillates and goes unstable.
        """
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gz = msg.pose.position.z
        _, _, gyaw = euler_from_quaternion(msg.pose.orientation)

        # RViz 2D Goal Pose sends z=0; in that case hold the cruise altitude.
        if gz <= 0.05:
            gz = self.takeoff_alt

        new_target = np.array([gx, gy, gz])

        # Ignore repeats of the goal we are already tracking.
        if np.allclose(new_target, self.target_pos, atol=1e-3) and \
                abs(gyaw - self.target_yaw) < 1e-3:
            return

        self.target_pos = new_target
        self.target_yaw = gyaw
        # Only reset the XY integral on a genuinely new goal, and never the
        # Z integral — that one is holding the drone up.
        self.int_xy = np.array([0.0, 0.0])

        if self.state in (self.HOVER, self.NAVIGATE, self.TAKEOFF):
            # Already airborne (or climbing): just retarget.  Stay in TAKEOFF
            # until the altitude is actually reached.
            if self.state != self.TAKEOFF:
                self.state = self.NAVIGATE
            self.get_logger().info(
                f'Goal: ({gx:.1f}, {gy:.1f}, {gz:.1f}) '
                f'yaw={math.degrees(gyaw):.0f}°',
                throttle_duration_sec=2.0)
        elif self.state == self.IDLE:
            self.state = self.TAKEOFF
            self.get_logger().info('Goal received — taking off first')

    def command_callback(self, msg: String):
        """Handle TAKEOFF / LAND / HOVER / RTL commands."""
        cmd = msg.data.strip().upper()

        if cmd == 'LAND':
            self.state = self.LAND
            self.target_pos = np.array([
                self.current_pos[0], self.current_pos[1], 0.0])
            self.int_xy = np.array([0.0, 0.0])
            self.int_z = 0.0
            self.get_logger().info('LAND commanded')

        elif cmd == 'TAKEOFF':
            self._envelope_tripped = False
            self.takeoff_x = self.current_pos[0]
            self.takeoff_y = self.current_pos[1]
            self.state = self.TAKEOFF
            self.target_pos = np.array([
                self.current_pos[0], self.current_pos[1], self.takeoff_alt])
            self.get_logger().info(
                f'TAKEOFF commanded to {self.takeoff_alt:.1f}m')

        elif cmd == 'HOVER':
            self.state = self.HOVER
            self.target_pos = np.array([
                self.current_pos[0], self.current_pos[1], self.takeoff_alt])
            self.get_logger().info('HOVER commanded')

        elif cmd == 'RTL':
            self.state = self.NAVIGATE
            self.target_pos = np.array([0.0, 0.0, self.takeoff_alt])
            self.int_xy = np.array([0.0, 0.0])
            self.get_logger().info('RTL commanded — returning to launch')

        else:
            self.get_logger().warn(f'Unknown flight command: "{msg.data}"')

    def _odom_is_stale(self) -> bool:
        """True if ground-truth odometry has gone silent."""
        if self.last_odom_time is None:
            return True
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        return age > self.odom_timeout_s

    def control_loop(self):
        """Main control loop running at control_rate Hz."""
        # Nothing to do until the drone has actually spawned and is
        # publishing odometry.  Sending wrenches before that just spams
        # Gazebo with "link does not exist" errors.
        if not self.odom_received:
            self.startup_timer += 1
            if self.startup_timer % int(self.rate * 5) == 0:
                self.get_logger().info(
                    'Waiting for /phoenix/ground_truth/odom — '
                    'is the drone spawned?')
            return

        # Auto-takeoff on first odom
        if self.state == self.IDLE and self.auto_takeoff:
            self.state = self.TAKEOFF
            self.takeoff_x = self.current_pos[0]
            self.takeoff_y = self.current_pos[1]
            self.target_pos = np.array([
                self.takeoff_x, self.takeoff_y, self.takeoff_alt])
            self.get_logger().info(
                f'Auto-takeoff to {self.takeoff_alt}m altitude')

        # ── Failsafe: if odometry goes stale mid-flight, stop commanding ──
        if self._odom_is_stale() and self.state != self.IDLE:
            self.get_logger().warn(
                'Odometry stale — holding last thrust, no new commands',
                throttle_duration_sec=2.0)
            return

        # ── Runaway protection ──────────────────────────────────────
        # Gazebo's continuous wrenches can be dropped or double-applied
        # under load; if the drone ever leaves the sane flight envelope,
        # cut thrust entirely rather than let it accelerate away.
        speed = float(np.linalg.norm(self.current_vel))
        if (self.current_pos[2] > self.max_altitude
                or speed > self.max_speed
                or not np.all(np.isfinite(self.current_pos))):
            self.get_logger().error(
                f'Flight envelope exceeded (alt={self.current_pos[2]:.1f}m, '
                f'speed={speed:.1f}m/s) — cutting thrust',
                throttle_duration_sec=2.0)
            self._stop_wrench()
            self.state = self.IDLE
            self.int_xy = np.array([0.0, 0.0])
            self.int_z = 0.0
            # Stay in IDLE (thrust cut) until the body has slowed down,
            # then resume from wherever it ended up rather than ending the
            # mission outright.
            self._envelope_tripped = True
            return

        # Recovered from an envelope trip: resume once we're slow again.
        if self._envelope_tripped and speed < 1.0:
            self._envelope_tripped = False
            self.takeoff_x = self.current_pos[0]
            self.takeoff_y = self.current_pos[1]
            self.state = self.TAKEOFF
            self.get_logger().info(
                'Back inside the flight envelope — resuming')

        # Publish state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # Publish the active target so RViz/monitors can show it
        tp = PoseStamped()
        tp.header.stamp = self.get_clock().now().to_msg()
        tp.header.frame_id = 'map'
        tp.pose.position.x = float(self.target_pos[0])
        tp.pose.position.y = float(self.target_pos[1])
        tp.pose.position.z = float(self.target_pos[2])
        tp.pose.orientation.z = math.sin(self.target_yaw / 2.0)
        tp.pose.orientation.w = math.cos(self.target_yaw / 2.0)
        self.pose_pub.publish(tp)

        # Compute control based on state
        if self.state == self.IDLE:
            self._stop_wrench()
            return

        elif self.state == self.TAKEOFF:
            # Climb straight up from where we are; only chase XY once high.
            climb_target = np.array([
                self.takeoff_x, self.takeoff_y, self.takeoff_alt])
            force, torque = self._compute_pid(climb_target)
            self._apply_wrench(force, torque)

            if abs(self.current_pos[2] - self.takeoff_alt) < 0.4:
                self.state = self.HOVER
                self.get_logger().info(
                    f'Reached {self.current_pos[2]:.1f}m — HOVER')

        elif self.state == self.HOVER:
            force, torque = self._compute_pid()
            self._apply_wrench(force, torque)

        elif self.state == self.NAVIGATE:
            force, torque = self._compute_pid()
            self._apply_wrench(force, torque)

            dist = np.linalg.norm(self.target_pos[:2] - self.current_pos[:2])
            if dist < 0.5:
                self.state = self.HOVER
                self.get_logger().info('Reached goal — HOVER')

        elif self.state == self.LAND:
            self.target_pos[2] = 0.0
            force, torque = self._compute_pid()
            # Descend gently: cap thrust just below hover so it settles
            force[2] = min(force[2], self.hover_thrust * 0.92)
            self._apply_wrench(force, torque)

            if self.current_pos[2] < 0.3:
                self.state = self.IDLE
                self.int_xy = np.array([0.0, 0.0])
                self.int_z = 0.0
                self._stop_wrench()
                self.get_logger().info('Landed — IDLE')

    def _compute_pid(self, target=None):
        """PID controller for position and yaw."""
        dt = 1.0 / self.rate

        # Position error
        if target is None:
            target = self.target_pos
        err = target - self.current_pos

        # XY control — integrate only near the target (anti-windup)
        if np.linalg.norm(err[:2]) < 2.0:
            self.int_xy += err[:2] * dt
            self.int_xy = np.clip(self.int_xy,
                                  -self.max_integral, self.max_integral)

        # Position error -> desired velocity, capped at cruise speed.
        dist_xy = float(np.linalg.norm(err[:2]))
        if dist_xy > 1e-6:
            speed_cmd = min(self.kp_xy * dist_xy, self.cruise_speed)
            vel_des = err[:2] / dist_xy * speed_cmd
        else:
            vel_des = np.zeros(2)

        # Velocity error -> force.  Because the commanded speed is capped,
        # the drone decelerates on approach instead of overshooting.
        vel_err = vel_des - self.current_vel[:2]
        fx = self.kv_xy * vel_err[0] + self.ki_xy * self.int_xy[0]
        fy = self.kv_xy * vel_err[1] + self.ki_xy * self.int_xy[1]
        fx = np.clip(fx, -self.max_force_xy, self.max_force_xy)
        fy = np.clip(fy, -self.max_force_xy, self.max_force_xy)

        # Z control (includes gravity compensation).
        # Integrate only when close to the target and unsaturated, so the
        # term trims hover droop instead of winding up during a climb.
        fz_raw = (self.hover_thrust
                  + self.kp_z * err[2]
                  - self.kd_z * self.current_vel[2]
                  + self.ki_z * self.int_z)
        if abs(err[2]) < 1.0 and 0.0 < fz_raw < self.max_force_z:
            self.int_z += err[2] * dt
            self.int_z = np.clip(self.int_z,
                                 -self.max_integral, self.max_integral)

        fz = (self.hover_thrust
              + self.kp_z * err[2]
              - self.kd_z * self.current_vel[2]
              + self.ki_z * self.int_z)
        fz = np.clip(fz, 0.0, self.max_force_z)

        # Yaw control
        yaw_err = self.target_yaw - self.current_yaw
        # Wrap to [-pi, pi]
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
        tz = self.kp_yaw * yaw_err
        tz = np.clip(tz, -self.max_torque, self.max_torque)

        force = np.array([fx, fy, fz])
        torque = np.array([0.0, 0.0, tz])
        return force, torque

    def _apply_wrench(self, force, torque):
        """
        Apply a wrench to the drone body.

        Gazebo Classic drops wrenches whose duration is shorter than one
        physics step, so short-duration requests are unreliable.  We instead
        hold a CONTINUOUS wrench (duration = -1) and refresh it.

        The refresh is clear-then-apply and both calls are async, so a new
        pair must never be started while the previous one is still in flight
        — interleaved pairs leave the drone with no force or a stale one.
        Gazebo replaces (does not accumulate) a continuous wrench, so
        refreshing every cycle is correct as long as the pairs are
        serialised.
        """
        if not (self.wrench_client.service_is_ready()
                and self.clear_client.service_is_ready()):
            if not self._warned_no_service:
                self._warned_no_service = True
                self.get_logger().warn(
                    'Gazebo wrench services not available yet — is gzserver '
                    'running with libgazebo_ros_force_system.so?')
            return

        # A refresh is still in progress: leave the current force in place
        # rather than interleaving a second clear/apply pair.
        if self._refreshing:
            return

        # Rate-limit the refresh.  The wrench is continuous, so it stays
        # applied between refreshes; re-issuing it every control cycle just
        # churns Gazebo's wrench list and destabilises the solver.
        now = self.get_clock().now()
        if self._last_refresh is not None:
            age = (now - self._last_refresh).nanoseconds / 1e9
            if age < self._min_refresh_dt:
                return
        self._last_refresh = now

        self._refreshing = True

        clear_req = LinkRequest.Request()
        clear_req.link_name = self._link_name
        clear_future = self.clear_client.call_async(clear_req)

        def _on_cleared(_fut, f=force, t=torque):
            req = ApplyLinkWrench.Request()
            req.link_name = self._link_name
            req.reference_frame = ''       # empty => world/inertial frame
            req.reference_point = Point(x=0.0, y=0.0, z=0.0)
            req.wrench.force = Vector3(x=float(f[0]), y=float(f[1]),
                                       z=float(f[2]))
            req.wrench.torque = Vector3(x=float(t[0]), y=float(t[1]),
                                        z=float(t[2]))
            req.start_time = TimeMsg(sec=0, nanosec=0)
            req.duration = DurationMsg(sec=-1, nanosec=0)   # continuous
            apply_future = self.wrench_client.call_async(req)
            apply_future.add_done_callback(
                lambda _f: setattr(self, '_refreshing', False))

        clear_future.add_done_callback(_on_cleared)

    def _stop_wrench(self):
        """Cancel any continuous wrench (used when idle/landed)."""
        self._last_refresh = None
        if self.clear_client.service_is_ready():
            req = LinkRequest.Request()
            req.link_name = self._link_name
            self.clear_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
