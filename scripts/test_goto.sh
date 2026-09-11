#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════
#  test_goto.sh — verify point-to-point goal navigation
# ══════════════════════════════════════════════════════════════════
#  Launches the sim with the survey DISABLED, then sends a series of
#  goal poses (exactly as RViz's "2D Goal Pose" tool does) and checks
#  the drone actually reaches each one.
#
#  Usage:  bash scripts/test_goto.sh
# ------------------------------------------------------------------
set +u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/phoenix_goto"
mkdir -p "$OUT"

cleanup() {
  for p in gzserver gzclient flight_controller road_survey pothole_detector \
           pothole_mapper navigation_monitor odom_tf_publisher \
           robot_state_publisher static_transform_publisher spawn_entity; do
    pkill -9 -f "$p" >/dev/null 2>&1
  done
  sleep 1
}
trap cleanup EXIT
cleanup
sleep 3

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export DISPLAY="${DISPLAY:-:0}"

echo "▶ Launching simulation (survey off, manual goals only)…"
ros2 launch phoenix_drone_bringup full_simulation.launch.py \
  gui:=false survey:=false perception:=false > "$OUT/launch.log" 2>&1 &

sleep 25
if ! pgrep -x gzserver >/dev/null; then
  echo "✗ gzserver failed to start"; sed 's/\x1b\[[0-9;]*m//g' "$OUT/launch.log" | tail -20; exit 1
fi
echo "✓ simulation up"

python3 - <<'PY'
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# (x, y, z, yaw_deg, description) — z=0 means "hold cruise altitude",
# which is exactly what RViz's 2D Goal Pose sends.
GOALS = [
    (20.0,   0.0, 0.0,    0, 'forward along the road'),
    (20.0,   3.0, 0.0,   90, 'sidestep left, face +Y'),
    (35.0,  -3.0, 0.0,    0, 'diagonal across the road'),
    (35.0,  -3.0, 9.0,    0, 'climb to 9 m in place'),
    (15.0,   0.0, 4.0,  180, 'come back, descend, face -X'),
]
ARRIVE_XY   = 0.5     # metres — must actually settle on the goal
ARRIVE_Z    = 0.4
SETTLE_S    = 2.0     # and hold it this long, so we measure steady state
TIMEOUT     = 60.0    # seconds per goal

rclpy.init()
n = rclpy.create_node('goto_test')
pub = n.create_publisher(PoseStamped, '/goal_pose', 10)
cmd = n.create_publisher(String, '/phoenix/flight_command', 10)

S = {'p': None, 'v': (0, 0, 0), 'state': ''}
def odom(m):
    q = m.pose.pose.position
    t = m.twist.twist.linear
    S['p'] = (q.x, q.y, q.z)
    S['v'] = (t.x, t.y, t.z)
n.create_subscription(Odometry, '/phoenix/ground_truth/odom', odom, 10)
n.create_subscription(String, '/phoenix/flight_state',
                      lambda m: S.__setitem__('state', m.data), 10)

def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.02)

def send(x, y, z, yaw_deg):
    m = PoseStamped()
    m.header.stamp = n.get_clock().now().to_msg()
    m.header.frame_id = 'map'
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
    yaw = math.radians(yaw_deg)
    m.pose.orientation.z = math.sin(yaw / 2)
    m.pose.orientation.w = math.cos(yaw / 2)
    pub.publish(m)

# Wait for takeoff to finish
print('waiting for takeoff…')
t0 = time.time()
while time.time() - t0 < 45:
    spin(0.2)
    if S['p'] and S['p'][2] > 5.0 and S['state'] in ('HOVER', 'NAVIGATE'):
        break
if not S['p']:
    print('✗ no odometry'); sys.exit(1)
print(f'✓ airborne at {S["p"][2]:.2f} m (state {S["state"]})\n')

CRUISE = 6.0
results = []
for gx, gy, gz, gyaw, label in GOALS:
    want_z = CRUISE if gz <= 0.05 else gz
    print(f'→ goal ({gx:.0f}, {gy:.0f}, {want_z:.0f}) yaw={gyaw:3d}°  [{label}]')
    t0 = time.time()
    reached = False
    last_send = 0.0
    inside_since = None
    while time.time() - t0 < TIMEOUT:
        # RViz sends once; resend slowly to prove idempotence holds.
        if time.time() - last_send > 2.0:
            send(gx, gy, gz, gyaw); last_send = time.time()
        spin(0.05)
        px, py, pz = S['p']
        dxy = math.hypot(px - gx, py - gy)
        dz = abs(pz - want_z)
        speed = math.sqrt(sum(c * c for c in S['v']))
        if dxy < ARRIVE_XY and dz < ARRIVE_Z and speed < 0.5:
            if inside_since is None:
                inside_since = time.time()
            elif time.time() - inside_since >= SETTLE_S:
                reached = True
                break
        else:
            inside_since = None
    px, py, pz = S['p']
    dxy = math.hypot(px - gx, py - gy)
    dz = abs(pz - want_z)
    el = time.time() - t0
    status = 'REACHED' if reached else 'TIMEOUT'
    print(f'   {status:8s} at ({px:6.2f},{py:6.2f},{pz:5.2f})  '
          f'err_xy={dxy:.2f} m err_z={dz:.2f} m  in {el:.0f}s\n')
    results.append((label, reached, dxy, dz, el))
    spin(1.5)

# Return-to-launch, then land, to exercise those commands too.
print('→ RTL then LAND')
cmd.publish(String(data='RTL'))
t0 = time.time()
while time.time() - t0 < 60:
    spin(0.05)
    px, py, _ = S['p']
    if math.hypot(px, py) < 1.0:
        break
px, py, pz = S['p']
rtl_ok = math.hypot(px, py) < 1.5
print(f'   RTL {"OK" if rtl_ok else "FAILED"} at ({px:.2f},{py:.2f},{pz:.2f})')

cmd.publish(String(data='LAND'))
t0 = time.time()
while time.time() - t0 < 45:
    spin(0.05)
    if S['p'][2] < 0.4:
        break
land_ok = S['p'][2] < 0.5
print(f'   LAND {"OK" if land_ok else "FAILED"} at z={S["p"][2]:.2f}\n')

print('═' * 70)
print(' GOAL NAVIGATION RESULTS')
print('═' * 70)
for label, ok, dxy, dz, el in results:
    print(f'  [{"PASS" if ok else "FAIL"}] {label:32s} '
          f'err {dxy:.2f} m / {dz:.2f} m  {el:4.0f}s')
print(f'  [{"PASS" if rtl_ok else "FAIL"}] return to launch')
print(f'  [{"PASS" if land_ok else "FAIL"}] land')
total = len(results) + 2
passed = sum(1 for _, ok, _, _, _ in results) + rtl_ok + land_ok
print('═' * 70)
print(f'  {passed}/{total} passed')
print('═' * 70)
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if passed == total else 1)
PY
RC=$?
echo "Logs: $OUT/launch.log"
exit $RC
