#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════
#  smoke_test.sh — end-to-end check of the PhoenixDrone simulation
# ══════════════════════════════════════════════════════════════════
#  Launches the full stack headless, flies the survey for a while,
#  then reports whether the drone flew, the sensors streamed, and the
#  pothole map was produced.  Cleans up every process on exit.
#
#  Usage:  bash scripts/smoke_test.sh [DURATION_SECONDS]
# ------------------------------------------------------------------
# ROS setup scripts reference unset vars, so no "set -u" here.
set +u

DURATION="${1:-150}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/phoenix_smoke"
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

leftovers=$(pgrep -c -f "gzserver|flight_controller|pothole_detector|road_survey" 2>/dev/null || echo 0)
if [ "$leftovers" -gt 0 ]; then
  echo "✗ $leftovers stale process(es) survived cleanup; aborting to avoid"
  echo "  measuring a previous run. Kill them and retry."
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export DISPLAY="${DISPLAY:-:0}"

rm -f /tmp/pothole_map.json /tmp/pothole_report.txt

echo "▶ Launching full simulation (headless)…"
ros2 launch phoenix_drone_bringup full_simulation.launch.py gui:=false \
  > "$OUT/launch.log" 2>&1 &

sleep 25
if ! pgrep -x gzserver >/dev/null; then
  echo "✗ gzserver failed to start. Last lines:"
  sed 's/\x1b\[[0-9;]*m//g' "$OUT/launch.log" | tail -20
  exit 1
fi
echo "✓ gzserver running"

python3 - "$DURATION" <<'PY'
import json, statistics, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, NavSatFix
from std_msgs.msg import String

DURATION = float(sys.argv[1])

rclpy.init()
n = rclpy.create_node('smoke_test')
sq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=1)

S = {'z': 0.0, 'x': 0.0, 'y': 0.0, 'max_z': 0.0, 'state': '', 'survey': '',
     'img': 0, 'scan': 0, 'gps': 0, 'det': 0, 'map': '', 'alts': []}


def odom(m):
    p = m.pose.pose.position
    S['x'], S['y'], S['z'] = p.x, p.y, p.z
    S['max_z'] = max(S['max_z'], p.z)


def det(m):
    try:
        S['det'] += len(json.loads(m.data).get('detections', []))
    except Exception:
        pass


n.create_subscription(Odometry, '/phoenix/ground_truth/odom', odom, 10)
n.create_subscription(Image, '/phoenix/camera/image_raw',
                      lambda m: S.__setitem__('img', S['img'] + 1), sq)
n.create_subscription(LaserScan, '/phoenix/lidar/scan',
                      lambda m: S.__setitem__('scan', S['scan'] + 1), sq)
n.create_subscription(NavSatFix, '/phoenix/gps/fix',
                      lambda m: S.__setitem__('gps', S['gps'] + 1), sq)
n.create_subscription(String, '/phoenix/flight_state',
                      lambda m: S.__setitem__('state', m.data), 10)
n.create_subscription(String, '/phoenix/survey/status',
                      lambda m: S.__setitem__('survey', m.data), 10)
n.create_subscription(String, '/phoenix/perception/detections', det, 10)
n.create_subscription(String, '/phoenix/pothole_map',
                      lambda m: S.__setitem__('map', m.data), 10)

t0 = time.time()
nxt = 10.0
while time.time() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.05)
    el = time.time() - t0
    if el >= nxt:
        nxt += 10.0
        if el > 30:
            S['alts'].append(S['z'])
        print(f'  t={el:5.1f}s {S["state"]:9s} pos=({S["x"]:6.1f},{S["y"]:5.1f},'
              f'{S["z"]:6.2f}) img={S["img"]} scan={S["scan"]} gps={S["gps"]} '
              f'det={S["det"]} | {S["survey"][:26]}', flush=True)

print('\n' + '═' * 66)
print(' SMOKE TEST RESULTS')
print('═' * 66)
checks = []


def check(name, ok, detail):
    checks.append(ok)
    print(f'  [{"PASS" if ok else "FAIL"}] {name:28s} {detail}')


check('camera streaming', S['img'] > 50, f'{S["img"]} frames')
check('lidar streaming', S['scan'] > 50, f'{S["scan"]} scans')
check('gps streaming', S['gps'] > 50, f'{S["gps"]} fixes')
check('drone took off', S['max_z'] > 3.0, f'max altitude {S["max_z"]:.2f} m')
check('altitude sane', S['max_z'] < 40.0, f'max altitude {S["max_z"]:.2f} m')
if S['alts']:
    spread = max(S['alts']) - min(S['alts'])
    check('altitude stable', spread < 4.0, f'spread {spread:.2f} m')
check('travelled along road', S['x'] > 15.0, f'x = {S["x"]:.1f} m')
check('survey progressing', 'WP' in S['survey'] or 'COMPLETE' in S['survey'],
      S['survey'] or '(no status)')
check('detections produced', S['det'] > 0, f'{S["det"]} raw detections')

confirmed = 0
if S['map']:
    try:
        d = json.loads(S['map'])
        confirmed = d.get('confirmed_count', 0)
        print(f'\n  Confirmed potholes: {confirmed}')
        for p in d.get('potholes', []):
            print(f'    #{p["id"]:2d} ({p["world_x"]:6.1f},{p["world_y"]:6.1f}) '
                  f'⌀{p["diameter_m"]:.2f}m {p["severity"]:6s} '
                  f'{p["latitude"]:.6f},{p["longitude"]:.6f}')
    except Exception as e:
        print('  map parse error:', e)
check('pothole map built', confirmed > 0, f'{confirmed} confirmed')

print('═' * 66)
print(f'  {sum(checks)}/{len(checks)} checks passed')
print('═' * 66)
n.destroy_node()
rclpy.shutdown()
sys.exit(0 if all(checks) else 1)
PY
RC=$?

echo
echo "Logs: $OUT/launch.log"
[ -f /tmp/pothole_report.txt ] && { echo; cat /tmp/pothole_report.txt; }
exit $RC
