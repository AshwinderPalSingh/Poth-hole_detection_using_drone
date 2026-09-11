#!/usr/bin/env python3
"""
score_map.py — Score a produced pothole map against the world's ground truth.
════════════════════════════════════════════════════════════════════════════
The world generator writes a manifest of every pothole it placed.  This
compares the map the perception stack produced against that manifest and
reports precision, recall and localisation error.

Usage:
    python3 scripts/score_map.py [map.json] [ground_truth.json]

Defaults to /tmp/pothole_map.json and the installed ground-truth manifest.
"""
import json
import math
import os
import sys

# A detection counts as a hit if it lands within this distance of a real
# pothole.  Potholes are up to ~1.3 m across and the survey flies at 6 m, so
# a couple of metres is a fair tolerance for "found the right pothole".
MATCH_RADIUS_M = 2.5


def load(path):
    with open(path) as f:
        return json.load(f)


def main():
    map_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/pothole_map.json'

    if len(sys.argv) > 2:
        gt_path = sys.argv[2]
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        gt_path = os.path.join(here, '..', 'phoenix_drone_gazebo', 'config',
                               'pothole_ground_truth.json')

    if not os.path.isfile(map_path):
        print(f'No map at {map_path} — run a survey first.')
        return 1
    if not os.path.isfile(gt_path):
        print(f'No ground truth at {gt_path}')
        return 1

    produced = load(map_path)
    truth = load(gt_path)

    detected = produced.get('potholes', [])
    real = truth['potholes']

    # Greedy one-to-one matching, nearest pairs first.
    pairs = []
    for d in detected:
        for g in real:
            dist = math.hypot(g['x'] - d['world_x'], g['y'] - d['world_y'])
            if dist <= MATCH_RADIUS_M:
                pairs.append((dist, d['id'], g['id']))
    pairs.sort()

    used_det, used_gt, matches = set(), set(), []
    for dist, did, gid in pairs:
        if did in used_det or gid in used_gt:
            continue
        used_det.add(did)
        used_gt.add(gid)
        matches.append((dist, did, gid))

    tp = len(matches)
    fp = len(detected) - tp
    fn = len(real) - tp
    precision = tp / len(detected) if detected else 0.0
    recall = tp / len(real) if real else 0.0
    f1 = (2 * precision * recall / (precision + recall)
          if precision + recall else 0.0)

    by_gt = {gid: (dist, did) for dist, did, gid in matches}
    det_by_id = {d['id']: d for d in detected}

    print('═' * 74)
    print(' POTHOLE MAP ACCURACY  (match radius %.1f m)' % MATCH_RADIUS_M)
    print('═' * 74)
    print(f'{"TRUE":>4} {"X":>7} {"Y":>6} {"⌀true":>6} | {"FOUND":>5} '
          f'{"err(m)":>7} {"⌀est":>6} {"SEV":>7}')
    print('─' * 74)
    for g in real:
        if g['id'] in by_gt:
            dist, did = by_gt[g['id']]
            d = det_by_id[did]
            print(f'{g["id"]:>4} {g["x"]:>7.1f} {g["y"]:>6.1f} '
                  f'{g["radius_m"] * 2:>6.2f} | {"#%d" % did:>5} '
                  f'{dist:>7.2f} {d["diameter_m"]:>6.2f} {d["severity"]:>7}')
        else:
            print(f'{g["id"]:>4} {g["x"]:>7.1f} {g["y"]:>6.1f} '
                  f'{g["radius_m"] * 2:>6.2f} | {"MISS":>5}')
    print('─' * 74)

    if matches:
        errs = [d for d, _, _ in matches]
        mean_err = sum(errs) / len(errs)
        print(f'  localisation error: mean {mean_err:.2f} m, '
              f'best {min(errs):.2f} m, worst {max(errs):.2f} m')
        size_err = [abs(det_by_id[did]['diameter_m']
                        - next(g for g in real if g['id'] == gid)['radius_m'] * 2)
                    for _, did, gid in matches]
        print(f'  diameter error:     mean {sum(size_err)/len(size_err):.2f} m')

    print(f'  detected {len(detected)}, real {len(real)}')
    print(f'  true positives {tp}, false positives {fp}, missed {fn}')
    print(f'  precision {precision:.2f}  recall {recall:.2f}  F1 {f1:.2f}')
    print('═' * 74)
    return 0


if __name__ == '__main__':
    sys.exit(main())
