#!/usr/bin/env python3
"""Generate the pothole_road world for Gazebo Classic 11 with true 3D potholes."""
import json, math

# (x, y, radius, depth)  -- road spans x in [0,100], y in [-5,5]
POTHOLES = [
    (10.0,  1.0, 0.40, 0.08),
    (20.0, -2.0, 0.60, 0.12),
    (32.0,  0.5, 0.30, 0.05),
    (41.0, -1.5, 0.50, 0.10),
    (50.0,  2.2, 0.45, 0.09),
    (58.0, -3.0, 0.35, 0.06),
    (67.0,  1.8, 0.55, 0.13),
    (74.0, -0.8, 0.42, 0.07),
    (83.0,  3.0, 0.65, 0.15),
    (92.0, -2.5, 0.38, 0.07),
]

ROAD_Z = 0.02          # top surface of road
SEG = 16               # ring segments per pothole wall

def pothole_model(i, x, y, r, d):
    """A pothole as a recessed bowl: dark floor below road level + ring wall."""
    name = f'pothole_{i}'
    floor_z = ROAD_Z - d
    parts = []
    # Dark floor disc, sunk to depth
    parts.append(f'''      <visual name="floor">
        <pose>0 0 {-d:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{r:.3f}</radius><length>0.004</length></cylinder></geometry>
        <material><ambient>0.03 0.025 0.02 1</ambient><diffuse>0.05 0.04 0.03 1</diffuse><specular>0 0 0 1</specular></material>
      </visual>''')
    # Collision: a real hole the LiDAR/physics can feel (recessed box)
    parts.append(f'''      <collision name="floor_c">
        <pose>0 0 {-d:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{r:.3f}</radius><length>0.004</length></cylinder></geometry>
      </collision>''')
    # Tapered side wall built from thin trapezoid boxes around the rim
    for s in range(SEG):
        a = 2 * math.pi * s / SEG
        wx, wy = r * math.cos(a), r * math.sin(a)
        seg_len = 2 * math.pi * r / SEG * 1.25
        parts.append(f'''      <visual name="wall_{s}">
        <pose>{wx:.4f} {wy:.4f} {-d/2:.4f} 0 0 {a:.4f}</pose>
        <geometry><box><size>0.02 {seg_len:.4f} {d:.4f}</size></box></geometry>
        <material><ambient>0.06 0.05 0.04 1</ambient><diffuse>0.08 0.07 0.05 1</diffuse></material>
      </visual>''')
    # Crumbling darker rim ring just above road level for visual contrast
    parts.append(f'''      <visual name="rim">
        <pose>0 0 0.0025 0 0 0</pose>
        <geometry><cylinder><radius>{r*1.12:.3f}</radius><length>0.003</length></cylinder></geometry>
        <material><ambient>0.10 0.09 0.08 1</ambient><diffuse>0.13 0.12 0.10 1</diffuse></material>
      </visual>''')
    body = '\n'.join(parts)
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {ROAD_Z} 0 0 0</pose>
      <link name="link">
{body}
      </link>
    </model>'''

def lane_dashes():
    """Yellow dashed centerline + solid white edge lines."""
    out = []
    n = 0
    x = 2.0
    while x < 99.0:
        out.append(f'''    <model name="lane_dash_{n}">
      <static>true</static>
      <pose>{x:.1f} 0 {ROAD_Z+0.001:.4f} 0 0 0</pose>
      <link name="link"><visual name="v">
        <geometry><box><size>2.0 0.16 0.002</size></box></geometry>
        <material><ambient>0.75 0.65 0.05 1</ambient><diffuse>0.95 0.82 0.08 1</diffuse></material>
      </visual></link>
    </model>''')
        n += 1
        x += 5.0
    for side, yy in (('l', 4.6), ('r', -4.6)):
        out.append(f'''    <model name="edge_line_{side}">
      <static>true</static>
      <pose>50 {yy} {ROAD_Z+0.001:.4f} 0 0 0</pose>
      <link name="link"><visual name="v">
        <geometry><box><size>100 0.14 0.002</size></box></geometry>
        <material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material>
      </visual></link>
    </model>''')
    return '\n'.join(out)

def barriers():
    out = []
    # Concrete barriers along both shoulders (LiDAR targets)
    for side, yy in (('left', 6.2), ('right', -6.2)):
        out.append(f'''    <model name="barrier_{side}">
      <static>true</static>
      <pose>50 {yy} 0.4 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>100 0.3 0.8</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>100 0.3 0.8</size></box></geometry>
          <material><ambient>0.55 0.55 0.52 1</ambient><diffuse>0.72 0.72 0.68 1</diffuse></material></visual>
      </link>
    </model>''')
    # Bollards as discrete obstacles
    n = 0
    for x in range(8, 100, 12):
        for yy in (5.4, -5.4):
            out.append(f'''    <model name="bollard_{n}">
      <static>true</static>
      <pose>{x} {yy} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>0.09</radius><length>1.0</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>0.09</radius><length>1.0</length></cylinder></geometry>
          <material><ambient>0.8 0.35 0.0 1</ambient><diffuse>0.95 0.45 0.05 1</diffuse></material></visual>
      </link>
    </model>''')
            n += 1
    return '\n'.join(out)

potholes_xml = '\n'.join(pothole_model(i + 1, *p) for i, p in enumerate(POTHOLES))

world = f'''<?xml version="1.0" ?>
<!-- AUTO-GENERATED by scripts/generate_world.py — do not edit by hand -->
<sdf version="1.6">
  <world name="pothole_road">

    <!-- GPS origin: New Delhi, India -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>28.6139</latitude_deg>
      <longitude_deg>77.2090</longitude_deg>
      <elevation>216.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <gravity>0 0 -9.81</gravity>

    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>10</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.6 0.72 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <include><uri>model://sun</uri></include>

    <!-- Dirt ground plane -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
          <material><ambient>0.35 0.30 0.22 1</ambient><diffuse>0.45 0.38 0.28 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Asphalt road surface: 100m x 10m -->
    <model name="road_surface">
      <static>true</static>
      <pose>50 0 0.01 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>100 10 0.02</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>100 10 0.02</size></box></geometry>
          <material>
            <ambient>0.26 0.26 0.27 1</ambient>
            <diffuse>0.34 0.34 0.35 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Lane markings -->
{lane_dashes()}

    <!-- Potholes (recessed, with depth) -->
{potholes_xml}

    <!-- Obstacles -->
{barriers()}

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-8 -12 9 0 0.45 0.75</pose>
      </camera>
    </gui>

  </world>
</sdf>
'''

import sys, os
out = sys.argv[1] if len(sys.argv) > 1 else 'pothole_road.world'
with open(out, 'w') as f:
    f.write(world)

# Ground-truth manifest for scoring detection accuracy
gt = {
    'world': 'pothole_road',
    'road': {'x_min': 0.0, 'x_max': 100.0, 'y_min': -5.0, 'y_max': 5.0, 'surface_z': ROAD_Z},
    'origin_gps': {'latitude': 28.6139, 'longitude': 77.2090, 'elevation': 216.0},
    'potholes': [
        {'id': i + 1, 'x': x, 'y': y, 'radius_m': r, 'depth_m': d}
        for i, (x, y, r, d) in enumerate(POTHOLES)
    ],
}
gt_path = os.path.join(os.path.dirname(out) or '.', 'pothole_ground_truth.json')
with open(gt_path, 'w') as f:
    json.dump(gt, f, indent=2)
print(f'Wrote {out} ({len(world.splitlines())} lines) and {gt_path} ({len(POTHOLES)} potholes)')
