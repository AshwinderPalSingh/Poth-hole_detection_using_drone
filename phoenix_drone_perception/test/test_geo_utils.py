"""
test_geo_utils.py — unit tests for camera→ground projection and geodesy.

Run with:  python3 -m pytest phoenix_drone_perception/test/test_geo_utils.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from phoenix_drone_perception.geo_utils import (   # noqa: E402
    enu_to_geodetic,
    estimate_diameter_m,
    focal_from_fov,
    geodetic_to_enu,
    ground_footprint,
    haversine_m,
    pixel_to_world,
)

W, H = 640, 480
FOV = 1.3962634                     # 80 deg, as in the drone's SDF
FX = focal_from_fov(W, FOV)
CX, CY = W / 2.0, H / 2.0
ALT = 6.0


def test_focal_length_matches_sdf():
    # Gazebo derives fx from width and horizontal FOV the same way.
    assert abs(FX - 381.36) < 0.1


def test_centre_pixel_is_directly_below():
    wx, wy = pixel_to_world(CX, CY, CX, CY, FX, FX, 10.0, 5.0, ALT, 0.0)
    assert abs(wx - 10.0) < 1e-9
    assert abs(wy - 5.0) < 1e-9


def test_pixel_above_centre_is_ahead():
    """Image +v points backward, so a pixel ABOVE centre is in front."""
    offset_px = 100.0
    wx, wy = pixel_to_world(CX, CY - offset_px, CX, CY, FX, FX,
                            0.0, 0.0, ALT, 0.0)
    expected = offset_px * ALT / FX
    assert abs(wx - expected) < 1e-9
    assert abs(wy) < 1e-9


def test_pixel_below_centre_is_behind():
    offset_px = 100.0
    wx, _ = pixel_to_world(CX, CY + offset_px, CX, CY, FX, FX,
                           0.0, 0.0, ALT, 0.0)
    assert wx < 0


def test_yaw_rotates_into_world_frame():
    """The same pixel must follow the drone's heading."""
    offset_px = 100.0
    expected = offset_px * ALT / FX
    wx, wy = pixel_to_world(CX, CY - offset_px, CX, CY, FX, FX,
                            0.0, 0.0, ALT, math.pi / 2)
    assert abs(wx) < 1e-6
    assert abs(wy - expected) < 1e-9


def test_pixel_right_of_centre_is_to_the_right():
    """Body +Y is left, so a pixel to the image right maps to -Y."""
    offset_px = 100.0
    _, wy = pixel_to_world(CX + offset_px, CY, CX, CY, FX, FX,
                           0.0, 0.0, ALT, 0.0)
    assert wy < 0


def test_matches_measured_simulator_frame():
    """
    Regression test against a frame captured from the running simulator:
    the drone hovered at (8, 1, 5.96) and pothole_1 at (10, 1) appeared
    at pixel (320, 110).
    """
    wx, wy = pixel_to_world(320.0, 110.0, 320.5, 240.5, 381.4, 381.4,
                            8.0, 1.0, 5.94, 0.0)
    assert abs(wx - 10.0) < 0.25
    assert abs(wy - 1.0) < 0.10


def test_geodetic_round_trip():
    lat, lon, alt = enu_to_geodetic(150.0, -230.0, 12.0,
                                    28.6139, 77.2090, 216.0)
    e, nth, up = geodetic_to_enu(lat, lon, alt, 28.6139, 77.2090, 216.0)
    assert abs(e - 150.0) < 1e-3
    assert abs(nth + 230.0) < 1e-3
    assert abs(up - 12.0) < 1e-6


def test_haversine_agrees_with_local_distance():
    lat, lon, _ = enu_to_geodetic(150.0, -230.0, 0.0,
                                  28.6139, 77.2090, 0.0)
    d = haversine_m(28.6139, 77.2090, lat, lon)
    assert abs(d - math.hypot(150.0, 230.0)) < 0.5


def test_ground_footprint_covers_the_road():
    w_m, h_m = ground_footprint(W, H, FX, FX, ALT)
    assert w_m > 9.0          # the road is 10 m wide
    assert h_m > 7.0


def test_diameter_estimate():
    true_d = 0.8
    px = true_d * FX / ALT
    est = estimate_diameter_m(
        (CX - px / 2, CY - px / 2, CX + px / 2, CY + px / 2), FX, FX, ALT)
    assert abs(est - true_d) < 1e-6
