"""
geo_utils.py — Camera→ground projection and geodetic conversion
════════════════════════════════════════════════════════════════
Turns a pixel detection from the downward-facing camera into a
world (ENU) coordinate on the road plane, and then into WGS-84
latitude/longitude.

Geometry
--------
The camera looks straight down from the drone.  For a pinhole camera
of focal length f (pixels) at altitude h above the road plane, a pixel
offset (du, dv) from the principal point maps to a ground offset of
(du * h / f, dv * h / f) in the camera's image plane axes.  Those axes
are then rotated by the drone yaw into the world ENU frame.

Image convention: +u is right in the image, +v is DOWN in the image.

The camera link is rotated by +90 deg about Y so its optical axis points at
the ground.  With that mounting, and the drone's nose along body +X, the
image axes map to the body frame as:

    image +v (down in image)  ->  body -X  (BACKWARD)
    image +u (right in image) ->  body -Y  (to the drone's right)

This was verified in the running simulator: with the drone hovering 2 m
short of a pothole at a known world position, the pothole appeared 130 px
ABOVE the principal point, which is -2.03 m along +v -- i.e. +2.03 m
forward.  Both signs are applied in `pixel_to_world`.
"""
import math
from typing import Optional, Tuple

# WGS-84 ellipsoid constants
_WGS84_A = 6378137.0                # semi-major axis (m)
_WGS84_F = 1.0 / 298.257223563      # flattening
_WGS84_E2 = _WGS84_F * (2 - _WGS84_F)


def focal_from_fov(image_width: int, horizontal_fov_rad: float) -> float:
    """Focal length in pixels from image width and horizontal FOV."""
    return (image_width / 2.0) / math.tan(horizontal_fov_rad / 2.0)


def pixel_to_camera_offset(
    u: float,
    v: float,
    cx: float,
    cy: float,
    fx: float,
    fy: float,
    height: float,
) -> Tuple[float, float]:
    """
    Project a pixel to a ground offset in the camera's own axes.

    Returns (v_m, u_m) — metres along the image +v (down) and +u (right)
    directions on the ground plane.  These are IMAGE axes; `pixel_to_world`
    maps them onto the body frame.
    """
    if fx <= 0 or fy <= 0 or height <= 0:
        return 0.0, 0.0
    right = (u - cx) * height / fx
    forward = (v - cy) * height / fy
    return forward, right


def pixel_to_world(
    u: float,
    v: float,
    cx: float,
    cy: float,
    fx: float,
    fy: float,
    drone_x: float,
    drone_y: float,
    height: float,
    yaw: float,
) -> Tuple[float, float]:
    """
    Project a detection pixel onto the ground plane in world ENU metres.

    Parameters
    ----------
    u, v      : pixel coordinates of the detection centre
    cx, cy    : principal point (image centre) in pixels
    fx, fy    : focal lengths in pixels
    drone_x/y : drone position in world ENU metres
    height    : drone altitude above the road plane (metres)
    yaw       : drone heading, radians, 0 = facing world +X

    Returns
    -------
    (world_x, world_y) in metres.
    """
    v_m, u_m = pixel_to_camera_offset(u, v, cx, cy, fx, fy, height)

    # Body frame: +X forward, +Y left.
    # Image +v points BACKWARD along the body, +u points to the right.
    body_x = -v_m
    body_y = -u_m

    # Rotate body → world by yaw
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    world_x = drone_x + body_x * cos_y - body_y * sin_y
    world_y = drone_y + body_x * sin_y + body_y * cos_y
    return world_x, world_y


def enu_to_geodetic(
    east: float,
    north: float,
    up: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float = 0.0,
) -> Tuple[float, float, float]:
    """
    Convert local ENU metres (relative to a reference) to WGS-84 lat/lon/alt.

    Uses the local radii of curvature, which is accurate to well under a
    metre for the few-kilometre offsets a survey produces.
    """
    lat_rad = math.radians(ref_lat_deg)
    sin_lat = math.sin(lat_rad)

    # Meridional and prime-vertical radii of curvature
    denom = math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    r_n = _WGS84_A * (1.0 - _WGS84_E2) / (denom ** 3)   # north-south
    r_e = _WGS84_A / denom                              # east-west

    d_lat = north / r_n
    d_lon = east / (r_e * math.cos(lat_rad))

    return (
        ref_lat_deg + math.degrees(d_lat),
        ref_lon_deg + math.degrees(d_lon),
        ref_alt_m + up,
    )


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float = 0.0,
) -> Tuple[float, float, float]:
    """Inverse of `enu_to_geodetic`."""
    lat_rad = math.radians(ref_lat_deg)
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    r_n = _WGS84_A * (1.0 - _WGS84_E2) / (denom ** 3)
    r_e = _WGS84_A / denom

    north = math.radians(lat_deg - ref_lat_deg) * r_n
    east = math.radians(lon_deg - ref_lon_deg) * r_e * math.cos(lat_rad)
    return east, north, alt_m - ref_alt_m


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two WGS-84 points."""
    r = 6371008.8  # mean Earth radius
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def ground_footprint(
    image_width: int,
    image_height: int,
    fx: float,
    fy: float,
    height: float,
) -> Tuple[float, float]:
    """Ground coverage (width_m, height_m) of the full frame at `height`."""
    if fx <= 0 or fy <= 0:
        return 0.0, 0.0
    return image_width * height / fx, image_height * height / fy


def estimate_diameter_m(
    bbox: Tuple[float, float, float, float],
    fx: float,
    fy: float,
    height: float,
) -> float:
    """Estimate a detection's real-world diameter in metres from its bbox."""
    x1, y1, x2, y2 = bbox
    if fx <= 0 or fy <= 0 or height <= 0:
        return 0.0
    w_m = abs(x2 - x1) * height / fx
    h_m = abs(y2 - y1) * height / fy
    return (w_m + h_m) / 2.0
