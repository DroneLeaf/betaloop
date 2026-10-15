"""sim_geodetic — the sim's ONE mapping between local ENU metres (the world,
about the home origin) and WGS84 lat/lon.

It is the exact inverse of what the flight software does with a GPS fix:
HEAR_FC's LocalProjection (HEAR_Util/Navigation/LocalProjection.hpp), i.e.
PX4 ecl's map_projection — an azimuthal equidistant projection on a spherical
earth of radius 6371000 m. A truth ENU point reprojected here and projected
back there is the truth point again (well under a millimetre over +-5 km).

Every sim source of lat/lon must use it so all vehicles share one frame: the
drone's virtual GPS (C++ twin: bf_sim_bridge/local_projection.h — keep the two
identical), the target and launcher GPS (sitl_redis_bridge.py) and the
launchers' Home in the ground-station roster (leaf-sim-ui backend/fleet.py).
Stdlib only.
"""

import math

EARTH_RADIUS_M = 6371000.0   # HEAR_FC / PX4 ecl spherical earth


def enu_to_latlon(origin_lat_deg, origin_lon_deg, east_m, north_m):
    """East / North metres about the origin -> (lat, lon) in degrees."""
    lat0 = math.radians(origin_lat_deg)
    lon0 = math.radians(origin_lon_deg)
    x = north_m / EARTH_RADIUS_M
    y = east_m / EARTH_RADIUS_M
    c = math.hypot(x, y)
    lat, lon = lat0, lon0
    if c > 0.0:
        sin_c, cos_c = math.sin(c), math.cos(c)
        lat = math.asin(cos_c * math.sin(lat0) + x * sin_c * math.cos(lat0) / c)
        lon = lon0 + math.atan2(y * sin_c, c * math.cos(lat0) * cos_c - x * math.sin(lat0) * sin_c)
    return math.degrees(lat), math.degrees(lon)


def latlon_to_enu(origin_lat_deg, origin_lon_deg, lat_deg, lon_deg):
    """(lat, lon) in degrees -> (east, north) metres about the origin."""
    lat0 = math.radians(origin_lat_deg)
    lon0 = math.radians(origin_lon_deg)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    cos_d_lon = math.cos(lon - lon0)
    arg = math.sin(lat0) * math.sin(lat) + math.cos(lat0) * math.cos(lat) * cos_d_lon
    c = math.acos(max(-1.0, min(1.0, arg)))
    k = c / math.sin(c) if c > 0.0 else 1.0
    east = k * math.cos(lat) * math.sin(lon - lon0) * EARTH_RADIUS_M
    north = k * (math.cos(lat0) * math.sin(lat) - math.sin(lat0) * math.cos(lat) * cos_d_lon) * EARTH_RADIUS_M
    return east, north
