"""座標変換の共通関数(画素 → 地上位置、ローカル NED ↔ 緯度経度)。"""
import math

import numpy as np

R_EARTH = 6378137.0   # WGS84 赤道半径 [m]


def quat_to_rot(q):
    """PX4 の四元数 [w, x, y, z](機体 FRD → NED)を回転行列にする。"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def yaw_quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pixel_ray_body(u, v, width, height, hfov, cam_tilt):
    """画素 (u, v) の視線ベクトルを機体座標 FRD(前・右・下)で返す。

    カメラは機首方向を向き、真下から前へ cam_tilt [rad] 起こしてある
    (0 なら真下向き)。光軸 = (sinθ, 0, cosθ)、画像の上 = (cosθ, 0, -sinθ)、
    画像の右 = 機体の右 (0, 1, 0)。
    """
    f = (width / 2.0) / math.tan(hfov / 2.0)
    xr = (u - width / 2.0) / f          # 右方向
    yu = -(v - height / 2.0) / f        # 上方向
    s, c = math.sin(cam_tilt), math.cos(cam_tilt)
    return np.array([s + yu * c, xr, c - yu * s])


def pixel_to_ground(u, v, width, height, hfov, q, pos_ned, cam_tilt=0.0, ground_z=0.0):
    """画像上の点 (u, v) が写している地面の位置 (north, east) を返す。

    画素の視線ベクトルを機体の姿勢(ロール・ピッチ・ヨー)で NED に回し、
    地面(z = ground_z)と交差させる。飛行中の機体の傾きも補正される。
    """
    ray = quat_to_rot(q) @ pixel_ray_body(u, v, width, height, hfov, cam_tilt)
    if ray[2] < 1e-3:          # 地平線より上を向いている
        return None
    t = (ground_z - pos_ned[2]) / ray[2]
    if t <= 0:
        return None
    return float(pos_ned[0] + t * ray[0]), float(pos_ned[1] + t * ray[1])


def ned_to_latlon(north, east, lat0, lon0):
    lat = lat0 + math.degrees(north / R_EARTH)
    lon = lon0 + math.degrees(east / (R_EARTH * math.cos(math.radians(lat0))))
    return lat, lon


def footprint(alt, hfov, width=1280, height=960):
    """高度 alt [m] で真下を撮ったときの撮影範囲 (横幅, 縦幅) [m]。"""
    w = 2.0 * alt * math.tan(hfov / 2.0)
    return w, w * height / width


def look_offset(alt, cam_tilt):
    """画像中心が写す地点までの水平距離 [m](機首方向)。"""
    return alt * math.tan(cam_tilt)
