#!/usr/bin/env python3
"""ドローン + 自律フォイルボードによる海難救助のワールド sea_board.sdf と正解データを生成する。

想定: 名護市シーグラスビーチで、遊泳者 3 名が離岸流で沖へ流された。監視員が最後に
見た地点(datum)を通報し、2 分後にドローンが砂浜から出動する。砂浜の水際には
自律航行フォイルボード 3 台が待機していて、ドローンが確定した遭難者のもとへ向かう。

  - 砂浜(北側)と海(南側)。汀線は原点付近。ドローンの離陸地点は砂浜の上の原点
  - 遭難者 3 名(立ち泳ぎの人 2・浮いている人 1)が海流で流れる
    (重力なし・当たり判定なしのモデルに VelocityControl で一定速度を与える)
  - フォイルボード 3 台(eFoil 型:サーフボード形のボード + マスト + モーター + 水中翼)と発光マーカー 3 個。どちらも重力なし・当たり判定なしで、
    動きは board_node が set_pose で与える(マーカーは到着まで水中に隠しておく)
  - 誤検出の要因として、ブイ・浮遊物・係留された漁船・桟橋を置く

座標: Gazebo のワールド座標(ENU: x=東, y=北)[m]。PX4 のローカル座標(NED)とは
north = y, east = x の関係(機体は原点から離陸)。

人のモデルは Gazebo Fuel のものを使うが、Fuel のモデルは当たり判定や静止設定を含み、
そのままでは海流で動かせない。そこで、ローカルにキャッシュされた model.sdf から
見た目(visual)だけを取り出し、漂流用のモデルに組み込む。
先に `gz fuel download` でモデルを取得しておくこと(scripts/setup.sh が行う)。

実行: python3 make_world.py   → 同じフォルダに sea_board.sdf と targets.json
"""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

FUEL_URL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
FUEL_CACHE = Path.home() / '.gz/fuel/fuel.gazebosim.org/openrobotics/models'
WORLD = 'sea_board'
HOME_LAT, HOME_LON = 26.5472, 128.0540   # 名護市 シーグラスビーチ(離陸地点)

# ---- シナリオ ---------------------------------------------------------------
INCIDENT_ELAPSED_S = 120.0               # 事案発生(最後に目撃)から出動までの時間 [s]
CURRENT_SPEED = 0.25                     # 離岸流 + 沿岸流の速さ [m/s](約 0.5 ノット)
CURRENT_TO_DEG = 200.0                   # 海流が流れていく方位 [deg](北=0、時計回り)。沖向き
# 監視員が最後に遭難者を見た地点(datum)。離陸地点から南へ 70 m
DATUM_NORTH, DATUM_EAST = -70.0, 20.0

# 遭難者: (id, モデル, datum からの北, 東 [m], 水面からの高さ z [m], ヨー, 説明)
# 位置は「事案発生時点」のもの。出動時には海流で INCIDENT_ELAPSED_S 秒分だけ流されている
PERSONS = [
    ('P1', 'Casual female', 6.0, -10.0, -1.0, 1.0, '立ち泳ぎの人'),
    ('P2', 'Rescue Randy', -22.0, 18.0, -0.12, 0.5, '浮いている人'),
    ('P3', 'Standing person', -40.0, -30.0, -1.1, -0.6, '立ち泳ぎの人'),
]
CAPSIZED = None

# フォイルボードの待機位置(水際): (id, 北, 東, 向き[deg])
BOARDS = [('B1', -5.0, 12.0, 180.0), ('B2', -5.0, 18.0, 180.0), ('B3', -5.0, 24.0, 180.0)]

# 動かない物(誤検出・遮蔽の要因): (モデル, 東 x, 北 y, ヨー)
STATIC_FUEL = [
    ('Pier', 70.0, -10.0, 0.0),
    ('Vessel C', 62.0, -30.0, 1.4),
    ('mb_round_buoy_orange', -10.0, -45.0, 0.0),
    ('mb_round_buoy_orange', 45.0, -120.0, 0.0),
    ('mb_marker_buoy_red', -40.0, -100.0, 0.0),
    ('mb_marker_buoy_red', 55.0, -75.0, 0.0),
]
# 浮遊物(木材など): (東, 北, 長さ, ヨー)
DEBRIS = [(30.0, -60.0, 3.0, 0.4), (-25.0, -130.0, 2.0, 1.2), (40.0, -150.0, 4.0, 2.0),
          (0.0, -110.0, 1.5, 0.9)]


def current_vector():
    """海流の速度 (vx=東, vy=北) [m/s]。"""
    a = math.radians(CURRENT_TO_DEG)
    return CURRENT_SPEED * math.sin(a), CURRENT_SPEED * math.cos(a)


HEADER = f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- 自動生成: worlds/make_world.py。直接編集しないこと -->
<sdf version="1.9">
  <world name="{WORLD}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <sky><clouds><speed>6</speed></clouds></sky>
      <grid>false</grid>
      <ambient>0.55 0.55 0.55 1.0</ambient>
      <background>0.6 0.7 0.9 1.0</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.3 0.4 -0.85</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{HOME_LAT}</latitude_deg>
      <longitude_deg>{HOME_LON}</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
    <!-- 当たり判定用の平面(海面・砂浜とも z=0)。見た目は下の海面モデルと砂浜 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
    <!-- 海面(Fuel の Coast Water: 6 km 四方、波模様のシェーダー) -->
    <include><uri>{FUEL_URL}Coast%20Water</uri><name>sea_surface</name>
      <pose>0 0 0 0 0 0</pose><static>true</static></include>
    <!-- 砂浜(北側)。南端 y=-3 付近が汀線 -->
    <model name="beach"><static>true</static><pose>0 37 0.05 0 0 0</pose>
      <link name="link"><visual name="sand"><cast_shadows>false</cast_shadows>
        <geometry><box><size>400 80 0.1</size></box></geometry>
        <material><diffuse>0.86 0.79 0.62 1</diffuse><specular>0 0 0 1</specular></material>
      </visual></link></model>
    <model name="helipad"><static>true</static><pose>0 0 0.11 0 0 0</pose>
      <link name="link">
        <visual name="pad"><geometry><box><size>5 5 0.02</size></box></geometry>
          <material><diffuse>0.25 0.25 0.25 1</diffuse></material></visual>
        <visual name="h1"><pose>-1 0 0.012 0 0 0</pose><geometry><box><size>0.4 2.6 0.01</size></box></geometry>
          <material><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
        <visual name="h2"><pose>1 0 0.012 0 0 0</pose><geometry><box><size>0.4 2.6 0.01</size></box></geometry>
          <material><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
        <visual name="h3"><pose>0 0 0.012 0 0 0</pose><geometry><box><size>2.0 0.4 0.01</size></box></geometry>
          <material><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
      </link></model>
"""


# ---- Fuel モデルの見た目だけを取り出す --------------------------------------------
def fuel_visuals(model_name, scale=1.0):
    """キャッシュの model.sdf から <visual> を取り出し、相対パスを Fuel の URL に直して返す。"""
    key = model_name.lower().replace(' ', '%20')
    mdir = FUEL_CACHE / key
    ver = max(int(v.name) for v in mdir.iterdir() if v.name.isdigit())
    base = f'{FUEL_URL}{model_name.replace(" ", "%20")}/{ver}/files/'
    root = ET.parse(mdir / str(ver) / 'model.sdf').getroot()
    out = []
    for i, vis in enumerate(root.iter('visual')):
        vis = ET.fromstring(ET.tostring(vis))
        vis.set('name', f'v{i}')
        for el in vis.iter():
            t = (el.text or '').strip()
            if el.tag in ('uri', 'albedo_map', 'normal_map', 'roughness_map', 'metalness_map',
                          'emissive_map', 'ambient_occlusion_map', 'light_map') and t:
                if t.startswith('model://'):
                    t = t.split('/', 3)[3] if t.count('/') >= 3 else t
                if '://' not in t:
                    el.text = base + t.lstrip('./')
        if scale != 1.0:
            for mesh in vis.iter('mesh'):
                sc = mesh.find('scale')
                if sc is None:
                    sc = ET.SubElement(mesh, 'scale')
                    sc.text = '1 1 1'
                sc.text = ' '.join(str(float(v) * scale) for v in sc.text.split())
            for p in vis.iter('pose'):
                vals = [float(v) for v in p.text.split()]
                p.text = ' '.join(str(v * scale) if k < 3 else str(v) for k, v in enumerate(vals))
        out.append(ET.tostring(vis, encoding='unicode'))
    return out


def drifting_model(name, visuals, x, y, z, roll, yaw, vel):
    """重力なし・当たり判定なしで、海流の速度 vel=(東, 北) で漂流するモデル。

    VelocityControl の initial_linear はモデル座標系で与えるので、
    ワールド座標系の海流速度をモデルの向き(roll, yaw)で逆回転して渡す。
    """
    c, s_ = math.cos(yaw), math.sin(yaw)
    u0, u1 = c * vel[0] + s_ * vel[1], -s_ * vel[0] + c * vel[1]
    v_body = (u0, math.cos(roll) * u1, -math.sin(roll) * u1)
    return f"""    <model name="{name}"><pose>{x:.2f} {y:.2f} {z:.2f} {roll} 0 {yaw}</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>70</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz></inertia></inertial>
        {''.join(visuals)}
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <initial_linear>{v_body[0]:.4f} {v_body[1]:.4f} {v_body[2]:.4f}</initial_linear>
      </plugin>
    </model>
"""


# ---- 自律フォイルボード(eFoil 型)-------------------------------------------
# ボード本体はサーフボードの輪郭を押し出したメッシュ(OBJ)で作る。x が前、y が左、z が上 [m]
BOARD_TAIL, BOARD_NOSE = -0.85, 0.95     # 後端と先端の x
BOARD_HALF_W = 0.32                      # 最大の半幅
BOARD_X_WIDEST = 0.10                    # 一番幅が広い所の x
BOARD_CORNER = 0.07                      # 後ろの角の丸み
BOARD_TOP = 0.06                         # デッキの高さ
BOARD_RGB = (0.52, 0.78, 0.78)           # 水色


def _board_half_width(x):
    if x <= BOARD_X_WIDEST:
        w = BOARD_HALF_W - 0.03 * ((BOARD_X_WIDEST - x) / (BOARD_X_WIDEST - BOARD_TAIL)) ** 2
        d = x - BOARD_TAIL
        if d < BOARD_CORNER:                                   # 後ろの角を丸める
            w -= BOARD_CORNER - math.sqrt(max(0.0, BOARD_CORNER ** 2 - (BOARD_CORNER - d) ** 2))
        return w
    t = (x - BOARD_X_WIDEST) / (BOARD_NOSE - BOARD_X_WIDEST)
    return BOARD_HALF_W * (1 - t ** 2.2) ** 0.62               # 先端へ丸く細くなる


def _board_thickness(x):
    return 0.12 - 0.06 * max(0.0, (x - 0.3) / (BOARD_NOSE - 0.3))    # 先端ほど薄い


def _board_rocker(x):
    return 0.07 * max(0.0, (x - 0.45) / (BOARD_NOSE - 0.45)) ** 2     # 先端の反り


def write_board_mesh(path, n=60):
    """ボード本体のメッシュ(OBJ と MTL)を書き出す。"""
    xs = [BOARD_TAIL + (BOARD_NOSE - BOARD_TAIL) * i / n for i in range(n + 1)]
    outline = [(x, _board_half_width(x)) for x in xs] + \
              [(x, -_board_half_width(x)) for x in reversed(xs[1:-1])]
    v = [(x, y, BOARD_TOP + _board_rocker(x)) for x, y in outline] + \
        [(x, y, BOARD_TOP - _board_thickness(x) + _board_rocker(x)) for x, y in outline]
    m = len(outline)
    v += [(0.0, 0.0, BOARD_TOP), (0.0, 0.0, BOARD_TOP - 0.12)]
    ct, cb = len(v) - 1, len(v)
    faces = []
    for i in range(m):
        j = (i + 1) % m
        faces += [(ct, j + 1, i + 1), (cb, m + i + 1, m + j + 1), (i + 1, j + 1, m + j + 1, m + i + 1)]

    def normal(f):
        a, b, c = (v[k - 1] for k in f[:3])
        u = [b[k] - a[k] for k in range(3)]
        w = [c[k] - a[k] for k in range(3)]
        nn = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        ln = math.sqrt(sum(q * q for q in nn)) or 1.0
        return [q / ln for q in nn]
    path = Path(path)
    mtl = path.with_suffix('.mtl')
    mtl.write_text('newmtl board\nKa %.3f %.3f %.3f\nKd %.3f %.3f %.3f\nKs 0.2 0.2 0.2\nNs 30\nd 1\n'
                   % (BOARD_RGB + BOARD_RGB))
    lines = ['# 自動生成: worlds/make_world.py(eFoil のボード本体)', f'mtllib {mtl.name}', 'usemtl board']
    lines += ['v %.4f %.4f %.4f' % p for p in v]
    for k, f in enumerate(faces):
        lines.append('vn %.4f %.4f %.4f' % tuple(normal(f)))
        lines.append('f ' + ' '.join(f'{i}//{k + 1}' for i in f))
    path.write_text('\n'.join(lines) + '\n')


def board_model(name, x, y, yaw, mesh_uri):
    """自律フォイルボード(eFoil 型:ボード + マスト + モーター + 前後の水中翼)。
    見た目だけで、動きは board_node が与える。原点はボードの中心で、デッキは +0.06 m。"""
    def vis(n, pose, geom, rgb):
        return (f'<visual name="{n}"><pose>{pose}</pose><geometry>{geom}</geometry>'
                f'<material><diffuse>{rgb} 1</diffuse><ambient>{rgb} 1</ambient></material></visual>')
    teal, dark, pad = '%.2f %.2f %.2f' % BOARD_RGB, '0.12 0.12 0.13', '0.25 0.27 0.28'
    v = [
        vis('board', '0 0 0 0 0 0', f'<mesh><uri>{mesh_uri}</uri></mesh>', teal),
        vis('pad', '-0.30 0 0.064 0 0 0', '<box><size>0.80 0.48 0.008</size></box>', pad),
        vis('hatch', '0.30 0 0.068 0 0 0', '<box><size>0.26 0.20 0.008</size></box>', '0.30 0.33 0.35'),
        vis('mast', '-0.35 0 -0.42 0 0 0', '<box><size>0.06 0.13 0.78</size></box>', dark),
        vis('fuselage', '-0.25 0 -0.82 0 1.5708 0', '<cylinder><radius>0.025</radius><length>0.95</length></cylinder>', dark),
        vis('motor', '-0.40 0 -0.82 0 1.5708 0', '<cylinder><radius>0.06</radius><length>0.26</length></cylinder>', dark),
        vis('prop', '-0.56 0 -0.82 0 1.5708 0', '<cylinder><radius>0.10</radius><length>0.015</length></cylinder>', '0.2 0.2 0.22'),
        vis('wing_front', '0.17 0 -0.82 0 0 0', '<ellipsoid><radii>0.13 0.42 0.014</radii></ellipsoid>', teal),
        vis('wing_rear', '-0.68 0 -0.82 0 0 0', '<ellipsoid><radii>0.07 0.21 0.01</radii></ellipsoid>', teal),
    ]
    return f"""    <model name="{name}"><pose>{x:.2f} {y:.2f} 0.05 0 0 {yaw:.4f}</pose>
      <link name="link"><gravity>false</gravity>
        <inertial><mass>25</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        {''.join(v)}
      </link>
    </model>
"""


def beacon_model(name):
    """到着時に展開する発光マーカー(到着までは水中に隠す)。"""
    return f"""    <model name="{name}"><pose>0 0 -20 0 0 0</pose>
      <link name="link"><gravity>false</gravity>
        <inertial><mass>0.5</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
        <visual name="pole"><pose>0 0 0.4 0 0 0</pose><geometry><cylinder><radius>0.02</radius><length>0.8</length></cylinder></geometry>
          <material><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
        <visual name="lamp"><pose>0 0 0.85 0 0 0</pose><geometry><sphere><radius>0.16</radius></sphere></geometry>
          <material><diffuse>1 0.3 0.1 1</diffuse><ambient>1 0.3 0.1 1</ambient><emissive>1 0.35 0.1 1</emissive></material></visual>
      </link>
    </model>
"""


def include(model, inst, x, y, yaw, z=0.0):
    return f"""    <include><uri>{FUEL_URL}{model.replace(' ', '%20')}</uri><name>{inst}</name>
      <pose>{x} {y} {z} 0 0 {yaw}</pose><static>true</static></include>
"""


def debris(name, x, y, length, yaw):
    return f"""    <model name="{name}"><static>true</static><pose>{x} {y} 0.05 0 1.5708 {yaw}</pose>
      <link name="link"><visual name="v"><geometry><cylinder><radius>0.18</radius><length>{length}</length></cylinder></geometry>
        <material><diffuse>0.45 0.33 0.2 1</diffuse><specular>0 0 0 1</specular></material></visual></link></model>
"""


def main():
    here = Path(__file__).resolve().parent
    vx, vy = current_vector()
    dx, dy = vx * INCIDENT_ELAPSED_S, vy * INCIDENT_ELAPSED_S   # 出動までに流された量
    parts = [HEADER]
    persons = []
    for pid, model, dn, de, z, yaw, note in PERSONS:
        x0, y0 = DATUM_EAST + de + dx, DATUM_NORTH + dn + dy
        parts.append(drifting_model(f'person_{pid}', fuel_visuals(model), x0, y0, z, 0.0, yaw, (vx, vy)))
        persons.append(dict(id=pid, model=model, note=note,
                            north0=round(y0, 2), east0=round(x0, 2)))
    if CAPSIZED:
        bm, bn, be, scale = CAPSIZED
        parts.append(drifting_model('capsized_boat', fuel_visuals(bm, scale),
                                    DATUM_EAST + be + dx, DATUM_NORTH + bn + dy, 0.25, math.pi, 0.8, (vx, vy)))
    # ボードのメッシュは gz_models/efoil_board に書き出し、model:// で参照する
    # (setup.sh が PX4 のモデルのフォルダにリンクを張る。絶対パスをワールドに書かないため)
    write_board_mesh(here.parent / 'gz_models' / 'efoil_board' / 'meshes' / 'efoil_board.obj')
    for bid, bn, be, hdg in BOARDS:
        # ENU のヨー(東=0、反時計回り)= 90° - 方位
        parts.append(board_model(f'board_{bid}', be, bn, math.radians(90.0 - hdg),
                                 'model://efoil_board/meshes/efoil_board.obj'))
        parts.append(beacon_model(f'beacon_{bid}'))
    for i, (m, x, y, yaw) in enumerate(STATIC_FUEL):
        parts.append(include(m, f'static_{i}', x, y, yaw))
    for i, d in enumerate(DEBRIS):
        parts.append(debris(f'debris_{i}', *d))
    parts.append('  </world>\n</sdf>\n')
    (here / f'{WORLD}.sdf').write_text(''.join(parts))

    targets = dict(
        world=WORLD, home_lat=HOME_LAT, home_lon=HOME_LON,
        frame='NED(north=y, east=x) [m]。north0/east0 はシミュレーション開始時の位置',
        current=dict(speed=CURRENT_SPEED, to_deg=CURRENT_TO_DEG,
                     v_north=round(vy, 4), v_east=round(vx, 4)),
        incident=dict(datum_north=DATUM_NORTH, datum_east=DATUM_EAST,
                      elapsed_s=INCIDENT_ELAPSED_S),
        persons=persons,
        boards=[dict(id=b, north=n, east=e, heading_deg=h) for b, n, e, h in BOARDS],
    )
    (here / 'targets.json').write_text(json.dumps(targets, ensure_ascii=False, indent=2))
    print('wrote', here / f'{WORLD}.sdf', here / 'targets.json')


if __name__ == '__main__':
    main()
