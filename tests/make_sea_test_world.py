#!/usr/bin/env python3
"""海面での検出性能を調べるテストワールド(海面の種類 × カメラ条件)。

海面 A: 不透明な単色の海(平面) / 海面 B: Fuel の Coast Water(波シェーダー)
各カメラの注視点のまわりに、立ち泳ぎの人(胸まで水中)・浮いている人・ボート・ブイを置く。
"""
import math
from pathlib import Path

FUEL = 'https://fuel.gazebosim.org/1.0/'
CONDITIONS = [(18, 30), (14, 30), (10, 35)]           # (高度 m, 真下からの傾き deg)
TARGETS = [  # (モデル, 東 dx, 北 dy, 沈める深さ z, ヨー)
    ('OpenRobotics/models/Casual female', -3.0, -2.0, -1.0, 0.3),
    ('OpenRobotics/models/Standing person', 3.0, -2.5, -1.1, 1.2),
    ('OpenRobotics/models/Rescue Randy', 0.0, 2.5, -0.12, 0.7),
    ('OpenRobotics/models/Walking person', -2.5, 3.5, -1.2, 2.2),
    ('OpenRobotics/models/Vessel A', 14.0, 10.0, 0.0, 0.5),
    ('OpenRobotics/models/Vessel C', -14.0, 12.0, 0.0, 2.0),
    ('OpenRobotics/models/mb_round_buoy_orange', -6.0, 0.0, 0.0, 0.0),
]

HEADER = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="sea_test">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.6 0.7 0.9 1</background><shadows>true</shadows><sky/></scene>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose><cast_shadows>true</cast_shadows>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.4 0.4 0.4 1</specular>
      <direction>0.3 0.4 -0.85</direction>
    </light>
"""


def main():
    parts = [HEADER]
    # 海面 A(x < 150): 不透明な平面 / 海面 B(x >= 150): Coast Water
    parts.append("""    <model name="sea_flat"><static>true</static><pose>100 0 0 0 0 0</pose><link name="l">
      <visual name="v"><geometry><box><size>600 300 0.02</size></box></geometry>
        <material><diffuse>0.06 0.28 0.42 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual>
    </link></model>
    <include><uri>""" + FUEL + """OpenRobotics/models/Coast%20Water</uri><name>coast_water</name>
      <pose>500 0 0 0 0 0</pose><static>true</static></include>
""")
    k = 0
    for base_x, sea in ((0.0, 'flat'), (400.0, 'wave')):
        for i, (alt, tilt) in enumerate(CONDITIONS):
            cx = base_x + i * 90.0
            look = alt * math.tan(math.radians(tilt))
            parts.append(f"""    <include><uri>model://mono_cam</uri><name>cam_{sea}_{alt}m_{tilt}deg</name>
      <pose>{cx - look} 0 {alt} 0 {math.radians(90 - tilt)} 0</pose><static>true</static></include>
""")
            for name, dx, dy, z, yaw in TARGETS:
                k += 1
                parts.append(f"""    <include><uri>{FUEL}{name.replace(' ', '%20')}</uri><name>t{k}</name>
      <pose>{cx + dx} {dy} {z} 0 0 {yaw}</pose><static>true</static></include>
""")
    parts.append('  </world>\n</sdf>\n')
    out = Path(__file__).with_name('sea_test.sdf')
    out.write_text(''.join(parts))
    print(out)


if __name__ == '__main__':
    main()
