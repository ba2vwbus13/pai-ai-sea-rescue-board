#!/usr/bin/env python3
"""自律航行フォイルボード(複数台)のシミュレーション。

ドローンから LoRa(/rescue/lora/rx)で届く「遭難者の推定位置・推定時刻・海流」に向かって、
各ボードが自律航行する。
  - 誘導: 遭難者の予測位置(推定位置 + 海流 × 経過時間)へ。到着までの時間だけ先を読む(先回り)
  - 方位制御: 目標方位との偏差に比例してヨーレートを決める(P 制御、50 Hz 相当)
  - 速度制御: 遠い間は最高速、近づくと減速。方位偏差が大きいときも減速
  - 離水: 速度が離水速度を超えると水中翼で船体が浮く(高さを滑らかに変える)
  - 停止: 遭難者の手前 standoff [m] で停止(逆転で急減速)→ 浮体として待機し、
    海流で遭難者と一緒に流れる。発光マーカーを展開する
ボードは海流にも流される(誘導はそれを含めて補正する)。

Gazebo 上のボードとマーカーは重力なし・当たり判定なしのモデルで、このノードが set_pose で動かす。
gz-transport の Python バインディングを使う(システムの python3 で動かすこと)。
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

STATUS_JP = dict(IDLE='待機(砂浜)', TRANSIT='航行中', APPROACH='接近', HOLD='到着・浮体として待機')


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


class Board:
    def __init__(self, bid, n, e, hdg_deg):
        self.id = bid
        self.n, self.e, self.psi = n, e, math.radians(hdg_deg)   # NED, 方位(北=0、時計回り)
        self.u = 0.0               # 対水速度 [m/s]
        self.r = 0.0               # ヨーレート [rad/s]
        self.z = 0.05              # 船体の高さ(離水で上がる)
        self.status = 'IDLE'
        self.target = None         # dict(id, north, east, t_est, vn, ve)
        self.t_dispatch = None
        self.t_arrive = None
        self.n_updates = 0
        self.dist_go = None
        self.trail = []


class BoardNode(Node):
    def __init__(self):
        super().__init__('board_sim')
        params = dict(world='sea_board', boards='B1:-5:12:180,B2:-5:18:180,B3:-5:24:180',
                      max_speed=6.0, takeoff_speed=4.0, accel=1.5, decel=3.0,
                      yaw_kp=2.0, max_yaw_rate=1.0, standoff=2.0, slow_radius=25.0,
                      current_speed=0.25, current_to_deg=200.0, foil_height=0.32)
        for k, v in params.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in params}
        a = math.radians(self.p['current_to_deg'])
        self.cvn, self.cve = self.p['current_speed'] * math.cos(a), self.p['current_speed'] * math.sin(a)
        self.boards = {}
        for item in self.p['boards'].split(','):
            bid, n, e, h = item.split(':')
            self.boards[bid] = Board(bid, float(n), float(e), float(h))
        self.gz = GzNode()
        self.world = self.p['world']
        self.pub_state = self.create_publisher(String, '/rescue/boards', 10)
        self.create_subscription(String, '/rescue/lora/rx', self._on_rx, 20)
        self.dt = 0.05
        self.tick = 0
        self.create_timer(self.dt, self._step)
        self.get_logger().info(f'フォイルボード {len(self.boards)} 台: 待機中')

    # ---- LoRa 受信 ----------------------------------------------------------
    def _on_rx(self, msg):
        d = json.loads(msg.data)
        b = self.boards.get(d.get('board'))
        if b is None:
            return
        tgt = dict(id=d['target'], north=d['north'], east=d['east'], t_est=d['t_est'],
                   vn=d['v_north'], ve=d['v_east'])
        if d['type'] == 'dispatch' and b.status == 'IDLE':
            b.target, b.status, b.t_dispatch = tgt, 'TRANSIT', time.time()
            self.get_logger().info(f'{b.id}: 発進の合図を受信 → {tgt["id"]} へ')
        elif d['type'] == 'update' and b.target and b.target['id'] == tgt['id']:
            b.target = tgt
            b.n_updates += 1

    # ---- 1 ステップ ----------------------------------------------------------
    def _guidance(self, b, now):
        """先回り誘導: 到着予定時刻の遭難者の予測位置から、手前 standoff [m] の点を狙う。"""
        t = b.target
        pn = t['north'] + t['vn'] * (now - t['t_est'])
        pe = t['east'] + t['ve'] * (now - t['t_est'])
        d = math.hypot(pn - b.n, pe - b.e)
        t_go = d / max(b.u, 2.0)
        pn += t['vn'] * t_go
        pe += t['ve'] * t_go
        d = math.hypot(pn - b.n, pe - b.e)
        s = self.p['standoff']
        an, ae = pn - s * (pn - b.n) / max(d, 1e-3), pe - s * (pe - b.e) / max(d, 1e-3)
        return an, ae, d

    def _step(self):
        now = time.time()
        p = self.p
        dt = self.dt
        for b in self.boards.values():
            if b.status in ('TRANSIT', 'APPROACH'):
                an, ae, d_person = self._guidance(b, now)
                d_aim = math.hypot(an - b.n, ae - b.e)
                b.dist_go = d_person
                bearing = math.atan2(ae - b.e, an - b.n)
                err = wrap(bearing - b.psi)
                rmax = p['max_yaw_rate'] * (0.6 if b.z > 0.2 else 1.0)   # 離水中は旋回を緩やかに
                b.r = max(-rmax, min(rmax, p['yaw_kp'] * err))
                u_cmd = p['max_speed'] if d_aim > p['slow_radius'] else \
                    max(0.8, p['max_speed'] * d_aim / p['slow_radius'])
                if abs(err) > math.radians(60):
                    u_cmd *= 0.4
                b.status = 'APPROACH' if d_aim < p['slow_radius'] else 'TRANSIT'
                if d_aim < 0.8:
                    # 停止シーケンス: 逆転で急減速して浮体として待機
                    b.status, b.t_arrive, b.u, b.r = 'HOLD', now, 0.0, 0.0
                    self.get_logger().info(f'{b.id}: {b.target["id"]} の手前に到着(遭難者まで {d_person:.1f} m)')
                else:
                    du = u_cmd - b.u
                    b.u += max(-p['decel'] * dt, min(p['accel'] * dt, du))
            else:
                b.r = 0.0
                b.u = 0.0
            b.psi = wrap(b.psi + b.r * dt)
            # 海の上では海流にも流される(砂浜で待機中は流されない)
            drift = b.status != 'IDLE'
            b.n += (b.u * math.cos(b.psi) + (self.cvn if drift else 0.0)) * dt
            b.e += (b.u * math.sin(b.psi) + (self.cve if drift else 0.0)) * dt
            z_cmd = p['foil_height'] if b.u > p['takeoff_speed'] else 0.05
            b.z += max(-0.3 * dt, min(0.3 * dt, z_cmd - b.z))
            if self.tick % 10 == 0 and b.status != 'IDLE':
                b.trail.append((round(b.n, 1), round(b.e, 1)))
        self._send_poses()
        self.tick += 1
        if self.tick % 4 == 0:
            self._publish_state(now)

    def _send_poses(self):
        req = Pose_V()
        for b in self.boards.values():
            yaw_enu = math.pi / 2 - b.psi
            roll = -0.25 * b.r * min(b.u, 6.0) / 6.0                 # 旋回で内側に傾く
            pitch = -0.05 if 0.07 < b.z < self.p['foil_height'] - 0.02 and b.u > 3 else 0.0
            q = quat_from_rpy(roll, pitch, yaw_enu)
            ps = req.pose.add()
            ps.name = f'board_{b.id}'
            ps.position.x, ps.position.y, ps.position.z = b.e, b.n, b.z
            ps.orientation.w, ps.orientation.x, ps.orientation.y, ps.orientation.z = q
            pm = req.pose.add()
            pm.name = f'beacon_{b.id}'
            if b.status == 'HOLD':
                pm.position.x, pm.position.y, pm.position.z = b.e, b.n, b.z + 0.15
            else:
                pm.position.x, pm.position.y, pm.position.z = b.e, b.n, -20.0
            pm.orientation.w = 1.0
        self.gz.request(f'/world/{self.world}/set_pose_vector', req, Pose_V, Boolean, 200)

    def _publish_state(self, now):
        out = []
        for b in self.boards.values():
            out.append(dict(id=b.id, status=b.status, status_jp=STATUS_JP[b.status],
                            north=round(b.n, 2), east=round(b.e, 2), heading=round(math.degrees(b.psi), 1),
                            speed_kmh=round(b.u * 3.6, 1), foiling=b.z > 0.2,
                            target=b.target['id'] if b.target else None,
                            dist_go=None if b.dist_go is None else round(b.dist_go, 1),
                            t_dispatch=b.t_dispatch, t_arrive=b.t_arrive, n_updates=b.n_updates,
                            trail=b.trail[-400:]))
        self.pub_state.publish(String(data=json.dumps(dict(now=now, boards=out))))


def main(args=None):
    rclpy.init(args=args)
    node = BoardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
