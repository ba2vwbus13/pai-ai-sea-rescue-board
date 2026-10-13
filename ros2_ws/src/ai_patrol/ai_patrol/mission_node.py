#!/usr/bin/env python3
"""海難救助ミッションノード(ドローン側): 漂流予測 → 拡大方形捜索 → AI で発見 → 再確認
→ オペレータの GO → フォイルボードを出動させ、到着まで上空から座標を送り続ける。

状態遷移:
  INIT → ARMING → TAKEOFF → TRANSIT(漂流を補正した捜索基点へ)→ SEARCH ─(人の候補)→ APPROACH
     → DESCEND → OBSERVE ─(確定)→ OPERATOR(映像を確認して GO)→ ESCORT(ボートを出動させ、
       遭難者を追いながら LoRa で座標を送る)─(ボート到着)→ CLIMB → RESUME → SEARCH …
                         └(誤検出として棄却)→ CLIMB
  SEARCH(全員発見してボートが到着 or 捜索パターン完了)→ RETURN → LAND → DONE(報告書を保存)

映像の伝送: 候補を見つけてから GO が出るまで(APPROACH〜OPERATOR)だけ、映像を地上へ送る想定。
その時間を数えて、常時伝送と比べた通信量の削減率を報告書に残す。

海難救助ならではの工夫:
  1. 漂流予測: 通報された転覆地点(datum)を、海流の予報値 × 経過時間だけずらして
     捜索の基点にする(IAMSAR マニュアルの datum の考え方)。
  2. 拡大方形捜索(Expanding Square Search): 基点から 1S, 1S, 2S, 2S, 3S, … と
     渦巻き状に広げる。S はカメラの撮影幅から決める。
  3. 動く対象の追跡: 要救助者は海流で流され続けるので、各対象の推定位置に
     「推定した時刻からの経過時間 × 海流」を足した予測位置で照合・接近・投下する。
  4. 乗員数(POB)の活用: 通報された乗員数を全員確定したら、捜索を打ち切って帰還する。
"""
import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

from .geo import footprint, look_offset, ned_to_latlon

STATE_JP = {
    'INIT': '準備中', 'ARMING': 'アーム', 'TAKEOFF': '離陸', 'TRANSIT': '捜索基点へ移動',
    'SEARCH': '拡大方形捜索(AI 監視中)', 'APPROACH': '候補へ移動', 'DESCEND': '降下して確認',
    'OBSERVE': 'AI で再確認', 'OPERATOR': 'オペレータが映像を確認', 'ESCORT': 'ボートを誘導中',
    'CLIMB': '上昇', 'RESUME': '捜索へ復帰',
    'RETURN': '帰還', 'LAND': '着陸', 'DONE': '完了',
}


def expanding_square(center_n, center_e, spacing, n_legs, first_bearing_deg):
    """拡大方形捜索の経路点。基点から 1S, 1S, 2S, 2S, 3S, 3S, … の脚で右回りに広げる。"""
    pts = [(center_n, center_e)]
    n, e = center_n, center_e
    b = math.radians(first_bearing_deg)
    for i in range(n_legs):
        length = spacing * (i // 2 + 1)
        n += length * math.cos(b)
        e += length * math.sin(b)
        pts.append((n, e))
        b += math.pi / 2                     # 90° 右へ曲がる
    return pts


class Target:
    """要救助者の候補。位置は時刻 t_est での推定値で、以降は海流で流れると予測する。"""

    def __init__(self, tid, north, east, conf, t):
        self.id = tid
        self.north, self.east, self.t_est = north, east, t
        self.first_north, self.first_east, self.first_conf, self.first_t = north, east, conf, t
        self.status = 'pending'       # pending / inspecting / confirmed / rejected
        self.obs = []                 # 確認高度での (north, east, conf, t)
        self.confirm_t = None
        self.board = None             # 出動させたボート
        self.dispatch_t = None
        self.arrive_t = None

    def predict(self, t, vn, ve):
        dt = t - self.t_est
        return self.north + vn * dt, self.east + ve * dt

    def as_dict(self, lat0, lon0, t, vn, ve):
        pn, pe = self.predict(t, vn, ve)
        lat, lon = ned_to_latlon(pn, pe, lat0, lon0)
        return dict(id=self.id, status=self.status,
                    north=round(pn, 2), east=round(pe, 2), lat=round(lat, 7), lon=round(lon, 7),
                    est_north=round(self.north, 2), est_east=round(self.east, 2), est_t=self.t_est,
                    first_north=round(self.first_north, 2), first_east=round(self.first_east, 2),
                    first_t=self.first_t, first_conf=round(self.first_conf, 3),
                    confirm_t=self.confirm_t, n_confirm=len(self.obs),
                    max_conf=round(max([o[2] for o in self.obs], default=0.0), 3),
                    board=self.board, dispatch_t=self.dispatch_t, arrive_t=self.arrive_t)


class MissionNode(Node):
    def __init__(self):
        super().__init__('ai_mission')
        defaults = dict(
            # 通報内容と海流の予報(ワールドの真値とは独立に与える想定)
            datum_north=-70.0, datum_east=20.0, incident_elapsed_s=120.0,
            current_speed=0.25, current_to_deg=200.0, persons_on_board=3,
            # 捜索
            patrol_alt=18.0, inspect_alt=10.0, escort_alt=12.0, hfov=1.74, cam_tilt_deg=30.0,
            search_overlap=0.25, search_legs=10,
            patrol_speed=7.0, transit_speed=10.0, vertical_speed=3.0,
            candidate_conf=0.45, confirm_conf=0.60, confirm_count=2,
            merge_radius=8.0, observe_s=3.0, observe_timeout_s=6.0, operator_delay_s=3.0,
            lora_update_s=1.0, video_mbps=20.0,
            accept_radius=1.5, home_lat=26.5472, home_lon=128.0540,
            report_dir=os.path.expanduser('~/work/pai_board/results'),
            target_labels=['person'],
        )
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        c = self.cfg

        a = math.radians(c['current_to_deg'])
        self.vn, self.ve = c['current_speed'] * math.cos(a), c['current_speed'] * math.sin(a)
        self.t_launch = time.time()
        self.swath, _ = footprint(c['patrol_alt'], c['hfov'])
        self.spacing = self.swath * (1.0 - c['search_overlap'])
        self.route = None             # 離陸後に漂流を補正して作る
        self.wp = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.pub_state = self.create_publisher(String, '/ai_patrol/mission_state', 10)
        self.pub_lora = self.create_publisher(String, '/rescue/lora/tx', 20)
        self.create_subscription(String, '/rescue/boards', self._on_boards, 10)
        self.create_subscription(String, '/rescue/lora/stats', self._on_lora_stats, 10)
        self.boards = {}
        self.lora_stats = None
        self.video_s = 0.0            # 映像を地上へ送った時間 [s]
        self._last_lora = 0.0
        for t in ('/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_local_position'):
            self.create_subscription(VehicleLocalPosition, t, self._on_pos, qos)
        for t in ('/fmu/out/vehicle_status_v4', '/fmu/out/vehicle_status'):
            self.create_subscription(VehicleStatus, t, self._on_status, qos)
        self.create_subscription(String, '/ai_patrol/detections', self._on_detections, 10)

        self.pos = None
        self.nav_state = self.arming_state = None
        self.state = 'INIT'
        self.state_t = time.time()
        self.t_start = None
        self.tick = 0
        self.carrot = None
        self.yaw = float('nan')
        self.targets = []
        self.current = None
        self.bearing = 0.0
        self.resume_point = None
        self.trail = []
        self.log_events = []
        self.dist_flown = 0.0
        self._last_pos = None
        self.n_dispatched = 0
        self.n_arrived = 0
        self.search_center = None

        self.get_logger().info(
            f'海流 {c["current_speed"]} m/s → {c["current_to_deg"]}°, 捜索の脚の間隔 S = {self.spacing:.1f} m')
        self.create_timer(0.1, self._on_timer)

    # ---- 受信 ---------------------------------------------------------------
    def _on_pos(self, msg):
        self.pos = msg
        if self._last_pos is not None:
            self.dist_flown += math.hypot(msg.x - self._last_pos[0], msg.y - self._last_pos[1])
        self._last_pos = (msg.x, msg.y)

    def _on_status(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def _on_boards(self, msg):
        self.boards = {b['id']: b for b in json.loads(msg.data)['boards']}

    def _on_lora_stats(self, msg):
        self.lora_stats = json.loads(msg.data)

    def _send_lora(self, kind, tg, now):
        """遭難者の推定位置・時刻・海流をボートへ送る(数十バイトに収まる数値だけ)。"""
        self.pub_lora.publish(String(data=json.dumps(dict(
            type=kind, board=tg.board, target=tg.id, north=round(tg.north, 2), east=round(tg.east, 2),
            t_est=tg.t_est, v_north=round(self.vn, 3), v_east=round(self.ve, 3)))))
        self._last_lora = now

    def _on_detections(self, msg):
        data = json.loads(msg.data)
        t = data['stamp']
        dets = [d for d in data['detections'] if d['label'] in self.cfg['target_labels']]
        if self.state in ('OBSERVE', 'ESCORT') and self.current is not None:
            tg = self.current
            pn, pe = tg.predict(t, self.vn, self.ve)
            for d in dets:
                if math.hypot(d['north'] - pn, d['east'] - pe) < 5.0:
                    tg.obs.append((d['north'], d['east'], d['conf'], t))
        elif self.state in ('SEARCH', 'RESUME', 'APPROACH', 'CLIMB', 'TRANSIT', 'ESCORT'):
            for d in dets:
                if d['conf'] >= self.cfg['candidate_conf']:
                    self._add_candidate(d, t)

    def _add_candidate(self, d, t):
        if d['north'] > -8.0:          # 砂浜(陸)と水際で待機中のボートの周りは対象外
            return
        for tg in self.targets:
            pn, pe = tg.predict(t, self.vn, self.ve)
            if math.hypot(pn - d['north'], pe - d['east']) < self.cfg['merge_radius']:
                return
        tg = Target(f'T{len(self.targets) + 1}', d['north'], d['east'], d['conf'], t)
        self.targets.append(tg)
        self._event(f'{tg.id}: 人の候補を発見 (確信度 {d["conf"]:.2f})')

    # ---- PX4 送信 -----------------------------------------------------------
    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command, m.param1, m.param2 = command, p1, p2
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = self._now_us()
        self.pub_cmd.publish(m)

    def _send(self, n, e, alt, vel=(math.nan, math.nan, math.nan)):
        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = not math.isnan(vel[0])
        mode.timestamp = self._now_us()
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.position = [float(n), float(e), float(-alt)]
        sp.velocity = [float(v) for v in vel]
        sp.yaw = float(self.yaw)
        sp.timestamp = self._now_us()
        self.pub_sp.publish(sp)

    def _fly_to(self, n, e, alt, speed, dt=0.1, yaw=None, tol=None):
        """目標点 carrot を (n, e, alt) へ speed [m/s] で進めて送信。到着したら True。"""
        if self.carrot is None:
            self.carrot = [self.pos.x, self.pos.y, -self.pos.z]
        cn, ce, ca = self.carrot
        dn, de = n - cn, e - ce
        dh = math.hypot(dn, de)
        vel = [0.0, 0.0, 0.0]
        step = speed * dt
        if dh > step:
            cn += dn / dh * step
            ce += de / dh * step
            vel[0], vel[1] = dn / dh * speed, de / dh * speed
            if yaw is None and dh > 3.0:
                self.yaw = math.atan2(de, dn)
        else:
            cn, ce = n, e
        if yaw is not None:
            self.yaw = yaw
        da = alt - ca
        vstep = self.cfg['vertical_speed'] * dt
        if abs(da) > vstep:
            ca += math.copysign(vstep, da)
            vel[2] = -math.copysign(self.cfg['vertical_speed'], da)
        else:
            ca = alt
        self.carrot = [cn, ce, ca]
        self._send(cn, ce, ca, vel)
        err = math.sqrt((n - self.pos.x) ** 2 + (e - self.pos.y) ** 2 + (alt + self.pos.z) ** 2)
        return (cn, ce, ca) == (n, e, alt) and err < (tol or self.cfg['accept_radius'])

    def _standoff(self, tg, alt, t):
        """候補が画像の中央に写る機体の位置(予測位置の手前 alt*tan(傾き))。"""
        pn, pe = tg.predict(t, self.vn, self.ve)
        off = look_offset(alt, math.radians(self.cfg['cam_tilt_deg']))
        return pn - off * math.cos(self.bearing), pe - off * math.sin(self.bearing)

    # ---- 状態遷移 -----------------------------------------------------------
    def _goto(self, state):
        self.get_logger().info(f'[{self.state} → {state}] {STATE_JP[state]}')
        self.state = state
        self.state_t = time.time()

    def _event(self, text):
        el = 0.0 if self.t_start is None else time.time() - self.t_start
        self.log_events.append(dict(t=round(el, 1), text=text))
        self.get_logger().info(text)

    def _next_pending(self):
        now = time.time()
        pend = [t for t in self.targets if t.status == 'pending']
        if not pend:
            return None
        return min(pend, key=lambda t: math.hypot(*(a - b for a, b in zip(
            t.predict(now, self.vn, self.ve), (self.pos.x, self.pos.y)))))

    def _n_confirmed(self):
        """ボートが到着した遭難者の数。"""
        return sum(t.status == 'confirmed' for t in self.targets)

    def _plan_search(self):
        """漂流を補正した捜索基点と拡大方形捜索の経路を作る。"""
        c = self.cfg
        # 基点に着くころの漂流位置: 通報からの経過時間 + 出動後の経過時間 + 移動時間の見込み
        dist = math.hypot(c['datum_north'], c['datum_east'])
        elapsed = c['incident_elapsed_s'] + (time.time() - self.t_launch) + dist / c['transit_speed']
        cn = c['datum_north'] + self.vn * elapsed
        ce = c['datum_east'] + self.ve * elapsed
        self.search_center = (cn, ce)
        self.route = expanding_square(cn, ce, self.spacing, c['search_legs'], c['current_to_deg'])
        self._event(f'漂流予測: 通報地点から {c["current_speed"] * elapsed:.0f} m 流されたと推定。'
                    f'その地点を中心に拡大方形捜索を開始します')

    def _on_timer(self):
        self.tick += 1
        if self.pos is None:
            if self.tick % 30 == 0:
                self.get_logger().warn('PX4 の位置情報待ち(SITL と MicroXRCEAgent を確認)')
            return
        c = self.cfg
        now = time.time()
        if self.state not in ('DONE', 'LAND') and self.tick % 5 == 0:
            self.trail.append((round(self.pos.x, 1), round(self.pos.y, 1)))
        elapsed_state = now - self.state_t
        if self.state in ('APPROACH', 'DESCEND', 'OBSERVE', 'OPERATOR'):
            self.video_s += 0.1        # 候補を見つけてから GO までだけ映像を伝送する

        if self.state == 'INIT':
            self._send(self.pos.x, self.pos.y, -self.pos.z)
            if self.tick > 20:
                self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self._goto('ARMING')

        elif self.state == 'ARMING':
            self._send(self.pos.x, self.pos.y, -self.pos.z)
            if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                self.t_start = now
                self.home = (self.pos.x, self.pos.y)
                self._event(f'通報: 遊泳者 {c["persons_on_board"]} 名が離岸流で沖へ。'
                            f'事案発生から {c["incident_elapsed_s"] / 60:.0f} 分後に出動')
                self._goto('TAKEOFF')
            elif elapsed_state > 3.0:
                self._goto('INIT')

        elif self.state == 'TAKEOFF':
            if self._fly_to(self.home[0], self.home[1], c['patrol_alt'], c['transit_speed']):
                self._plan_search()
                self._goto('TRANSIT')

        elif self.state == 'TRANSIT':
            n, e = self.route[0]
            nxt = self._next_pending()
            if nxt is not None:
                self._start_inspection(nxt)
            elif self._fly_to(n, e, c['patrol_alt'], c['transit_speed']):
                self.wp = 1
                self._goto('SEARCH')

        elif self.state in ('SEARCH', 'RESUME'):
            nxt = self._next_pending()
            if nxt is not None:
                self._start_inspection(nxt)
            elif self._n_confirmed() >= c['persons_on_board']:
                self._event(f'遭難者 {c["persons_on_board"]} 名全員にボートが到着。捜索を終了して帰還します')
                self._goto('RETURN')
            elif self.state == 'RESUME':
                n, e = self.resume_point
                if self._fly_to(n, e, c['patrol_alt'], c['transit_speed']):
                    self._goto('SEARCH')
            else:
                n, e = self.route[self.wp]
                if self._fly_to(n, e, c['patrol_alt'], c['patrol_speed']):
                    self.wp += 1
                    if self.wp >= len(self.route):
                        self._event('拡大方形捜索を完了。帰還します')
                        self._goto('RETURN')

        elif self.state == 'APPROACH':
            n, e = self._standoff(self.current, c['patrol_alt'], now)
            if self._fly_to(n, e, c['patrol_alt'], c['transit_speed'], yaw=self.bearing, tol=2.5):
                self._goto('DESCEND')

        elif self.state == 'DESCEND':
            n, e = self._standoff(self.current, c['inspect_alt'], now)
            if self._fly_to(n, e, c['inspect_alt'], c['patrol_speed'], yaw=self.bearing, tol=2.5):
                self.current.obs = []
                self._goto('OBSERVE')

        elif self.state == 'OBSERVE':
            tg = self.current
            n, e = self._standoff(tg, c['inspect_alt'], now)
            self._fly_to(n, e, c['inspect_alt'], 3.0, yaw=self.bearing, tol=3.0)
            strong = [o for o in tg.obs if o[2] >= c['confirm_conf']]
            if tg.obs:
                # 最新の近距離観測で推定位置を更新(以降はそこから漂流を予測)
                last = tg.obs[-1]
                tg.north, tg.east, tg.t_est = last[0], last[1], last[3]
            if len(strong) >= c['confirm_count'] and elapsed_state >= c['observe_s']:
                # 確定位置: 近距離の観測を同じ時刻に揃えて(漂流分を補正して)平均する
                ns = [o[0] + self.vn * (now - o[3]) for o in strong]
                es = [o[1] + self.ve * (now - o[3]) for o in strong]
                tg.north, tg.east, tg.t_est = sum(ns) / len(ns), sum(es) / len(es), now
                tg.status = 'rescuing'
                tg.confirm_t = now
                lat, lon = ned_to_latlon(tg.north, tg.east, c['home_lat'], c['home_lon'])
                self._event(f'{tg.id}: AI が遭難者を確定 (確信度 {max(o[2] for o in strong):.2f}) '
                            f'{lat:.6f}, {lon:.6f}')
                self._goto('OPERATOR')
            elif elapsed_state >= c['observe_timeout_s']:
                tg.status = 'rejected'
                self._event(f'{tg.id}: 近距離で確認できず、誤検出として棄却')
                self._goto('CLIMB')

        elif self.state == 'OPERATOR':
            # 現場オペレータが伝送された映像を確認して救助を決定する(ここでは一定時間後に GO)
            tg = self.current
            n, e = self._standoff(tg, c['inspect_alt'], now)
            self._fly_to(n, e, c['inspect_alt'], 3.0, yaw=self.bearing, tol=3.0)
            if elapsed_state >= c['operator_delay_s']:
                idle = [b for b in self.boards.values() if b['status'] == 'IDLE']
                if not idle:
                    if self.tick % 50 == 0:
                        self._event('待機中のボートがありません。空きを待ちます')
                    return self._publish_state() if self.tick % 2 == 0 else None
                pn, pe = tg.predict(now, self.vn, self.ve)
                b = min(idle, key=lambda b: math.hypot(b['north'] - pn, b['east'] - pe))
                tg.board, tg.dispatch_t = b['id'], now
                self.n_dispatched += 1
                self._send_lora('dispatch', tg, now)
                self._event(f'{tg.id}: オペレータが救助を決定(GO)。ボート {b["id"]} を出動')
                self._goto('ESCORT')

        elif self.state == 'ESCORT':
            # 遭難者を画像の中央に捉えたまま上空で待機し、推定位置を更新して LoRa で送り続ける
            tg = self.current
            if tg.obs:
                last = tg.obs[-1]
                tg.north, tg.east, tg.t_est = last[0], last[1], last[3]
            n, e = self._standoff(tg, c['escort_alt'], now)
            self._fly_to(n, e, c['escort_alt'], 4.0, yaw=self.bearing, tol=3.0)
            if now - self._last_lora >= c['lora_update_s']:
                self._send_lora('update', tg, now)
            b = self.boards.get(tg.board)
            if b and b['status'] == 'HOLD' and b['target'] == tg.id:
                tg.arrive_t = b['t_arrive']
                self.n_arrived += 1
                self._event(f'{tg.id}: ボート {tg.board} が到着(出動から {tg.arrive_t - tg.dispatch_t:.0f} 秒)。'
                            f'浮体として待機・発光マーカー展開')
                tg.status = 'confirmed'
                self._goto('CLIMB')

        elif self.state == 'CLIMB':
            if self._fly_to(self.pos.x, self.pos.y, c['patrol_alt'], c['patrol_speed'], tol=2.0):
                self.current = None
                self._goto('RESUME')

        elif self.state == 'RETURN':
            if self._fly_to(self.home[0], self.home[1], c['patrol_alt'], c['transit_speed']):
                self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._event('砂浜の離陸地点に到着。着陸します')
                self._goto('LAND')

        elif self.state == 'LAND':
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self._event('着陸完了。ミッション終了')
                self._write_report()
                self._goto('DONE')

        if self.tick % 2 == 0:
            self._publish_state()

    def _start_inspection(self, tg):
        if self.resume_point is None or self.state in ('SEARCH', 'TRANSIT'):
            self.resume_point = (self.pos.x, self.pos.y)
        self.current = tg
        tg.status = 'inspecting'
        pn, pe = tg.predict(time.time(), self.vn, self.ve)
        self.bearing = math.atan2(pe - self.pos.y, pn - self.pos.x)
        self._event(f'{tg.id}: 捜索を中断して確認に向かいます')
        self._goto('APPROACH')

    # ---- 出力 -------------------------------------------------------------
    def _summary(self):
        c = self.cfg
        now = time.time()
        return dict(
            scenario='sea',
            state=self.state, state_jp=STATE_JP[self.state], now=now,
            elapsed=0.0 if self.t_start is None else round(now - self.t_start, 1),
            drone=dict(north=self.pos.x, east=self.pos.y, alt=-self.pos.z),
            yaw=self.yaw, wp=self.wp, route=self.route or [],
            datum=dict(n=c['datum_north'], e=c['datum_east']),
            search_center=self.search_center,
            current=dict(v_north=self.vn, v_east=self.ve, speed=c['current_speed'],
                         to_deg=c['current_to_deg']),
            pob=c['persons_on_board'], n_dispatched=self.n_dispatched, n_arrived=self.n_arrived,
            incident_elapsed_s=c['incident_elapsed_s'], t_launch=self.t_launch,
            video_s=round(self.video_s, 1), lora=self.lora_stats,
            boards=list(self.boards.values()),
            patrol_alt=c['patrol_alt'], inspect_alt=c['inspect_alt'],
            current_target=None if self.current is None else self.current.id,
            targets=[t.as_dict(c['home_lat'], c['home_lon'], now, self.vn, self.ve)
                     for t in self.targets],
            events=self.log_events[-8:], dist_flown=round(self.dist_flown, 1),
        )

    def _publish_state(self):
        s = self._summary()
        s['trail'] = self.trail[-3000:]
        self.pub_state.publish(String(data=json.dumps(s, ensure_ascii=False)))

    def _write_report(self):
        os.makedirs(self.cfg['report_dir'], exist_ok=True)
        path = os.path.join(self.cfg['report_dir'],
                            f'mission_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
        rep = self._summary()
        cfg = {k: v for k, v in self.cfg.items() if k != 'report_dir'}
        rep.update(trail=self.trail, events=self.log_events, config=cfg, t_start=self.t_start)
        with open(path, 'w') as f:
            json.dump(rep, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f'報告書を保存: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
