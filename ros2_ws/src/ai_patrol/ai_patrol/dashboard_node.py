#!/usr/bin/env python3
"""表示ノード: カメラ映像(AI の検出枠つき)と捜索マップを1画面にまとめて表示する。

デモ動画の撮影用(1920x1080)。ミッション完了時には最終画面を PNG で保存する。
  左: 前方斜め下カメラ + YOLO の検出枠(タイル分割の境界も薄く表示)
  右上: Gazebo の 3D 画面を動画編集で重ねる枠(scripts/make_video.sh)
  右下: 捜索マップ(捜索範囲・予定ルート・飛行軌跡・候補/確定/棄却の位置)と状態パネル
  左下: 出来事のログ
"""
import json
import math
import os
import subprocess

import cv2
import numpy as np
import rclpy
import rclpy.executors
from PIL import Image as PILImage, ImageDraw, ImageFont
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .geo import pixel_to_ground, yaw_quat

W, H = 1920, 1080
CAM = (20, 70, 1000, 750)           # x, y, w, h
VIEW3D = (1040, 70, 860, 484)       # Gazebo の 3D 画面を重ねる枠(16:9)
MAP = (1040, 570, 420, 420)
PANEL = (1475, 570, 425, 490)
LOG = (20, 835, 1000, 225)
MAP_N = (-215.0, 15.0)              # 地図に表示する北方向の範囲 [m](砂浜〜沖合)
MAP_E = (-130.0, 100.0)

COL = dict(bg=(30, 30, 34), panel=(48, 48, 54), text=(235, 235, 235), sub=(170, 170, 175),
           area=(90, 160, 90), route=(120, 120, 120), trail=(255, 200, 60),
           pending=(255, 255, 120), inspecting=(255, 200, 0), confirmed=(60, 60, 255),
           rejected=(140, 140, 140), drone=(255, 255, 255), sea=(70, 45, 20), sand=(150, 190, 215),
           datum=(0, 215, 255), ring=(20, 110, 255), rescuing=(60, 60, 255), board=(90, 230, 90),
           board_hold=(0, 255, 0))
STATUS_JP = dict(pending='候補', inspecting='確認中', rescuing='救助中', confirmed='救助済', rejected='棄却')


def find_font():
    try:
        p = subprocess.run(['fc-match', '-f', '%{file}', 'Noto Sans CJK JP:bold'],
                           capture_output=True, text=True, timeout=3).stdout.strip()
        if p and os.path.exists(p):
            return p
    except Exception:  # noqa: BLE001
        pass
    return None


class FontCache(dict):
    """文字サイズごとのフォント。初めて使うサイズはその場で読み込む。"""

    def __init__(self, path):
        super().__init__()
        self.path = path

    def __missing__(self, size):
        f = ImageFont.truetype(self.path, size) if self.path else ImageFont.load_default()
        self[size] = f
        return f


class Dashboard(Node):
    def __init__(self):
        super().__init__('ai_dashboard')
        self.declare_parameter('show_window', True)
        self.declare_parameter('report_dir', os.path.expanduser('~/work/pai_final/results'))
        self.declare_parameter('fullscreen', False)
        self.declare_parameter('window_x', -1)
        self.declare_parameter('window_y', -1)
        self.show = self.get_parameter('show_window').value
        self.report_dir = self.get_parameter('report_dir').value
        fp = find_font()
        self.font = FontCache(fp)
        self.cam = None
        self.state = None
        self.det = None
        self.saved = False
        self.create_subscription(Image, '/ai_patrol/annotated_image', self._on_img, 1)
        self.create_subscription(String, '/ai_patrol/mission_state', self._on_state, 10)
        self.create_subscription(String, '/ai_patrol/detections', self._on_det, 10)
        self.create_subscription(String, '/rescue/boards', self._on_boards, 10)
        self.boards = None
        self.pub = self.create_publisher(Image, '/ai_patrol/dashboard', 1)
        self.create_timer(0.2, self._render)
        if self.show:
            cv2.namedWindow('AI Search Drone', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('AI Search Drone', W, H)
            wx, wy = self.get_parameter('window_x').value, self.get_parameter('window_y').value
            if wx >= 0:
                cv2.moveWindow('AI Search Drone', wx, wy)    # 録画するモニタへ移す
            if self.get_parameter('fullscreen').value:
                cv2.setWindowProperty('AI Search Drone', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def _on_img(self, msg):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        self.cam = a[:, :, ::-1] if msg.encoding == 'rgb8' else a

    def _on_state(self, msg):
        self.state = json.loads(msg.data)

    def _on_boards(self, msg):
        self.boards = json.loads(msg.data)

    def _on_det(self, msg):
        self.det = json.loads(msg.data)

    # ---- 地図座標 -----------------------------------------------------------
    @staticmethod
    def m2px(n, e):
        x0, y0, w, h = MAP
        x = x0 + (e - MAP_E[0]) / (MAP_E[1] - MAP_E[0]) * w
        y = y0 + h - (n - MAP_N[0]) / (MAP_N[1] - MAP_N[0]) * h
        return int(round(x)), int(round(y))

    def _draw_map(self, img, s):
        x0, y0, w, h = MAP
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), COL['sea'], -1)
        _, sy = self.m2px(-3.0, 0.0)                     # 汀線より北は砂浜
        cv2.rectangle(img, (x0, y0), (x0 + w, sy), COL['sand'], -1)
        # 通報地点 → 漂流を補正した捜索基点
        dn, de = s['datum']['n'], s['datum']['e']
        pd = self.m2px(dn, de)
        cv2.drawMarker(img, pd, COL['datum'], cv2.MARKER_TILTED_CROSS, 16, 2)
        if s.get('search_center'):
            pc = self.m2px(*s['search_center'])
            cv2.arrowedLine(img, pd, pc, COL['datum'], 2, cv2.LINE_AA, tipLength=0.12)
        route = s['route']
        for i in range(len(route) - 1):
            col = (80, 80, 80) if i < s['wp'] - 1 else COL['route']
            cv2.line(img, self.m2px(*route[i]), self.m2px(*route[i + 1]), col, 1, cv2.LINE_AA)
        tr = s.get('trail', [])
        if len(tr) > 1:
            pts = np.array([self.m2px(n, e) for n, e in tr], np.int32)
            cv2.polylines(img, [pts], False, COL['trail'], 2, cv2.LINE_AA)
        hx, hy = self.m2px(0, 0)
        cv2.rectangle(img, (hx - 7, hy - 7), (hx + 7, hy + 7), (60, 60, 60), 2)
        tpos = {t['id']: (t['north'], t['east']) for t in s['targets']}
        for b in (self.boards or {}).get('boards', []):   # フォイルボード(航跡・向かう先・現在位置)
            col = COL['board_hold'] if b['status'] == 'HOLD' else COL['board']
            if len(b['trail']) > 1:
                pts = np.array([self.m2px(n, e) for n, e in b['trail']], np.int32)
                cv2.polylines(img, [pts], False, (60, 160, 60), 2, cv2.LINE_AA)
            bp = self.m2px(b['north'], b['east'])
            if b['target'] in tpos and b['status'] in ('TRANSIT', 'APPROACH'):
                cv2.line(img, bp, self.m2px(*tpos[b['target']]), (60, 160, 60), 1, cv2.LINE_AA)
            h = math.radians(b['heading'])
            pts = [(bp[0] + r * math.sin(h + a), bp[1] - r * math.cos(h + a))
                   for a, r in ((0, 10), (2.4, 7), (math.pi, 4), (-2.4, 7))]
            cv2.fillPoly(img, [np.array(pts, np.int32)], col, cv2.LINE_AA)
            cv2.putText(img, b['id'], (bp[0] + 9, bp[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1,
                        cv2.LINE_AA)
        for t in s['targets']:
            p = self.m2px(t['north'], t['east'])
            col = COL[t['status']]
            if t['status'] == 'rejected':
                cv2.drawMarker(img, p, col, cv2.MARKER_TILTED_CROSS, 14, 2)
            elif t['status'] in ('confirmed', 'rescuing'):
                cv2.circle(img, p, 9, col, -1, cv2.LINE_AA)
                cv2.circle(img, p, 12, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(img, t['id'], (p[0] + 12, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1, cv2.LINE_AA)
            else:
                cv2.circle(img, p, 9, col, 2, cv2.LINE_AA)
        d = s['drone']
        px, py = self.m2px(d['north'], d['east'])
        yaw = s['yaw'] if s['yaw'] is not None and not math.isnan(s['yaw']) else 0.0
        tri = []
        for ang, r in ((0, 12), (2.5, 8), (-2.5, 8)):
            tri.append((px + r * math.sin(yaw + ang), py - r * math.cos(yaw + ang)))
        cv2.fillPoly(img, [np.array(tri, np.int32)], COL['drone'], cv2.LINE_AA)
        # 現在のカメラ撮影範囲(斜めカメラなので台形。遠方は max 40m で打ち切り)
        tilt = (self.det or {}).get('cam_tilt', math.radians(30))
        corners = []
        for u, v in ((0, 0), (1280, 0), (1280, 960), (0, 960)):
            g = pixel_to_ground(u, v, 1280, 960, 1.74, yaw_quat(yaw),
                                (d['north'], d['east'], -max(d['alt'], 0.5)), tilt)
            if g is None:
                continue
            dn, de = g[0] - d['north'], g[1] - d['east']
            k = min(1.0, 40.0 / max(1e-6, math.hypot(dn, de)))
            corners.append(self.m2px(d['north'] + dn * k, d['east'] + de * k))
        if len(corners) >= 3:
            cv2.polylines(img, [np.array(corners, np.int32)], True, (255, 255, 255), 1, cv2.LINE_AA)

    def _render(self):
        img = np.full((H, W, 3), COL['bg'], np.uint8)
        x, y, w, h = CAM
        if self.cam is not None:
            img[y:y + h, x:x + w] = cv2.resize(self.cam, (w, h))
        else:
            cv2.rectangle(img, (x, y), (x + w, y + h), COL['panel'], -1)
        s = self.state
        if s is not None:
            # 地図は別の画像に描き、地図の枠の部分だけを貼る(枠外へのはみ出しを防ぐ)
            canvas = img.copy()
            self._draw_map(canvas, s)
            mx, my, mw, mh = MAP
            img[my:my + mh, mx:mx + mw] = canvas[my:my + mh, mx:mx + mw]
        px, py, pw, ph = PANEL
        cv2.rectangle(img, (px, py), (px + pw, py + ph), COL['panel'], -1)
        lx0, ly0, lw, lh = LOG
        cv2.rectangle(img, (lx0, ly0), (lx0 + lw, ly0 + lh), COL['panel'], -1)
        vx, vy, vw, vh = VIEW3D
        cv2.rectangle(img, (vx, vy), (vx + vw, vy + vh), (60, 60, 66), -1)

        # 文字は PIL で描く(日本語のため)
        pil = PILImage.fromarray(img[:, :, ::-1])
        dr = ImageDraw.Draw(pil)
        f = self.font
        rgb = lambda c: (c[2], c[1], c[0])  # noqa: E731
        dr.text((20, 18), 'AI 海難救助システム ― ドローン捜索 × 自律フォイルボード(名護市沖 / PX4 + Gazebo + ROS 2 + YOLO)',
                font=f[28], fill=rgb(COL['text']))
        dr.text((CAM[0] + 8, CAM[1] + 6), '前方斜め下 30° カメラ + YOLO11 (タイル分割推論)', font=f[18],
                fill=(255, 255, 255), stroke_width=2, stroke_fill=(0, 0, 0))
        dr.text((vx + vw // 2 - 90, vy + vh // 2 - 12), 'Gazebo 3D ビュー', font=f[22], fill=rgb(COL['sub']))
        dr.text((MAP[0] + 8, MAP[1] + 6), '捜索マップ(北が上・上端は砂浜)', font=f[18], fill=(60, 60, 60))
        lx, ly = MAP[0] + 4, MAP[1] + MAP[3] + 8
        for mark, key, label in (('●', 'confirmed', '遭難者'), ('×', 'rejected', '棄却'),
                                 ('◆', 'board', 'ボード'), ('×', 'datum', '目撃地点'),
                                 ('━', 'trail', 'ドローン')):
            dr.text((lx, ly), mark, font=f[16], fill=rgb(COL[key]))
            dr.text((lx + 18, ly), label, font=f[16], fill=rgb(COL['text']))
            lx += 50 + 16 * len(label)

        if s is not None:
            d = s['drone']
            n_conf = sum(t['status'] == 'confirmed' for t in s['targets'])
            n_rej = sum(t['status'] == 'rejected' for t in s['targets'])
            m, sec = divmod(int(s['elapsed']), 60)
            busy = s['state'] in ('APPROACH', 'DESCEND', 'OBSERVE', 'OPERATOR', 'ESCORT')
            cur = s['current']
            inc = s['elapsed'] + s['incident_elapsed_s']        # 出動(アーム)の 2 分前が事案発生
            im, isec = divmod(int(inc), 60)
            lines = [
                (f'状態: {s["state_jp"]}', 26, COL['pending'] if busy else COL['text']),
                (f'事案発生から {im:02d}:{isec:02d}(目標 5:00)', 20, COL['text']),
                (f'遭難者 {s["pob"]} 名中 {n_conf} 名にボード到着', 26, COL['confirmed'] if n_conf else COL['text']),
                (f'ボード発進 {s["n_dispatched"]} 台 / 誤検出の棄却 {n_rej} 件', 16, COL['text']),
            ]
            for b in (self.boards or {}).get('boards', []):
                extra = ''
                if b['status'] in ('TRANSIT', 'APPROACH'):
                    extra = f' {b["speed_kmh"]:.0f}km/h{" 離水" if b["foiling"] else ""} 残り{b["dist_go"]:.0f}m'
                col = COL['board_hold'] if b['status'] == 'HOLD' else (COL['board'] if b['target'] else COL['sub'])
                lines.append((f'{b["id"]} {b["status_jp"]}{extra}', 16, col))
            lo = s.get('lora')
            if lo and lo['n_tx']:
                lines.append((f'LoRa 受信 {lo["n_rx"]}/{lo["n_tx"]} 遅延 {lo["delay_ms_mean"]}ms', 16, COL['sub']))
            if s['elapsed'] > 0:
                red = 100.0 * (1.0 - s['video_s'] / s['elapsed'])
                lines.append((f'映像伝送 {s["video_s"]:.0f}秒のみ(常時比 {red:.0f}%減)', 16, COL['sub']))
            if self.det:
                tl = self.det.get('tiles')
                tiles = f'{tl[0]}x{tl[1]} 分割' if tl else '停止中'
                lines.append((f'AI: {self.det["backend"]}  {tiles}  {self.det["infer_ms"]} ms/枚', 16, COL['sub']))
            yy = py + 12
            for text, size, col in lines:
                dr.text((px + 14, yy), text, font=f[size], fill=rgb(col))
                yy += size + 13
            for i, ev in enumerate(s['events'][-7:]):
                em, es = divmod(int(ev['t']), 60)
                dr.text((lx0 + 12, ly0 + 8 + i * 30), f'[{em:02d}:{es:02d}] {ev["text"]}', font=f[18],
                        fill=rgb(COL['text']))
        img = np.array(pil)[:, :, ::-1].copy()

        if self.show:
            cv2.imshow('AI Search Drone', img)
            cv2.waitKey(1)
        if not rclpy.ok():
            return
        out = Image()
        out.height, out.width, out.encoding, out.step = H, W, 'bgr8', W * 3
        out.data = img.tobytes()
        self.pub.publish(out)
        if s is not None and s['state'] == 'DONE' and not self.saved:
            os.makedirs(self.report_dir, exist_ok=True)
            cv2.imwrite(os.path.join(self.report_dir, 'final_dashboard.png'), img)
            self.saved = True
            self.get_logger().info('最終画面を保存しました')


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
