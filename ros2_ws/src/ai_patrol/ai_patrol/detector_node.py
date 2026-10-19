#!/usr/bin/env python3
"""AI 検出ノード: 下向きカメラ映像を YOLO で解析し、人の地上位置を出力する。

入力:
  カメラ画像 (sensor_msgs/Image, ros_gz_image のブリッジ経由。前方斜め下 30° のカメラ)
  機体の姿勢・位置 (/fmu/out/vehicle_attitude, /fmu/out/vehicle_local_position_v1)
出力:
  /ai_patrol/detections      std_msgs/String (JSON)  検出結果と推定地上位置
  /ai_patrol/annotated_image sensor_msgs/Image       枠を描いた画像(表示用)

工夫:
  - 上空からは人が十数画素にしか写らないため、高度に応じて画像をタイル分割して推論
    (高い所では 3x3、低い所では 2x2 で十分なので速度を優先)
  - カメラの傾きと機体の姿勢を考慮して、画素の視線と地面の交点から人の位置を推定
  - 画像を受け取った瞬間の姿勢・位置を保存し、推論後にその姿勢で地上位置を計算
    (推論に約1秒かかっても、その間に機体が動いた分のずれが生じない)
  - GPU(onnxruntime CUDA)が使えなければ自動で CPU + 軽量モデルに切り替える
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

from .geo import pixel_to_ground, yaw_from_quat
from .yolo_detector import YoloDetector, draw_detections


class DetectorNode(Node):
    def __init__(self):
        super().__init__('ai_detector')
        self.declare_parameter('image_topic',
                               '/world/sea_board/model/x500_sar_cam_0/link/camera_link/sensor/camera/image')
        self.declare_parameter('model_path', '')
        self.declare_parameter('cpu_model_path', '')
        self.declare_parameter('conf_threshold', 0.30)
        self.declare_parameter('detect_rate_hz', 1.0)
        self.declare_parameter('hfov', 1.74)
        self.declare_parameter('cam_tilt_deg', 30.0)
        self.declare_parameter('max_range', 40.0)
        self.declare_parameter('tile_alt_threshold', 12.0)
        self.declare_parameter('min_alt', 4.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.hfov = p('hfov')
        self.cam_tilt = math.radians(p('cam_tilt_deg'))
        self.max_range = p('max_range')
        self.tile_alt_th = p('tile_alt_threshold')
        self.min_alt = p('min_alt')
        self.detector = self._load_detector(p('model_path'), p('cpu_model_path'), p('conf_threshold'))

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        img_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, p('image_topic'), self._on_image, img_qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._on_att, px4_qos)
        for t in ('/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_local_position'):
            self.create_subscription(VehicleLocalPosition, t, self._on_pos, px4_qos)

        self.pub_det = self.create_publisher(String, '/ai_patrol/detections', 10)
        self.pub_img = self.create_publisher(Image, '/ai_patrol/annotated_image', 1)

        self.q = None
        self.pos = None
        self.latest = None        # (Image, q, pos) 画像受信時点の姿勢・位置
        self.n_frames = 0
        self.create_timer(1.0 / p('detect_rate_hz'), self._on_timer)
        self.get_logger().info(f'AI 検出ノード開始: {self.backend}')

    def _load_detector(self, model, cpu_model, conf):
        import onnxruntime as ort
        if model and 'CUDAExecutionProvider' in ort.get_available_providers():
            try:
                det = YoloDetector(model, conf_th=conf, providers=['CUDAExecutionProvider'])
                if det.sess.get_providers()[0] == 'CUDAExecutionProvider':
                    self.backend = f'GPU / {os.path.basename(model)}'
                    return det
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'GPU での読み込みに失敗: {e}')
        path = cpu_model or model
        self.backend = f'CPU / {os.path.basename(path)}'
        return YoloDetector(path, conf_th=conf, providers=['CPUExecutionProvider'], threads=8)

    def _on_att(self, msg):
        self.q = [float(v) for v in msg.q]

    def _on_pos(self, msg):
        self.pos = (msg.x, msg.y, msg.z)

    def _on_image(self, msg):
        if self.q is not None and self.pos is not None:
            # 写真を受け取った瞬間の時刻・姿勢・位置をいっしょに覚えておく
            self.latest = (msg, list(self.q), tuple(self.pos), time.time())

    def _on_timer(self):
        if self.latest is None:
            return
        msg, q, pos, t_img = self.latest
        self.latest = None
        alt = -pos[2]

        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
        if msg.encoding == 'rgb8':
            img = img[:, :, ::-1]
        img = np.ascontiguousarray(img)

        dets, tiles, ms = [], None, 0.0
        if alt >= self.min_alt:
            tiles = (3, 3) if alt >= self.tile_alt_th else (2, 2)
            t0 = time.time()
            dets = self.detector.detect(img, tiles=tiles)
            ms = (time.time() - t0) * 1000.0

        out = []
        for d in dets:
            u, v = d.center
            g = pixel_to_ground(u, v, msg.width, msg.height, self.hfov, q, pos, self.cam_tilt)
            # 遠方(画像上端付近)は位置の誤差が大きいので使わない
            if g is None or math.hypot(g[0] - pos[0], g[1] - pos[1]) > self.max_range:
                continue
            out.append(dict(label=d.label, coco=d.coco, conf=round(d.conf, 3),
                            u=round(u, 1), v=round(v, 1),
                            box=[round(d.x0), round(d.y0), round(d.x1), round(d.y1)],
                            north=round(g[0], 2), east=round(g[1], 2)))
        self.n_frames += 1
        # stamp は「人がその位置にいた時刻」= 写真を受け取った時刻(推論が終わった時刻ではない)
        payload = dict(stamp=t_img, frame=self.n_frames, backend=self.backend,
                       infer_ms=round(ms), tiles=tiles, cam_tilt=self.cam_tilt,
                       drone=dict(north=pos[0], east=pos[1], alt=alt, yaw=yaw_from_quat(q)),
                       detections=out)
        self.pub_det.publish(String(data=json.dumps(payload)))

        # 表示用の画像(縮小して送る)。地上ではカメラが地面に近すぎて空しか写らないので送らない
        if alt < 1.0:
            return
        vis = draw_detections(img.copy(), dets)
        if tiles and tiles != (1, 1):
            h, w = vis.shape[:2]
            for i in range(1, tiles[0]):
                vis[:, int(w * i / tiles[0])] //= 2
            for j in range(1, tiles[1]):
                vis[int(h * j / tiles[1]), :] //= 2
        vis = vis[::2, ::2].copy()
        im = Image()
        im.header = msg.header
        im.height, im.width = vis.shape[:2]
        im.encoding = 'bgr8'
        im.step = im.width * 3
        im.data = vis.tobytes()
        self.pub_img.publish(im)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
