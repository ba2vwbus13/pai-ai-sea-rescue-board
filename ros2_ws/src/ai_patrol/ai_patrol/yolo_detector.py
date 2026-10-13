"""YOLOv8 (ONNX) による物体検出。ROS に依存しない共通部品。

上空から見下ろす画像では人や車が小さく写るため、画像全体を 640x640 に
縮小して1回推論するだけでは見落としが多い。そこで画像を重なりのある
タイルに分割して各タイルを縮小せずに推論し、結果を元画像の座標に戻して
NMS で統合する(SAHI と同じ考え方の簡易実装)。
"""
from dataclasses import dataclass

import cv2
import numpy as np
import onnxruntime as ort

COCO_NAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
    'toothbrush']

# 捜索で扱うクラスと、まとめ先のラベル(海難救助: 人と船)
TARGET_CLASSES = {'person': 'person', 'boat': 'boat'}


@dataclass
class Detection:
    label: str        # 'person' / 'vehicle'
    coco: str         # 元の COCO クラス名
    conf: float
    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def center(self):
        return (0.5 * (self.x0 + self.x1), 0.5 * (self.y0 + self.y1))


class YoloDetector:
    def __init__(self, model_path, input_size=640, conf_th=0.25, iou_th=0.45, threads=4,
                 providers=('CPUExecutionProvider',)):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = threads
        provs = [('CUDAExecutionProvider', {'cudnn_conv_algo_search': 'HEURISTIC'})
                 if p == 'CUDAExecutionProvider' else p for p in providers]
        if 'CPUExecutionProvider' not in providers:
            provs.append('CPUExecutionProvider')
        self.sess = ort.InferenceSession(model_path, opts, providers=provs)
        self.input_name = self.sess.get_inputs()[0].name
        self.size = input_size
        self.conf_th = conf_th
        self.iou_th = iou_th

    # ---- 1回分の推論 ---------------------------------------------------
    def _infer(self, img):
        """img (BGR) を letterbox して推論し、(boxes_xyxy, conf, cls) を返す。"""
        h, w = img.shape[:2]
        r = self.size / max(h, w)
        nw, nh = int(round(w * r)), int(round(h * r))
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        canvas[:nh, :nw] = cv2.resize(img, (nw, nh))
        x = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run(None, {self.input_name: x})[0][0]     # (84, 8400)
        boxes, scores = out[:4].T, out[4:].T
        cls = scores.argmax(1)
        conf = scores.max(1)
        m = conf > self.conf_th
        boxes, cls, conf = boxes[m] / r, cls[m], conf[m]
        xyxy = np.column_stack([boxes[:, 0] - boxes[:, 2] / 2, boxes[:, 1] - boxes[:, 3] / 2,
                                boxes[:, 0] + boxes[:, 2] / 2, boxes[:, 1] + boxes[:, 3] / 2])
        return xyxy, conf, cls

    # ---- タイル推論 ----------------------------------------------------
    def detect(self, img, tiles=(1, 1), overlap=0.2):
        """tiles=(列, 行) に分割して推論。(1,1) なら通常の全体推論。"""
        h, w = img.shape[:2]
        nx, ny = tiles
        tw = int(np.ceil(w / (nx - (nx - 1) * overlap)))
        th = int(np.ceil(h / (ny - (ny - 1) * overlap)))
        all_b, all_c, all_k = [], [], []
        for iy in range(ny):
            for ix in range(nx):
                x0 = 0 if nx == 1 else int(round(ix * (w - tw) / (nx - 1)))
                y0 = 0 if ny == 1 else int(round(iy * (h - th) / (ny - 1)))
                b, c, k = self._infer(img[y0:y0 + th, x0:x0 + tw])
                b[:, [0, 2]] += x0
                b[:, [1, 3]] += y0
                all_b.append(b)
                all_c.append(c)
                all_k.append(k)
        boxes = np.concatenate(all_b)
        conf = np.concatenate(all_c)
        cls = np.concatenate(all_k)

        dets = []
        if len(boxes) == 0:
            return dets
        # クラスごとに NMS(タイルの重なりで生じた重複もここで消える)
        for k in np.unique(cls):
            name = COCO_NAMES[int(k)]
            if name not in TARGET_CLASSES:
                continue
            m = cls == k
            b, c = boxes[m], conf[m]
            xywh = np.column_stack([b[:, 0], b[:, 1], b[:, 2] - b[:, 0], b[:, 3] - b[:, 1]])
            idx = cv2.dnn.NMSBoxes(xywh.tolist(), c.tolist(), self.conf_th, self.iou_th)
            for i in np.array(idx).flatten():
                dets.append(Detection(TARGET_CLASSES[name], name, float(c[i]),
                                      *[float(v) for v in b[i]]))
        return dets


def draw_detections(img, dets, color_map=None):
    color_map = color_map or {'person': (0, 0, 255), 'boat': (0, 200, 255)}
    for d in dets:
        col = color_map.get(d.label, (0, 255, 255))
        p0, p1 = (int(d.x0), int(d.y0)), (int(d.x1), int(d.y1))
        cv2.rectangle(img, p0, p1, col, 2)
        cv2.putText(img, f'{d.label} {d.conf:.2f}', (p0[0], max(12, p0[1] - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
    return img
