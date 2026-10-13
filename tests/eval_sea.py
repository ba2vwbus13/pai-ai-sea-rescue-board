#!/usr/bin/env python3
"""海面テストの画像で、人(正解 4)と船(正解 1)の検出を数える。

実行: python3 eval_sea.py FRAME_DIR MODEL.onnx
"""
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/ai_patrol'))
from ai_patrol.yolo_detector import YoloDetector, draw_detections  # noqa: E402

frame_dir = Path(sys.argv[1])
det = YoloDetector(sys.argv[2], conf_th=0.25,
                   providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
out = frame_dir / 'annotated'
out.mkdir(exist_ok=True)
print(f'{"条件":>20} {"人":>3} {"船":>3}  人の確信度')
for p in sorted(frame_dir.glob('cam_*.png')):
    img = cv2.imread(str(p))
    ds = det.detect(img, tiles=(3, 3))
    ps = sorted((d.conf for d in ds if d.label == 'person'), reverse=True)
    bs = [d for d in ds if d.label == 'boat']
    print(f'{p.stem[4:]:>20} {len(ps):>3} {len(bs):>3}  ' + ' '.join(f'{c:.2f}' for c in ps))
    cv2.imwrite(str(out / f'{p.stem}.jpg'), draw_detections(img.copy(), ds))
