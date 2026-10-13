#!/usr/bin/env python3
"""gz-transport で複数カメラの画像を1枚ずつ保存する(海面での検出テスト用)。

実行: PYTHONNOUSERSITE=1 python3 grab_frames.py OUT_DIR [WORLD CAM1 CAM2 ...]
  WORLD を省略すると海面テスト(sea_test、tests/make_sea_test_world.py の全カメラ)
"""
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image as PILImage
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

out = Path(sys.argv[1] if len(sys.argv) > 1 else 'frames')
if len(sys.argv) > 3:
    WORLD, CAMS = sys.argv[2], sys.argv[3:]
else:
    WORLD, CAMS = 'sea_test', [f'cam_{sea}_{a}m_{t}deg' for sea in ('flat', 'wave')
                               for a, t in ((18, 30), (14, 30), (10, 35))]
out.mkdir(parents=True, exist_ok=True)
got = {}
lock = threading.Lock()


def make_cb(cam):
    def cb(msg: Image):
        with lock:
            if cam in got:
                return
            arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            PILImage.fromarray(arr).save(out / f'{cam}.png')
            got[cam] = True
            print('saved', cam, flush=True)
    return cb


node = Node()
for c in CAMS:
    topic = f'/world/{WORLD}/model/{c}/link/camera_link/sensor/camera/image'
    node.subscribe(Image, topic, make_cb(c))

t0 = time.time()
while len(got) < len(CAMS) and time.time() - t0 < 120:
    time.sleep(0.5)
print('done', sorted(got))
