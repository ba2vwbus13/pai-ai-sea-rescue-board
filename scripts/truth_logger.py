#!/usr/bin/env python3
"""評価用: 遭難者とフォイルボードの「本当の位置」を Gazebo から記録する(AI 側には渡さない)。

/world/<world>/dynamic_pose/info を購読し、0.5 秒ごとに person_* と board_* の位置を
JSON Lines で保存する。時刻は壁時計(time.time())で、ミッションの報告書と照合できる。

実行(システムの python3): PYTHONNOUSERSITE=1 python3 truth_logger.py OUT.jsonl [WORLD]
"""
import json
import sys
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

out_path = sys.argv[1]
world = sys.argv[2] if len(sys.argv) > 2 else 'sea_board'
latest = {}
lock = threading.Lock()


def on_poses(msg: Pose_V):
    with lock:
        for p in msg.pose:
            if p.name.startswith(('person_', 'board_', 'beacon_')):
                # NED に合わせて north = y, east = x で保存する
                latest[p.name] = (round(p.position.y, 3), round(p.position.x, 3))


node = Node()
node.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', on_poses)
with open(out_path, 'a') as f:
    while True:
        time.sleep(0.5)
        with lock:
            if latest:
                f.write(json.dumps(dict(t=time.time(), poses=latest)) + '\n')
                f.flush()
