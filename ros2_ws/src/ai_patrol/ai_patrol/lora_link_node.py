#!/usr/bin/env python3
"""LoRa 無線リンクの模擬: ドローン → フォイルボードの座標・指令の伝送。

/rescue/lora/tx(ドローンが送る JSON)を受け取り、次の性質を与えて /rescue/lora/rx へ渡す。
  - 伝送遅延: 送信時間(エアタイム)+ 処理遅延
  - パケットロス: 一定の確率で失われる
  - 送信時間の計算: LoRa 変調(SF・帯域幅・符号化率)とペイロード長から求める(Semtech AN1200.13 の式)
また、送信数・受信数・遅延・累積送信時間を /rescue/lora/stats に出し、評価に使う。
"""
import json
import math
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def lora_airtime_s(payload_bytes, sf=7, bw_hz=125e3, cr=1, preamble=8, header=True, crc=True):
    """LoRa の 1 パケットの送信時間 [s](Semtech AN1200.13)。cr=1 は 4/5。"""
    t_sym = (2 ** sf) / bw_hz
    de = 1 if t_sym > 0.016 else 0                 # 低データレート最適化
    ih = 0 if header else 1
    n_payload = 8 + max(math.ceil((8 * payload_bytes - 4 * sf + 28 + 16 * crc - 20 * ih)
                                  / (4 * (sf - 2 * de))) * (cr + 4), 0)
    return (preamble + 4.25) * t_sym + n_payload * t_sym


class LoraLink(Node):
    def __init__(self):
        super().__init__('lora_link')
        self.declare_parameter('loss_rate', 0.03)        # パケットロス率
        self.declare_parameter('proc_delay_s', 0.02)     # 処理遅延 [s]
        self.declare_parameter('sf', 7)
        self.declare_parameter('seed', 1)
        self.loss = self.get_parameter('loss_rate').value
        self.proc = self.get_parameter('proc_delay_s').value
        self.sf = self.get_parameter('sf').value
        random.seed(self.get_parameter('seed').value)
        self.pub_rx = self.create_publisher(String, '/rescue/lora/rx', 20)
        self.pub_stats = self.create_publisher(String, '/rescue/lora/stats', 10)
        self.create_subscription(String, '/rescue/lora/tx', self._on_tx, 20)
        self.queue = []                                  # (配信時刻, データ)
        self.n_tx = self.n_rx = 0
        self.bytes = 0
        self.airtime = 0.0
        self.delays = []
        self.create_timer(0.005, self._deliver)
        self.create_timer(1.0, self._publish_stats)
        self.t0 = time.time()

    def _on_tx(self, msg):
        # 実機では座標などを数十バイトのバイナリに詰める。ここでは JSON から必要な数値だけ数える
        d = json.loads(msg.data)
        payload = 4 + 4 * sum(1 for v in d.values() if isinstance(v, (int, float)))
        air = lora_airtime_s(payload, sf=self.sf)
        self.n_tx += 1
        self.bytes += payload
        self.airtime += air
        if random.random() < self.loss:
            return
        delay = air + self.proc
        d['_sent'] = time.time()
        self.queue.append((time.time() + delay, d))

    def _deliver(self):
        now = time.time()
        ready = [q for q in self.queue if q[0] <= now]
        self.queue = [q for q in self.queue if q[0] > now]
        for _, d in ready:
            self.delays.append(now - d['_sent'])
            self.n_rx += 1
            self.pub_rx.publish(String(data=json.dumps(d)))

    def _publish_stats(self):
        el = max(1e-6, time.time() - self.t0)
        s = dict(n_tx=self.n_tx, n_rx=self.n_rx, bytes=self.bytes, airtime_s=round(self.airtime, 3),
                 duty=round(self.airtime / el, 5),
                 delay_ms_mean=round(1000 * sum(self.delays) / len(self.delays), 1) if self.delays else None,
                 delay_ms_max=round(1000 * max(self.delays), 1) if self.delays else None, sf=self.sf)
        self.pub_stats.publish(String(data=json.dumps(s)))


def main(args=None):
    rclpy.init(args=args)
    node = LoraLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
