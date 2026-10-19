#!/usr/bin/env python3
"""海難救助(ドローン + フォイルボード)ミッションの評価。

報告書 JSON を、Gazebo から記録した本当の位置(truth_*.jsonl)と照合し、
数値目標に対応する指標を出す。
  - 発見数・誤った確定、確定時点の位置誤差
  - 事案発生からボード到着までの時間(目標 5 分以内)
  - ボード到着時の遭難者との距離(目標: 停止距離 2 m に対する誤差 2 m 以内)
  - LoRa の受信成功率・遅延・送信時間の割合(デューティ)
  - 映像伝送時間と、常時伝送に対する削減率
出力: 標準出力の表と、結果地図 PNG(報告書と同じ名前)。

実行: python3 evaluate_mission.py results/mission_XXXX.json [--truth results/truth_XXXX.jsonl]
"""
import argparse
import bisect
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
for name in ('Noto Sans CJK JP', 'IPAexGothic', 'Hiragino Sans'):
    if any(name in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = name
        break


class Truth:
    def __init__(self, path):
        self.t, self.poses = [], []
        for line in Path(path).read_text().splitlines():
            d = json.loads(line)
            self.t.append(d['t'])
            self.poses.append(d['poses'])

    def at(self, name, t):
        i = min(max(bisect.bisect_left(self.t, t), 1), len(self.t) - 1)
        t0, t1 = self.t[i - 1], self.t[i]
        p0, p1 = self.poses[i - 1].get(name), self.poses[i].get(name)
        if p0 is None or p1 is None:
            return p1 or p0
        a = 0.0 if t1 == t0 else min(1.0, max(0.0, (t - t0) / (t1 - t0)))
        return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]))

    def names(self, prefix):
        return sorted({n for p in self.poses for n in p if n.startswith(prefix)})


def find_truth(report_path, rep):
    best = None
    for c in sorted(Path(report_path).parent.glob('truth_*.jsonl')):
        if json.loads(c.open().readline())['t'] <= rep['t_start']:
            best = c
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('report')
    ap.add_argument('--truth')
    ap.add_argument('--match', type=float, default=5.0)
    a = ap.parse_args()

    rep = json.loads(Path(a.report).read_text())
    truth = Truth(a.truth or find_truth(a.report, rep))
    persons = truth.names('person_')
    targets = [t for t in rep['targets'] if t['status'] in ('confirmed', 'rescuing')]
    rejected = [t for t in rep['targets'] if t['status'] == 'rejected']

    # 事案発生の時刻: ドローンの出動(アーム)の incident_elapsed_s 秒前とする。
    # シミュレータの起動待ちの時間は含めない(その間に遭難者が流された分は、捜索側に不利に働くだけ)
    t_incident = rep['t_start'] - rep['incident_elapsed_s']

    # AI の推定位置(est_*、最後に更新した時刻 est_t)と、同じ時刻の本当の位置を対応づける
    pairs, used_t, used_p = [], set(), set()
    errs = {}
    for i, t in enumerate(targets):
        tc = t['confirm_t']
        best = min(persons, key=lambda p: math.hypot(*(x - y for x, y in zip(
            truth.at(p, tc), (t['est_north'], t['est_east']))))) if tc else None
        if best is None:
            continue
        tn, te = truth.at(best, t['est_t'])
        d = math.hypot(t['est_north'] - tn, t['est_east'] - te)
        if d <= a.match and best not in used_p:
            pairs.append((i, best))
            used_p.add(best)
            errs[best] = d
    tp, fp = len(pairs), len(targets) - len(pairs)

    print(f'遭難者(正解)         : {len(persons)} 名')
    print(f'確定 (うち正解 / 誤り) : {len(targets)} ({tp} / {fp})   棄却した候補: {len(rejected)}')
    print(f'見逃し                : {[p for p in persons if p not in used_p]}')
    rows = []
    for i, p in sorted(pairs, key=lambda x: targets[x[0]]['confirm_t']):
        t = targets[i]
        row = dict(person=p, id=t['id'], board=t['board'],
                   found=t['first_t'] - t_incident, confirm=t['confirm_t'] - t_incident,
                   err=errs[p])
        if t.get('arrive_t'):
            ta = t['arrive_t'] + 1.0                       # 停止して落ち着いた時点
            bp = truth.at(f'board_{t["board"]}', ta)
            pp = truth.at(p, ta)
            row.update(arrive=t['arrive_t'] - t_incident, transit=t['arrive_t'] - t['dispatch_t'],
                       gap=math.hypot(bp[0] - pp[0], bp[1] - pp[1]))
        rows.append(row)
    print('  遭難者    ID  ボード  発見   確定   到着(事案発生から)  航行時間  ボードとの距離  推定誤差')
    for r in rows:
        if 'arrive' in r:
            print(f'  {r["person"]:9s} {r["id"]:3s} {r["board"]:4s}  {r["found"]:5.0f}s {r["confirm"]:5.0f}s '
                  f'{r["arrive"]:6.0f}s ({r["arrive"] / 60:4.1f}分)  {r["transit"]:5.0f}s  '
                  f'{r["gap"]:5.2f} m        {r["err"]:.2f} m')
    arr = [r for r in rows if 'arrive' in r]
    if arr:
        print(f'事案発生から全員にボード到着: {max(r["arrive"] for r in arr):.0f} s '
              f'({max(r["arrive"] for r in arr) / 60:.1f} 分、目標 5 分以内: '
              f'{sum(r["arrive"] <= 300 for r in arr)}/{len(arr)} 名)')
        gaps = [r['gap'] for r in arr]
        print(f'ボードと遭難者の距離 平均 {sum(gaps) / len(gaps):.2f} m / 最大 {max(gaps):.2f} m'
              f'(停止距離 2 m に対する誤差 最大 {max(abs(g - 2.0) for g in gaps):.2f} m)')
    lo = rep.get('lora')
    if lo and lo['n_tx']:
        print(f'LoRa: 受信 {lo["n_rx"]}/{lo["n_tx"]} ({100 * lo["n_rx"] / lo["n_tx"]:.1f}%)  '
              f'遅延 平均 {lo["delay_ms_mean"]} ms / 最大 {lo["delay_ms_max"]} ms  '
              f'送信時間の割合 {100 * lo["duty"]:.2f}%  (SF{lo["sf"]})')
    if rep['elapsed']:
        print(f'映像伝送: {rep["video_s"]:.0f} s / 飛行 {rep["elapsed"]:.0f} s → 常時伝送比 '
              f'{100 * (1 - rep["video_s"] / rep["elapsed"]):.0f}% 削減')
    print(f'飛行時間 {rep["elapsed"]:.0f} s   飛行距離 {rep["dist_flown"]:.0f} m')

    # ---- 地図 --------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    ax.axhspan(-3, 30, color='#e8dcb5', zorder=0)
    ax.set_facecolor('#d6e8f5')
    tr = rep['trail']
    ax.plot([p[1] for p in tr], [p[0] for p in tr], c='orange', lw=1.2, label='ドローン')
    for k, p in enumerate(persons):
        seg = [truth.at(p, t) for t in truth.t[::4]]
        ax.plot([q[1] for q in seg], [q[0] for q in seg], c='blue', lw=1.0, alpha=0.6,
                label='遭難者の漂流(正解)' if k == 0 else None)
        ax.annotate(p.replace('person_', ''), (seg[-1][1], seg[-1][0]), xytext=(4, -10),
                    textcoords='offset points', color='blue', fontsize=8)
    for k, b in enumerate(truth.names('board_')):
        seg = [truth.at(b, t) for t in truth.t[::2]]
        ax.plot([q[1] for q in seg], [q[0] for q in seg], c='green', lw=1.6,
                label='フォイルボード' if k == 0 else None)
        ax.plot(seg[-1][1], seg[-1][0], 'D', c='green', ms=6)
        ax.annotate(b.replace('board_', ''), (seg[-1][1], seg[-1][0]), xytext=(5, 4),
                    textcoords='offset points', color='green', fontsize=8)
    ax.scatter([t['est_east'] for t in targets], [t['est_north'] for t in targets], c='red', s=30,
               zorder=5, label='AI の推定位置(最終)')
    if rejected:
        ax.scatter([t['est_east'] for t in rejected], [t['est_north'] for t in rejected],
                   marker='x', c='gray', s=40, label='誤検出として棄却')
    ax.plot(rep['datum']['e'], rep['datum']['n'], 'X', c='#d4a000', ms=11, label='最後の目撃地点')
    if rep.get('search_center'):
        sc = rep['search_center']
        ax.annotate('', (sc[1], sc[0]), (rep['datum']['e'], rep['datum']['n']),
                    arrowprops=dict(arrowstyle='->', color='#d4a000', lw=1.8))
    ax.plot(0, 0, 's', c='k', ms=7, label='ドローン離陸地点')
    ax.set_xlabel('東 [m]')
    ax.set_ylabel('北 [m]')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    title = f'{len(persons)} 名中 {len(arr)} 名にボード到着'
    if arr:
        title += f'(事案発生から {max(r["arrive"] for r in arr) / 60:.1f} 分)'
    ax.set_title(title)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=3, fontsize=8)
    fig.tight_layout()
    out = Path(a.report).with_suffix('.png')
    fig.savefig(out, dpi=150)
    print('地図:', out)


if __name__ == '__main__':
    main()
