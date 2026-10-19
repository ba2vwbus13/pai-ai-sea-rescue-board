#!/usr/bin/env python3
"""動画用のタイトル画面・まとめ画面・倍速表示を PNG で作る(ffmpeg に drawtext が無いため)。

実行: python3 make_cards.py OUT_DIR REPORT.json [MAP.png] [SPEED]
まとめ画面の数値は tests/evaluate_mission.py と同じ照合(本当の位置の記録)から計算する。
"""
import json
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
out = Path(sys.argv[1])
rep_path = Path(sys.argv[2])
rep = json.loads(rep_path.read_text())
map_png = Path(sys.argv[3]) if len(sys.argv) > 3 else None
speed = sys.argv[4] if len(sys.argv) > 4 else '2'
speed = speed.rstrip('0').rstrip('.') if '.' in speed else speed
out.mkdir(parents=True, exist_ok=True)
fp = subprocess.run(['fc-match', '-f', '%{file}', 'Noto Sans CJK JP:bold'],
                    capture_output=True, text=True).stdout.strip()
F = {s: ImageFont.truetype(fp, s) for s in (28, 36, 44, 64, 84)}
BG, TX, SUB, ACC = (16, 30, 48), (240, 240, 240), (170, 185, 200), (255, 140, 60)

# 評価(ボードとの距離など)は evaluate_mission.py の出力をそのまま使う
ev = subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / 'tests/evaluate_mission.py'),
                     str(rep_path)], capture_output=True, text=True).stdout
ev_lines = {k: line for line in ev.splitlines()
            for k in ('事案発生から全員', 'ボードと遭難者', 'LoRa', '映像伝送') if line.startswith(k)}


def center(d, y, text, font, fill):
    w = d.textlength(text, font=font)
    d.text(((W - w) / 2, y), text, font=font, fill=fill)


# ---- タイトル --------------------------------------------------------------
im = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(im)
center(d, 220, 'AI 海難救助システム', F[84], TX)
center(d, 350, 'ドローンが見つけ、自律フォイルボードが駆けつける', F[44], ACC)
items = ['想定: 遊泳者 3 名が離岸流で沖へ。事案発生の 2 分後にドローンが出動',
         '漂流予測 → 拡大方形捜索 → YOLO11m で発見・再確認 → オペレータがボードを発進させる',
         'ドローンが上空から座標を LoRa で送り、ボードが先回り誘導で接近・停止',
         'PX4 SITL + Gazebo Harmonic + ROS 2 Humble / 目標: 事案発生から 5 分以内']
for i, t in enumerate(items):
    center(d, 490 + i * 70, t, F[36], SUB)
center(d, 900, f'本編は {speed} 倍速で再生', F[36], TX)
im.save(out / 'title.png')

# ---- まとめ ------------------------------------------------------------------
im = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(im)
d.text((80, 60), '結果', font=F[64], fill=TX)
conf = [t for t in rep['targets'] if t['status'] in ('confirmed', 'rescuing')]
rej = [t for t in rep['targets'] if t['status'] == 'rejected']
lines = [
    (f'遭難者 {rep["pob"]} 名中 {rep["n_arrived"]} 名にボードが到着', F[36], ACC),
    (f'誤検出の候補 {len(rej)} 件は、降下して再確認し棄却' if rej else 'AI の誤検出は 0 件。人以外を遭難者と判定しなかった', F[36], TX),
]
for k in ('事案発生から全員', 'ボードと遭難者', 'LoRa', '映像伝送'):
    if k in ev_lines:
        t = ev_lines[k]
        t = t.split('(停止距離')[0].split('  送信時間')[0].replace('事案発生から全員にボード到着', '全員に到着')
        t = t.replace(' / 飛行', '/飛行').replace('常時伝送比', '常時比')
        lines.append((t, F[28], SUB))
y = 180
for t, f, c in lines:
    d.text((80, y), t, font=f, fill=c)
    y += 70 if f is not F[28] else 50
d.text((80, 980), 'コード: GitHub(説明欄)  /  PAI最終課題_ba2vwbus', font=F[28], fill=SUB)
if map_png and map_png.exists():
    m = Image.open(map_png).convert('RGB')
    r = min(900 / m.height, 820 / m.width)
    m = m.resize((int(m.width * r), int(m.height * r)))
    im.paste(m, (W - m.width - 50, (H - m.height) // 2))
im.save(out / 'summary.png')

# ---- 倍速バッジ(透過 PNG)----------------------------------------------------
b = Image.new('RGBA', (200, 56), (0, 0, 0, 0))
d = ImageDraw.Draw(b)
d.rounded_rectangle((0, 0, 199, 55), 12, fill=(220, 60, 40, 230))
label = f'{speed}倍速'
d.text(((200 - d.textlength(label, font=F[36])) / 2, 6), label, font=F[36], fill=(255, 255, 255, 255))
b.save(out / 'speed_badge.png')
print('cards:', out)
