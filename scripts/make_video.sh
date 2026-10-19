#!/usr/bin/env bash
# 録画した生動画から提出用の動画(1920x1080)を作る
#   使い方: [SPEED=2] [TAIL=8] make_video.sh RAW.mp4 REPORT.json [OUT.mp4]
#   構成: タイトル(5秒)→ 本編(SPEED 倍速。表示画面の右上に Gazebo の 3D 画面を合成)→ 結果(7秒)
#   本編は、全員にボードが着いた時刻の TAIL 秒後(録画の時間で)で切る。帰還と着陸の場面は入れない。
#   切る位置は、録画開始の時刻(RAW と同じ名前の .start)と、mission の JSON の出来事の時刻から求める。
set -e
source "$(dirname "$0")/env.sh"
FF=$(command -v ffmpeg || echo "$HOME/anaconda3/bin/ffmpeg")
RAW=$1; REP=$2; OUT=${3:-$PAI_ROOT/video/PAI_board_demo.mp4}
TMP=$(mktemp -d)
MAP=${REP%.json}.png
[ -f "$MAP" ] || /usr/bin/python3 "$PAI_ROOT/tests/evaluate_mission.py" "$REP" > /dev/null
SPEED=${SPEED:-2}
TAIL=${TAIL:-8}
END=$(/usr/bin/python3 - "$REP" "${RAW%.mp4}.start" "$TAIL" <<'PY'
import json, sys
rep = json.load(open(sys.argv[1]))
start = float(open(sys.argv[2]).read())
ev = [e for e in rep['events'] if '全員' in e['text']]
t = rep['t_start'] + (ev[0]['t'] if ev else rep['elapsed'])
print(round(t - start + float(sys.argv[3]), 1))
PY
)
echo "本編は録画の $END 秒目まで($SPEED 倍速で $(awk -v e=$END -v s=$SPEED 'BEGIN{printf "%.0f", e/s}') 秒)"
/usr/bin/python3 "$(dirname "$0")/make_cards.py" "$TMP" "$REP" "$MAP" "$SPEED"

ENC="-c:v h264_nvenc -preset p5 -cq 21 -pix_fmt yuv420p -r 30"
# 本編: 左モニタ(表示画面)を土台に、右モニタの Gazebo 3D ビュー部分(タイトルバーを除く)を
# 縮小して 3D 枠に重ね、SPEED 倍速にする
"$FF" -loglevel error -y -t "$END" -i "$RAW" -i "$TMP/speed_badge.png" -filter_complex "\
  [0:v]crop=2048:1152:0:0,scale=1920:1080[base];\
  [0:v]crop=1770:996:2123:156,scale=860:484[view];\
  [base][view]overlay=1040:70[v1];\
  [v1][1:v]overlay=1700:12,setpts=PTS/$SPEED[v]" -map "[v]" -an $ENC "$TMP/main.mp4"
"$FF" -loglevel error -y -loop 1 -t 5 -i "$TMP/title.png" $ENC "$TMP/title.mp4"
"$FF" -loglevel error -y -loop 1 -t 7 -i "$TMP/summary.png" $ENC "$TMP/summary.mp4"
printf "file '%s'\n" "$TMP/title.mp4" "$TMP/main.mp4" "$TMP/summary.mp4" > "$TMP/list.txt"
"$FF" -loglevel error -y -f concat -safe 0 -i "$TMP/list.txt" -c copy "$OUT"
echo "完成: $OUT ($("$FF" -i "$OUT" 2>&1 | grep -o 'Duration: [0-9:.]*'))(課題の条件は 1〜3 分)"
