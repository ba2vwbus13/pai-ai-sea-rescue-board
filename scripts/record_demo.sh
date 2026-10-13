#!/usr/bin/env bash
# デモの録画: 画面全体(2 モニタ)を録画しながら AI 捜索を実行し、着陸したら録画を止める
#   前提: start_sim.sh と start_view.sh を起動済み。
#   左モニタ(eDP-1, 2048x1152)= 表示画面(全画面)、右モニタ(HDMI, 1920x1080, +2048+72)= Gazebo 3D
source "$(dirname "$0")/env.sh"
FF=$(command -v ffmpeg || echo "$HOME/anaconda3/bin/ffmpeg")
RAW=${1:-$PAI_ROOT/video/raw_$(date +%Y%m%d_%H%M%S).mp4}
mkdir -p "$(dirname "$RAW")"
SIZE=$(xdpyinfo -display "${DISPLAY:-:1}" | awk '/dimensions:/{print $2}')

"$FF" -loglevel error -y -f x11grab -draw_mouse 0 -framerate 30 -video_size "$SIZE" -i "${DISPLAY:-:1}" \
  -c:v h264_nvenc -preset p5 -cq 23 -pix_fmt yuv420p "$RAW" &
REC=$!
echo "録画開始: $RAW"
sleep 2
"$(dirname "$0")/start_ai.sh" fullscreen:=true window_x:=0 window_y:=0 &
AI=$!
# ミッション完了(新しい報告書 JSON の保存)を待つ
MARK=$(mktemp); mkdir -p "$PAI_ROOT/results"
until find "$PAI_ROOT/results" -name 'mission_*.json' -newer "$MARK" | grep -q .; do
  sleep 3
done
sleep 4
kill -INT $REC; wait $REC 2>/dev/null
echo "録画終了: $RAW"
pkill -9 -f "[i]mage_bridge"      # 終了時のクラッシュ報告を避ける
kill -INT $AI 2>/dev/null
