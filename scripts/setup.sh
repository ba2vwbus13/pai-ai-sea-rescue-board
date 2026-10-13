#!/usr/bin/env bash
# 初回だけ実行するセットアップ(Ubuntu 22.04 + ROS 2 Humble + PX4 + Gazebo Harmonic 構築済みが前提)
set -e
source "$(dirname "$0")/env.sh"

echo "== 1. Python 仮想環境(onnxruntime-gpu)"
if [ ! -d "$PAI_VENV" ]; then
  /usr/bin/python3 -m venv --system-site-packages "$PAI_VENV"
fi
"$PAI_VENV/bin/pip" install -q -r "$PAI_ROOT/requirements.txt"

echo "== 2. YOLO モデル(ONNX)の用意"
if [ ! -f "$PAI_MODELS/yolo11m.onnx" ] || [ ! -f "$PAI_MODELS/yolo11s.onnx" ]; then
  EXP=$HOME/work/export_venv
  [ -d "$EXP" ] || /usr/bin/python3 -m venv "$EXP"
  "$EXP/bin/pip" install -q torch torchvision --index-url https://download.pytorch.org/whl/cpu
  "$EXP/bin/pip" install -q ultralytics onnx onnxslim
  (cd "$PAI_MODELS" && for m in yolo11m yolo11s; do
     "$EXP/bin/yolo" export model=$m.pt format=onnx imgsz=640 opset=12 > /dev/null; rm -f $m.pt; done)
fi

echo "== 3. Gazebo のモデルを取得し、ワールドと捜索用機体を PX4 に登録"
for m in "Casual female" "Rescue Randy" "Standing person" "Walking person" "RC Boat" \
         "Coast Water" Pier "Vessel C" mb_round_buoy_orange mb_marker_buoy_red; do
  gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/${m// /%20}" > /dev/null || true
done
/usr/bin/python3 "$PAI_ROOT/worlds/make_world.py"
ln -sf "$PAI_ROOT/worlds/sea_board.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/sea_board.sdf"
ln -sfn "$PAI_ROOT/gz_models/x500_sar_cam" "$PX4_DIR/Tools/simulation/gz/models/x500_sar_cam"
AF=$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes
cp "$PAI_ROOT/gz_models/4099_gz_x500_sar_cam" "$AF/"
grep -q 4099_gz_x500_sar_cam "$AF/CMakeLists.txt" || \
  sed -i 's/^\(\s*\)4014_gz_x500_mono_cam_down/&\n\14099_gz_x500_sar_cam/' "$AF/CMakeLists.txt"
(cd "$PX4_DIR" && env -u PYTHONNOUSERSITE make px4_sitl)

echo "== 4. ROS 2 パッケージのビルド"
source /opt/ros/humble/setup.bash
source "$PX4_WS/install/setup.bash"
cd "$PAI_ROOT/ros2_ws" && colcon build --symlink-install
echo "完了"
