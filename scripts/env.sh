#!/usr/bin/env bash
# 共通の環境設定(他のスクリプトから source する)
# Anaconda が PATH の先頭にあると PX4 / ROS 2 が誤動作するため取り除く
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i -E 'anaconda|miniconda' | paste -sd:)
unset CONDA_PREFIX PYTHONHOME

export PAI_ROOT=${PAI_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
export PX4_DIR=${PX4_DIR:-$HOME/work/PX4-Autopilot}
export PX4_WS=${PX4_WS:-$HOME/work/map-drone/px4_ws}        # px4_msgs をビルド済みのワークスペース
export PAI_VENV=${PAI_VENV:-$HOME/work/pai_venv}
export PAI_MODELS=$PAI_ROOT/models

# ~/.local の NumPy 2.x がシステムの OpenCV / ROS と衝突するのを防ぐ
export PYTHONNOUSERSITE=1
# 離陸地点(名護市 シーグラスビーチの砂浜)
export PX4_HOME_LAT=26.5472 PX4_HOME_LON=128.0540 PX4_HOME_ALT=3
