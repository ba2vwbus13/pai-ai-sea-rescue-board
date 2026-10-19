#!/usr/bin/env bash
# ターミナル1: PX4 SITL + Gazebo サーバーを海難救助(ボード)ワールド・捜索用機体で起動する
source "$(dirname "$0")/env.sh"
# PX4 のビルドは ~/.local の kconfiglib などを使うので、ここでは無効化しない
unset PYTHONNOUSERSITE
cd "$PX4_DIR"
[ -x build/px4_sitl_default/bin/px4 ] || make px4_sitl
# ノート PC のハイブリッド GPU で、Gazebo の描画を NVIDIA 側で行う
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
# Gazebo の画面は start_view.sh で別に開く(パネルなし・ドローン追従)
export HEADLESS=${HEADLESS-1}
# 機体: 捜索用機体(エアフレーム 4099)、ワールド: sea_board(worlds/)。
# make のターゲットはワールドの追加に cmake の再設定が要るので、make と同じ手順で直接起動する
cd build/px4_sitl_default/rootfs
PX4_SYS_AUTOSTART=4099 PX4_SIM_MODEL=gz_x500_sar_cam PX4_GZ_WORLD=sea_board GZ_IP=127.0.0.1 ../bin/px4
