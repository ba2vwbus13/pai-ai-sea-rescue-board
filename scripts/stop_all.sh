#!/usr/bin/env bash
# シミュレーションと AI ノードをすべて止める
# image_bridge は終了時に異常終了してクラッシュ報告(apport)が出るので、強制終了しておく
pkill -9 -f "[i]mage_bridge"
pkill -INT -f "[r]os2 launch ai_patrol"
pkill -f "[l]ib/ai_patrol/"
pkill -f "[i]mage_bridge"
pkill -x MicroXRCEAgent
pkill -f "[b]in/px4"
pkill -f "[g]z sim"
pkill -f "[t]ruth_logger.py"
echo "停止しました"
