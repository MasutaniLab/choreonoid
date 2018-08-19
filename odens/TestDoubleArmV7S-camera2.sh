#!/bin/bash
#アームの制御にBodyIoRTC，カメラ画像出力にBodyRTC（旧方式）を使った試行

#RTC名を変数に登録
s=/localhost/TkSlider0.rtc
r=/localhost/DoubleArmV7-DoubleArmV7PDControllerIoRTC.rtc
m=/localhost/TkMonitorSlider0.rtc
c=/localhost/DoubleArmV7.rtc
v1=/localhost/ImageViewer1.rtc
p1=/localhost/PointCloudViewer1.rtc

./TkSlider.py &
./TkMonitorSlider.py &
#../../ImageViewerがあることが前提
../../ImageViewer/ImageViewer.bash 1
#../../PointCloudViewerがあることが前提
../../PointCloudViewer/PointCloudViewer.bash 1

sleep 3

rtconf $v1 set BGR 0

#BodyRTCに接続していないRTCはChoreonoidからactivateされないので
rtact $s $m

(cd ..; bin/choreonoid sample/OpenRTM/OpenRTM-DoubleArmV7S-camera2.cnoid &)

sleep 3

echo "Chorenoidでシミュレーションを開始，停止してください"

input=
while [ "$input" != "y" ]
do
    echo -n "終了しますか？"
    read input
done

#BodyRTCに接続していないRTCはChoreonoidからdeactivateされないので
rtdeact $s $m

rtexit $s
rtexit $m
rtexit $v1
rtexit $p1
killall choreonoid
