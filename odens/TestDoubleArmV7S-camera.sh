#!/bin/bash
#アームの制御とカメラ画像出力にBodyIoRTCを使った試行

#RTC名を変数に登録
s=/localhost/TkSlider0.rtc
r=/localhost/DoubleArmV7-DoubleArmV7PDControllerIoRTC.rtc
m=/localhost/TkMonitorSlider0.rtc
c=/localhost/DoubleArmV7-VisionSensorIoRTC.rtc
v1=/localhost/ImageViewer1.rtc
v2=/localhost/ImageViewer2.rtc
p1=/localhost/PointCloudViewer1.rtc

./TkSlider.py &
./TkMonitorSlider.py &
#../../ImageViewerがあることが前提
../../ImageViewer/ImageViewer.bash 1
../../ImageViewer/ImageViewer.bash 2
#../../PointCloudViewerがあることが前提
../../PointCloudViewer/PointCloudViewer.bash 1

sleep 3

#外部のRTCはChoreonoidからactivateされないので
rtact $s $m $v1 $v2 $p1

(cd ..; bin/choreonoid sample/OpenRTM/OpenRTM-DoubleArmV7S-camera.cnoid &)

sleep 3

echo "Chorenoidでシミュレーションを開始，停止してください"

input=
while [ "$input" != "y" ]
do
    echo -n "終了しますか？"
    read input
done

#外部のRTCはChoreonoidからdeactivateされないので
rtdeact $s $m $v1 $v2  $p1

rtexit $s
rtexit $m
rtexit $v1
rtexit $v2
rtexit $p1
killall choreonoid
