#!/bin/bash
#DoubleArmV7の遠隔側のテストプログラム(カメラあり)

#RTC名を変数に登録
s=/localhost/TkSlider0.rtc
sv=/localhost/TkSliderVelocity0.rtc
r=/localhost/DoubleArmV7-DoubleArmV7PDControllerIoRTC.rtc
m=/localhost/TkMonitorSlider0.rtc
ma=/localhost/TkMonitorSliderAcceleration0.rtc
c=/localhost/DoubleArmV7-VisionSensorIoRTC.rtc
v1=/localhost/ImageViewer1.rtc
v2=/localhost/ImageViewer2.rtc
p1=/localhost/PointCloudViewer1.rtc

./TkSlider.py &
./TkSliderVelocity.py &
./TkMonitorSlider.py &
./TkMonitorSliderAcceleration.py &
#../../ImageViewerがあることが前提
../../ImageViewer/ImageViewer.bash 1
../../ImageViewer/ImageViewer.bash 2
#../../PointCloudViewerがあることが前提
../../PointCloudViewer/PointCloudViewer.bash 1

sleep 3

#コンフィギュレーション
rtconf $v1 set windowTitle FRAME_FRONT_CAMERA
rtconf $v2 set windowTitle Upper_Hand_Camera

#接続
rtcon $s:slider $r:qt
rtcon $sv:slider $r:dq
rtcon $r:q $m:value
rtcon $r:dv $ma:value
rtcon $c:FRAME_FRONT_CAMERA $v1:Image
rtcon $c:Upper_Hand_Camera $v2:Image
rtcon $c:UF_FRONT_CAMERA_DEPTH-depth $p1:pc

#activate
rtact $s $sv $m $ma $v1 $v2 $p1

echo "Chorenoidでシミュレーションを開始，停止してください"

input=
while [ "$input" != "y" ]
do
    echo -n "終了しますか？"
    read input
done

#deactivate
rtdeact $s $sv $m $ma $v1 $v2  $p1

rtexit $s
rtexit $sv
rtexit $m
rtexit $ma
rtexit $v1
rtexit $v2
rtexit $p1
