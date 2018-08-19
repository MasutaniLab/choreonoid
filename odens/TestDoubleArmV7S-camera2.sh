#!/bin/bash
(cd ..; bin/choreonoid sample/OpenRTM/OpenRTM-DoubleArmV7S-camera2.cnoid &)
./TkSlider.py &
./TkMonitorSlider.py &
#../../ImageViewerがあることが前提
../../ImageViewer/ImageViewer.bash 1
#../../ImageViewer/ImageViewer.bash 2

sleep 5

s=/localhost/TkSlider0.rtc
r=/localhost/DoubleArmV7-DoubleArmV7PDControllerIoRTC.rtc
m=/localhost/TkMonitorSlider0.rtc
c=/localhost/DoubleArmV7.rtc
v1=/localhost/ImageViewer1.rtc
v2=/localhost/ImageViewer2.rtc

#input=
#while [ "$input" != "y" ]
#do
#    echo -n "進めていいですか？"
#    read input
#done

rtconf $v1 set BGR 0
#rtconf $v2 set BGR 1

rtcon $s:slider $r:qt
rtcon $r:q $m:value

rtcon $c:FRAME_FRONT_CAMERA $v1:Image
#rtcon $c:WORK_RIGHT_VIEW $v2:Image

rtact $s $m $v1 #$v2

echo "Chorenoidでシミュレーションを開始，停止してください"

input=
while [ "$input" != "y" ]
do
    echo -n "終了しますか？"
    read input
done

rtdeact $s $m $v1 #$v2

rtexit $s
rtexit $m
rtexit $v1
#rtexit $v2
killall choreonoid
