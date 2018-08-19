#!/bin/bash
#アームの制御にBodyIoRTCを使った試行

#RTC名を変数に登録
s=/localhost/TkSlider0.rtc
r=/localhost/DoubleArmV7-DoubleArmV7PDControllerIoRTC.rtc
m=/localhost/TkMonitorSlider0.rtc

./TkSlider.py &
./TkMonitorSlider.py &

sleep 3

#外部のRTCはChoreonoidからactivateされないので
rtact $s $m

(cd ..; bin/choreonoid sample/OpenRTM/OpenRTM-DoubleArmV7S-single.cnoid &)

sleep 3

echo "Chorenoidでシミュレーションを開始，停止してください"

input=
while [ "$input" != "y" ]
do
    echo -n "終了しますか？"
    read input
done

#外部のRTCはChoreonoidからdeactivateされないので
rtdeact $s $m $v1 $v2

rtexit $s
rtexit $m
killall choreonoid
