#!/bin/bash
#DoubleArmV7の遠隔側のテストプログラム(カメラなし)

#RTC名を変数に登録
s=/localhost/TkSlider0.rtc
sv=/localhost/TkSliderVelocity0.rtc
r=/localhost/DoubleArmV7-DoubleArmV7PDControllerIoRTC.rtc
m=/localhost/TkMonitorSlider0.rtc
ma=/localhost/TkMonitorSliderAcceleration0.rtc

./TkSlider.py &
./TkSliderVelocity.py &
./TkMonitorSlider.py &
./TkMonitorSliderAcceleration.py &

sleep 3

#接続
rtcon $s:slider $r:qt
rtcon $sv:slider $r:dq
rtcon $r:q $m:value
rtcon $r:dv $ma:value

#activate
rtact $s $sv $m $ma

echo "Chorenoidでシミュレーションを開始，停止してください"

input=
while [ "$input" != "y" ]
do
    echo -n "終了しますか？"
    read input
done

#deactivate
rtdeact $s $sv $m $ma

rtexit $s
rtexit $sv
rtexit $m
rtexit $ma
