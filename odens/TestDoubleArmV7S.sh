#!/bin/bash
(cd ..; bin/choreonoid sample/OpenRTM/OpenRTM-DoubleArmV7S-single.cnoid &)
./TkSlider.py &
./TkMonitorSlider.py &

sleep 5

s=/localhost/TkSlider0.rtc
r=/localhost/DoubleArmV7-DoubleArmV7PDControllerIoRTC.rtc
m=/localhost/TkMonitorSlider0.rtc

rtcon $s:slider $r:qt
rtcon $r:q $m:value

rtact $s $m

while [ "$input" != "y" ]
do
    echo "終了しますか？"
    read input
done

rtdeact $s $m

rtexit $s
rtexit $m
