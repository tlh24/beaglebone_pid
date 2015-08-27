#!/bin/bash

echo cape-universaln > /sys/devices/bone_capemgr.*/slots
# echo bone_eqep0 > /sys/devices/bone_capemgr.9/slots
# this will also pinmux P9.22 to pwm0A function.

config-pin P9.27 qep
config-pin P9.41 in
config-pin P9.91 qep
config-pin P9.42 in
config-pin P9.92 qep
config-pin P9.22 pwm
config-pin P9.15 out
config-pin P9.16 out

echo 0 > /sys/class/pwm/export
echo 50000 > /sys/class/pwm/pwm0/period_ns
echo 0 > /sys/class/pwm/pwm0/duty_ns
echo 1 > /sys/class/pwm/pwm0/run
#20kHz. enabled, but no pulses.

#export gpio for P9.15 and P9.16 (forward and reverse)
echo 48 > /sys/class/gpio/export
echo 51 > /sys/class/gpio/export

echo out > /sys/class/gpio/gpio48/direction
echo out > /sys/class/gpio/gpio51/direction

#turn on timer1. (timer0 is the 32kHz RTC)
echo on > /sys/devices/ocp.3/48044000.timer/power/control
