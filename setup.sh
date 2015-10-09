#!/bin/bash
# must be run as root!
echo BB-pid0 > /sys/devices/platform/bone_capemgr/slots
# echo PyBBBIO-eqep0 > /sys/devices/platform/bone_capemgr/slots
# echo bone_eqep0 > /sys/devices/bone_capemgr.9/slots
# this will also pinmux P9.22 to pwm0A function.

config-pin P9.27 qep
config -pin P9.41 in
config-pin P9.91 qep
config-pin P9.42 in
config-pin P9.92 qep
config-pin P9.22 pwm
config-pin P9.15 out
config-pin P9.16 out

#export to userspace. 
echo 0 > /sys/class/pwm/pwmchip0/export 
# echo 0 > /sys/class/pwm/export # kernel 3.8
echo 50000 > /sys/class/pwm/pwmchip0/pwm0/period
echo 500 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable
echo on > /sys/class/pwm/pwmchip0/pwm0/power/control
# echo 50000 > /sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm0/period
# echo 50000 > /sys/class/pwm/pwmchip0/period_ns
# echo 500 > /sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm0/duty_cycle
#echo 1 > /sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm0/enable
# echo 0 > /sys/class/pwm/pwmchip0/duty_ns
# echo 1 > /sys/class/pwm/pwmchip0/run
#20kHz. enabled, but no pulses.

#export gpio for P9.15 and P9.16 (forward and reverse)
echo 48 > /sys/class/gpio/export
echo 51 > /sys/class/gpio/export

echo out > /sys/class/gpio/gpio48/direction
echo out > /sys/class/gpio/gpio51/direction

#turn on timer1. (timer0 is the 32kHz RTC)
echo on > /sys/devices/ocp.3/48044000.timer/power/control
# this is used for high-speed internal interval timing. 
