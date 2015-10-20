#!/bin/bash
# must be run as root!
echo cape-universaln > /sys/devices/platform/bone_capemgr/slots
# echo PyBBBIO-eqep0 > /sys/devices/platform/bone_capemgr/slots
# echo bone_eqep0 > /sys/devices/bone_capemgr.9/slots
# this will also pinmux P9.22 to pwm0A function.

config-pin P9.27 qep # quadrature B. 
# config-pin P9.41 in # seems to have a 32kHz clock on it these days.
config-pin P9.91 qep # index -- disable pinmux? no index signal to ocp.
config-pin P9.42 in
config-pin P9.92 qep # quadrature A. 
config-pin P9.22 pwm
config-pin P9.17 out
config-pin P9.18 out

sleep 1
#export to userspace. (this should not fail with a write error)
echo 0 > /sys/class/pwm/pwmchip0/export 
# echo 0 > /sys/class/pwm/export # kernel 3.8
echo 50000 > /sys/class/pwm/pwmchip0/pwm0/period
echo 500 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable
echo on > /sys/class/pwm/pwmchip0/pwm0/power/control
#20kHz. enabled
#(but does not seem to output a pwm until the code is  run)
# below, however, does (on kernel 4.1.9-bone-rt-r16
PWM=/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip?
# i do not know why it's pwmchip2 ... 
echo 0 > $PWM/export
echo 50000 > $PWM/pwm0/period
echo 500 > $PWM/pwm0/duty_cycle
echo 1 > $PWM/pwm0/enable

#export gpio for P9.17 and P9.18 (forward and reverse)
echo 5 > /sys/class/gpio/export
echo 4 > /sys/class/gpio/export

echo out > /sys/class/gpio/gpio5/direction
echo out > /sys/class/gpio/gpio4/direction

#turn on timer1. (timer0 is the 32kHz RTC)
echo on > /sys/devices/platform/ocp/48044000.timer/power/control
# timer is used for high-speed internal interval timing. 
echo on > /sys/devices/platform/ocp/4804c000.gpio/power/control

# code only works with gcc 4.7+
if 0 ; then
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.6 
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.7 40 --slave /usr/bin/g++ g++ /usr/bin/g++-4.7 
sudo update-alternatives --config gcc
fi
