#!/bin/bash
# sudo rm -f /mnt/ramdisk/pid.dat
# git pull
# make
sudo chrt -f 99 /home/debian/bbpid/pid_tst
# scp /mnt/ramdisk/pid.dat tlh24@192.168.10.1:/home/tlh24/bbpid/pid.dat
