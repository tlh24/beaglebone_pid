#!/bin/bash
scp debian@192.168.10.12:/home/debian/beaglebone_pid/pid.dat .
gnuplot pid.gnuplot
