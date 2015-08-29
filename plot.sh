#!/bin/bash
scp debian@169.230.191.137:/home/debian/beaglebone_pid/pid.dat .
gnuplot pid.gnuplot
