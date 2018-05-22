#!/bin/bash
scp debian@beaglebone.local:/home/debian/bbpid/pid.dat .
gnuplot pid.gnuplot
