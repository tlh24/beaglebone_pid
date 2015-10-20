#!/bin/bash
scp debian@192.168.10.12:/home/debian/bbpid/pid.dat .
gnuplot pid.gnuplot
