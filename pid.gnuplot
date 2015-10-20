# gnuplot script file
set xlabel "time"

plot "pid.dat" using 1:2 title 'x' with lines, \
#	"pid.dat" using 1:($3/200) title 'v/200' with lines, \
#	"pid.dat" using 1:($4/2000) title 'v-/2000' with lines, \
	"pid.dat" using 1:($5*1000) title 'dr*1000' with lines

pause -1
