% unix('scp debian@beaglebone.local:/mnt/ramdisk/pid.dat .')
load pid.dat
t = pid(:,1); 
x = pid(:,2); 
v = pid(:,3); 
v_ = pid(:,4); 
dr = pid(:,5); 

separate = 0;

%need to break into segments.  
dt = diff(t); 
dt = [dt; 0]; 
ends = find(dt < -0.05); 
if numel(ends) == 0 
	ends = length(t); 
end
if separate == 0
	hold off; 
	plot(0,0); 
	hold on;
end
s = 1; 
if separate == 1
	for i = 1:length(ends)
		 e = ends(i); 
		 fig = figure;
		 set (fig, 'Units', 'normalized', 'Position', [0,0,1,1]);
		 plot(t(s:e-1), dt(s:e-1)*24e6, 'm'); 
		 hold on
	%     plot(t(s:e), v_(s:e)/200, 'g'); 
		 plot(t(s:e), v(s:e)/200, 'b'); 
		 plot(t(s:e), dr(s:e)*1000, 'r'); 
		  plot(t(s:e), x(s:e), 'k'); 
	end
else
	for i = 1:length(ends)
		 e = ends(i); 
		 plot(t(s:e), v(s:e)/200, 'b');
		 s = e+1; 
	end
	for i = 1:length(ends)
		 e = ends(i); 
		 plot(t(s:e), dr(s:e)*1000, 'r');
		 s = e+1; 
	end
	for i = 1:length(ends)
		 e = ends(i); 
		 plot(t(s:e), x(s:e), 'k'); 
		 s = e+1; 
	end
	axis([0, 0.08, -1200, 2200])
	q(1) = text(0.04, 1500, 'black = motor / slug position (encoder counts)');
	q(2) = text(0.04, 1400, 'red = motor drive (1000 = DC)');
	q(3) = text(0.04, 1300, 'blue = estimated velocity, arb units');
	q(4) = text(0.04, 1200, 'simlulated insertions spaced at 300\mum');
	for k = 1:4 
		q(k).FontSize = 14; 
	end
end