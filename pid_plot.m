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
	plot(0,0); 
	hold on;
end
s = 1; 
for i = 1:length(ends)
    e = ends(i); 
	 if separate == 1
		 fig = figure;
		 set (fig, 'Units', 'normalized', 'Position', [0,0,1,1]);
		 plot(t(s:e-1), dt(s:e-1)*24e6, 'm'); 
		 hold on
	%     plot(t(s:e), v_(s:e)/200, 'g'); 
		 plot(t(s:e), v(s:e)/200, 'b'); 
		 plot(t(s:e), dr(s:e)*1000, 'r'); 
		  plot(t(s:e), x(s:e), 'k'); 
	 else
		 plot(t(s:e), v(s:e)/200, 'b'); 
		 plot(t(s:e), dr(s:e)*1000, 'r'); 
		 plot(t(s:e), x(s:e), 'k'); 
	 end
	 axis([0, 0.1, -1200, 1500])
    s = e+1; 
end