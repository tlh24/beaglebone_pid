% unix('scp debian@beaglebone.local:/mnt/ramdisk/pid.dat .')
load pid.dat
t = pid(:,1); 
x = pid(:,2); 
v = pid(:,3); 
v_ = pid(:,4); 
dr = pid(:,5); 

%need to break into segments.  
dt = diff(t); 
dt = [dt; 0]; 
ends = find(dt < -0.05); 
if numel(ends) == 0 
	ends = length(t); 
end
plot(0,0); 
hold on;
s = 1; 
for i = 1:length(ends)
    e = ends(i); 
% 	 figure
    plot(t(s:e-1), dt(s:e-1)*24e6, 'm'); 
% 	 hold on
    plot(t(s:e), v_(s:e)/200, 'g'); 
    plot(t(s:e), v(s:e)/200, 'b'); 
    plot(t(s:e), x(s:e), 'k'); 
    plot(t(s:e), dr(s:e)*1000, 'r'); 
    s = e+1; 
end