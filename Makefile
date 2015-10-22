# simplified makefile. 

OBJS = bbb-eqep.o sock.o biphasic.o

all: pid_tst

%.o:%.cpp
	g++ -std=c++11 -c -O2 -o $@ $<
	
pid_tst: $(OBJS)
	g++ -o pid_tst $(OBJS)

# BB-pid0-00A0.dtbo: BB-pid0-00A0.dts
# 	dtc -O dtb -b 0 $< -o $@
# 
# install: BB-pid0-00A0.dtbo
# 	sudo cp $< /lib/firmware
# 	echo "echo BB-pid0 > /sys/devices/platform/bone_capemgr/slots"
	
BB-PWM2-00A0.dtbo: BB-PWM2-00A0.dts
	dtc -O dtb -b 0 -@ $< -o $@

install: BB-PWM2-00A0.dtbo
	sudo cp $< /lib/firmware
	echo "echo BB-PWM2 > /sys/devices/platform/bone_capemgr/slots"
	
clean:
	rm -rf *.o pid_tst *.dtbo
