# simplified makefile. 

OBJS = bbb-eqep.o biphasic.o 

all: pid_tst

%.o:%.cpp
	g++ -c -O2 -o $@ $<
	
pid_tst: $(OBJS)
	g++ -o pid_tst $(OBJS)
	
clean:
	rm -rf *.o pid_tst
