/**
 * This example program demonstrates the use of the BBB-eQEP library.
 * Copyright (C) 2014 James Zapico <james.zapico@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
**/
// http://www.ti.com/lit/ug/spruh73l/spruh73l.pdf
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <errno.h>
#include <math.h>
#include <string.h>
#include <signal.h>

#include "bbb-eqep.h"
#include "sock.h"

using std::cout;
using std::endl;
using std::cerr;
using namespace BBB;

bool g_die = false; 
int	mem_fd; 
uint16_t* pwm_addr = 0; 
uint32_t* gpio_addr = 0; 
uint32_t* timer_addr = 0; 
uint32_t* pwmss_addr = 0; 
uint32_t* prcm_addr = 0; 

uint32_t saved[0x1000/4]; 

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;

int g_controlSock; //RX from controller.ml
int g_statusSock; //TX to controller (e.g. 'done')
struct sockaddr_in g_statusAddr;
#define CMD_SIZ 5120
static char g_cmdt[CMD_SIZ]; 
static char g_stat[CMD_SIZ]; 

//mmap is used as a general-purpose mechanism for accessing system registers from userspace.
uint32_t* map_register(uint32_t base_addr, uint32_t len){
	uint32_t* addr = (uint32_t*)mmap(NULL, len,
		PROT_READ | PROT_WRITE, MAP_SHARED , mem_fd, base_addr);
	if (addr == MAP_FAILED )
	{
		printf("Memory Mapping failed for 0x%04x register\n", base_addr);
		printf("ERROR: (errno %d)\n", errno);
		return 0;
	}
	printf("address 0x%08x memmapped\n", base_addr);
	return addr; 
}

void motor_forward(){
	gpio_addr[0x190 / 4] = 0x1 << 4; //clear data out
	gpio_addr[0x194 / 4] = 0x1 << 5; //set data out
}
void motor_reverse(){
	gpio_addr[0x190 / 4] = 0x1 << 5; //clear data out
	gpio_addr[0x194 / 4] = 0x1 << 4; //set data out
}
void motor_stop(){
	gpio_addr[0x190 / 4] = 0x1 << 4; //clear data out
	gpio_addr[0x190 / 4] = 0x1 << 5; //clear data out
}
void motor_setDrive(float dr){
	if(dr < 0.0) motor_reverse(); 
	if(dr > 0.0) motor_forward();
	if(dr == 0.0) motor_stop(); 
	dr = dr > 1.0 ? 1.0 : dr; 
	dr = dr < -1.0 ? -1.0 : dr; //should work without this, too. 
	pwm_addr[9] = (int)(fabs(dr) * 2000.0); 
	//25kHz PWM cycle. 
}
//simple step test, constant acceleration and deceleration. 
//returns the desired position.
float acc_seg(float d, float duration, float t){
	float a = 4.f * d / (duration * duration); 
	if(t < duration / 2.f){
		return 0.5f * a * t * t; 
	}
	else if(t < duration){
		return d - 0.5f * a * (duration-t)*(duration-t); 
	}else{
		return d; 
	}
}
float step_seg(float d, float duration, float t){
	float u = duration / 4.f; 
	if(t < u){
		return 0; 
	}else if(t < u*2.f){
		return acc_seg(d, u, t-u); 
	}else if(t < u*3.f){
		return d; 
	}else if(t < duration){
		return acc_seg(-1.f*d, u, t - 3.f*u) + d; 
	}else return 0; 
}

float get_time(){
	int i = timer_addr[0x3c / 4];  //counter register.
	return (float)i / 24e6; //output in seconds.
}

static void set_latency_target(void){
	struct stat s;
	int err;
	
	errno = 0;
	err = stat("/dev/cpu_dma_latency", &s);
	if (err == -1) {
		printf("WARN: stat /dev/cpu_dma_latency failed, %s\n", strerror(errno));
		return;
	}

	errno = 0;
	latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
	if (latency_target_fd == -1) {
		printf("WARN: open /dev/cpu_dma_latency, %s\n", strerror(errno));
		return;
	}

	errno = 0;
	err = write(latency_target_fd, &latency_target_value, 4);
	if (err < 1) {
		printf("# error setting cpu_dma_latency to %d! %s\n", latency_target_value, strerror(errno));
		close(latency_target_fd);
		return;
	}
	printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
}

void cleanup(){
	motor_stop(); 
	motor_setDrive(0.0); 
	munmap(gpio_addr, 0x200);
	munmap(timer_addr, 0x200);
	munmap(pwmss_addr, 0x1000); 
	munmap(prcm_addr, 0x1000); 
	close (mem_fd); 
}

void lock(){
	set_latency_target(); 
	/* lock all memory (prevent swapping) */
	if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		perror("mlockall");
		cleanup(); 
		g_die = true; 
	}
}
void unlock(){
	munlockall();
	/* close the latency_target_fd if it's open */
	if (latency_target_fd >= 0)
		close(latency_target_fd);
}
void sig_handler(int s){
	printf("Caught signal %d\n",s);
	g_die = true; 
}

int main (int argc, char const *argv[])
{
	struct timeval tv1, tv2;
	uint32_t eqep_pos, eqep0_pos;
	
	// bidirectional socket communication. 
	g_controlSock = setup_socket(4596); 
	g_statusSock = connect_socket(4597,"mongolianbbq.local"); 
	get_sockaddr(4597,"mongolianbbq.local",&g_statusAddr);
	
	// sig hangler for control-C (sigint)
	struct sigaction sigIntHandler;
   sigIntHandler.sa_handler = sig_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;
   sigaction(SIGINT, &sigIntHandler, NULL);
	
	mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mem_fd < 0){
		printf("Can't open /dev/mem\n");
		return 1;
	}
	
	gpio_addr = map_register(0x44e07000, 0x200); //gpio0.
	timer_addr = map_register(0x48044000, 0x200); // timer 4.
	pwmss_addr = map_register(0x48300000, 0x1000); //see page 180 in the TRM.
	prcm_addr = map_register(0x44e00000, 0x1000); //CM_PER module.
	
	//enable l4ls_glck, so that we may access epwmss peripherals. 
	// (though ... it seems to be already enabled )
	//if you don't enable these clock domains, 
	//you'll ge a bus error in dmesg when reading/writing.
	// e.g. "unhandled fault external abort on non-linefetch". 
	prcm_addr[0x60 / 4] = 0x2; //CM_PER_L4LS_CLKCTRL, p 1182, all l4ls peripheral clocks. 
	prcm_addr[0x88 / 4] = 0x2; //enable timer4. page 1191, CM_PER_TIMER4_CLKCTRL
	prcm_addr[0xd4 / 4] = 0x2; //enable epwmss0. page 1199, CM_PER_EPWMSS0_CLKCTRL
	prcm_addr[0xac / 4] = 0x40002; //enable GPIO1. page 1192, CM_PER_GPIO1_CLKCTRL
	prcm_addr[0x120 / 4] = 0x2; //enable L4HS clock, page 1214, CM_PER_L4HS_CLKCTRL (should be enabled already)
	printf("CM_PER_L4LS_CLKSTCTRL = 0x%x\n", prcm_addr[0]); // page 1163.
	printf("CM_PER_L4HS_CLKSTCTRL = 0x%x\n", prcm_addr[0x11c / 4]); //see?  enabled..
	
	gpio_addr[0x134 / 4] &= 0xffffffff ^ ((0x1 << 4) | (0x1 << 5)); //enable output (should be already ./testenabled by setup.sh) 
	printf("GPIO0_REV 0x%X\n", gpio_addr[0]); 
	if(gpio_addr[0] == 0x50600801) 
		printf(" .. looks OK\n"); 
	printf("GPIO0_OE 0x%X\n", gpio_addr[0x134 / 4]); 
	printf("GPIO0_DI 0x%X\n", gpio_addr[0x138 / 4]); 
	
	printf("TIMER4_TIDR 0x%X\n", timer_addr[0]);
	printf("TIMER4_TIOCP_CFG 0x%X\n", timer_addr[0x10 / 4]);
	printf("TIMER4_TCLR 0x%X\n", timer_addr[0x38 / 4]); 
	printf("TIMER4_TCRR 0x%X\n", timer_addr[0x3c / 4]);  //counter register. 
	printf("TIMER4_TLDR 0x%X\n", timer_addr[0x40 / 4]); //load on overflow; defalut to zero. 
	timer_addr[0x10 / 4] = 0xa; //1010, smart-idle, emufree, no reset.
	timer_addr[0x44 / 4] = 0xffffffff; //reload (zero) the TCRR from the TLDR. 
	timer_addr[0x38 / 4] = 0x3 ; //0000 0000 0000 0011
	
	printf("EPWMSS0_IDVER 0x%X\n", pwmss_addr[0]);
	printf("EPWMSS0_SYSCFG 0x%X\n", pwmss_addr[1]);
	printf("EPWMSS0_CLKCFG 0x%X\n", pwmss_addr[2]);
	printf("EPWMSS0_CLKSTAT 0x%X\n", pwmss_addr[3]);
	// pwmss_addr[2] = 0x111; //enable all clocks. EPWMSS0_CLKCFG, page 2231
	// again, above should be done by setup.sh

	printf("accessing eQEP0...\n"); 
	eQEP eqep(0);
	printf("base address (mmaped) 0x%X\n", eqep.getPWMSSPointer()); 
	printf("SYSCONFIG 0x%X\n", *(uint32_t*)(eqep.getPWMSSPointer() + PWM_SYSCONFIG));
	printf("CLKCONFIG 0x%X\n", *(uint32_t*)(eqep.getPWMSSPointer()+PWM_CLKCONFIG));
	printf("QEPCTL0   0x%X\n",  eqep.getControl());
	printf("QDECCTL0  0x%X\n",  eqep.getDecoderControl());
	printf("QEINT0    0x%X\n",  eqep.getInterruptEnable());
	printf("QUPRD0    0x%u\n", eqep.getUnitPeriod());
	printf("QPOSMAX0  0x%X\n", eqep.getMaxPos());
	printf("QEPSTS0   0x%X\n",  eqep.getStatus());

	//enable the PWM module associated with this eQEP. 
	pwm_addr = (uint16_t*)(pwmss_addr + (0x200/4)); //got the offset from the device tree.
	//note indexing as shorts. 
	pwm_addr[5] = 2000; // PRD. 25kHz from the 100Mhz clock.
	pwm_addr[7] = 0x0; // CMPCTL.  enable shadowm load when counter=0. 
	pwm_addr[9] = 166; //CMPA. 5% duty cycle.
	pwm_addr[11] = 0x12;  // set when the counter = 0; clear when counter = CMPA
 	pwm_addr[12] = 0; //disable output B. 
 	pwm_addr[0] = 0xc0; // up count mode, software sync, shadow on.
	printf("PWM TBCTL 0x%X (0xC0)\n", pwm_addr[0]); 
	printf("PWM TBSTS 0x%X (0x1)\n", pwm_addr[1]); 
	printf("PWM TBCNT 0x%X (not zero)\n", pwm_addr[4]); 
	printf("PWM TBPRD 0x%X (%d) \n", pwm_addr[5], pwm_addr[5]); 
	printf("PWM CMPCTL 0x%X (0x0)\n", pwm_addr[7]); 
	printf("PWM CMPA 0x%X (%d)\n", pwm_addr[9], pwm_addr[9]);
	printf("PWM CMPB 0x%X (%d)\n", pwm_addr[10], pwm_addr[10]);  
	printf("PWM AQCTLA 0x%X (0x12)\n", pwm_addr[11]); 

	//time a read-loop to assess speed
	timer_addr[0x44 / 4] = 0xffffffff; //reload (zero) the TCRR from the TLDR. 
	int num_reads = 1000000;
	int i;
	gettimeofday(&tv1,NULL);
	for(i=0;i<num_reads;i++){
		eqep_pos = eqep.getPosition();
	}
	int timer1_s = timer_addr[0x3c / 4]; 
	gettimeofday(&tv2,NULL);

	//find difference between start and end time
	unsigned long dt_micros = (1000000 * tv2.tv_sec + tv2.tv_usec)-(1000000 * tv1.tv_sec + tv1.tv_usec);
	float time_per_read = (float)dt_micros/num_reads;
	float change_rate = (float)(num_reads)/((float)(dt_micros)) * 1000;

	printf("last position: %i\n", eqep_pos);
	printf("micros per read: %f\n", time_per_read);
	printf("quadrature read rate (kHz): %f\n",change_rate);
	printf("revid: 0x%X (should read 44D31103)\n",eqep.getRevisionID());

	//calc clock rate. 
	printf("timer1 clock rate %f Mhz\n", (float)timer1_s / ((float)dt_micros)); 
	
	//calibrate friction point. 
	//slug must start at bottom.
	float friction = 0.0; 
	if(0){
		int st = eqep.getPosition();
		int dd = 0; 
		friction = 0.0; 
		while(dd > -250 && friction <= 0.1){
			friction += 0.005; 
			motor_setDrive(-1.0*friction); 
			usleep(500000); 
			dd = eqep.getPosition() - st; 
		}
		if(dd >= 0){
			printf("Motor polarity looks reversed, %d.  Check your wiring.\n", dd); 
			cleanup(); 
			return 0; 
		}
		printf("measured friction point %f (%d)\n", friction, dd); 
	}else{
		friction = 0.045; // seems pretty consistent now.
	}
	friction += 0.005; // a bit of margin.

	
	float t = 0.f; 
	float td = 0.0; 
	int x = 0; 
	int x_old = 0; 
	float dr = 0.0; //drive command.
	float v = 0.0;  //estimate of the velocity.
	float v_ = 0.0; //instantaneous velocity.
	float t_old = 0.0; 
 	float t_vold = 0.0; 
 	float v0 = 0.0; 
	float dr_int = 0.0; 
	float dr_int2 = 0.0; 
	float c = 1e9; 
	int n = 0; 
	int stoppos = 0; 
	int cyltop = 0x3fffffff; 
	int cylbot = 0x3fffffff; 
	float* sav = (float*)malloc(sizeof(float) * 5 * 1e6); 
		//20MB .. 1e6 samples: should be more than enough. 
	int totalWrite = 0; 
	int savn = 0; 
	auto update_velocity = [&] (int nn, float lerp) -> void {
		float t1 = get_time(); 
		x = eqep.getPosition() - cyltop;
		float t2 = get_time();
		t = (t1 + t2) / 2.0;
		if(nn == 0){
			x_old = x; 
			t_vold = t; 
		}else{
			if(x != x_old){
				//update the velocity. 
				float dt_ = t - t_vold;
				v_ = (float)(x - x_old) / dt_; 
				float lerp_ = (dt_ + 0.00002) / 0.0002; 
				lerp_ = lerp_ > 4.0 ? 4.0 : lerp_; 
				lerp_ *= lerp; 
				v = lerp_*v_ + (1.0 - lerp_)*v; //simple fading-memory filter 
					//(with semi-constant timeconstant at increasing update)
				t_vold = t; 
				x_old = x;
			}
		}
	};
	auto save_dat = [&] () -> void {
		if(savn < 1e6 && sav){
			sav[savn*5+0] = t; 
			sav[savn*5+1] = (float)x; 
			sav[savn*5+2] = v; 
			sav[savn*5+3] = v_; 
			sav[savn*5+4] = dr; 
			savn++; 
		}
	};
	printf("waiting for commands.\n"); 
	//read in commands
	while(!g_die){
		bzero(g_cmdt, sizeof(g_cmdt)); 
		int n = recvfrom(g_controlSock, g_cmdt, CMD_SIZ,0,0,0); 
		if( n > 0 ){
			unsigned int i = 0; 
			char cmd[CMD_SIZ]; 
			while(i<sizeof(g_cmdt) && g_cmdt[i]){
				if(g_cmdt[i] >= 'A' && g_cmdt[i] <= 'z'){
					printf("command from controller %s ... parsing\n",g_cmdt);
					bool valid = true; 
					memcpy(cmd, &g_cmdt[i], std::min(strlen(&g_cmdt[i]),(size_t)(CMD_SIZ-i-1))); 
					// need to copy it - strtok modifies the string in-place. 
					if((cmd[0] == 'U' && cmd[1] == 'p') || 
						(cmd[0] == 'D' && cmd[1] == 'w')){
						//drive the slug up -- gradually ramp up the torque, if it moves, 
						//reply when it stops.  
						float scl = 1.0;
						if(cmd[0] == 'U') scl = -1.0; 
						int sta = eqep.getPosition(); 
						dr = scl * 0.4 * friction; 
						while(dr*scl < friction){
							dr += 0.005*scl; 
							motor_setDrive(dr); 
							usleep(200000); 
						}
						//see if it moved.  if so, wait (one sec max) for it to stop. 
						if(eqep.getPosition() != sta){
							for(int j=0; j<10; j++){
								sta = eqep.getPosition(); 
								usleep(100000); 
								if(eqep.getPosition() == sta)
									break; 
							}
						}
						if(scl < 0.0){ //drove the slug up + wait; store the position.
							cyltop = eqep.getPosition(); 
						}
						snprintf(g_stat, CMD_SIZ, "move done \n"); 
					}
					else if(cmd[0] == 'O' && cmd[1] == '!'){
						//retract -- out!
						//slug needs to be at bottom -- needle exposed / inserted.
						//check if the move makes sense. 
						if(eqep.getPosition() - cyltop < 390){
							snprintf(g_stat, CMD_SIZ, "invalid retraction -- insufficient deceleration space, %d \nmove done\n", eqep.getPosition() - cyltop); 
							//don't change the drive.
						}else{
							printf("deceleration space %d\n", eqep.getPosition() - cyltop);
							fflush(stdout); 
							usleep(50000); 
							// we're going for it!!
							cylbot = eqep.getPosition(); 
							lock(); 
							timer_addr[0x44 / 4] = 0xffffffff; //reload (zero) the TCRR from the TLDR.
							//the reload may take a little bit ...
							int pt = get_time(); 
							t = pt; 
							while(t - pt >= 0.0){
								update_velocity(0, 0.0);
							}
							savn = 0; 
							n = 0; 
							int stoplatch = 0; 
							update_velocity(0, 0.0); //updates the time, too.
							while(t < 0.1){ //total retraction should take (much) less than 100ms.
								update_velocity(n, 0.2);
								if(t < 0.0066){
									dr = 1.0; //compress the spring down; stop just before it maxes out
								}else if(t < 0.0168 && x > 390 ){ //+ (j/3)*100
									//drive up, but stop if too close to cyltop.
									if(x > cylbot - cyltop && x > 800) dr = -1.0; //drive up.  near peak velocity @ crossing (when the slug will hit the actuator rod anyway)
									else dr = -0.1; //coast up
								}else if(t < 0.035){
									if(v < -50*200 && !stoplatch){
										dr = -1.0 * (v + 40*200) / (600.0*200.0); 
									}else{
										if(!stoplatch){
											stoppos = eqep.getPosition(); 
											stoplatch = 1; 
										}
										dr = -0.5*friction; //retract(slowly)
									}
								}else{
									dr = -0.7*friction; //hold (up)
								}
								motor_setDrive(dr); 
								save_dat(); 
								n++; 
							}
							unlock(); 
							printf("deceleration margin %d\n", stoppos-eqep.getPosition());  
							printf("writing out data record (%d)..", savn); 
							FILE* dat_fd = fopen("/mnt/ramdisk/pid.dat", "a"); 
							for(int j=0; j<savn && totalWrite<9e5; j++){
								for(int k=0; k<5; k++){
									fprintf(dat_fd, "%e\t", sav[j*5+k]); 
								}
								fprintf(dat_fd, "\n"); 
								fflush(dat_fd); 
								totalWrite++; //to keep from overflowing the ramdisk.
								//this is a realtime process -- make sure the kernel has time to flush its buffers.
								if(j%10000 == 0){
									usleep(100000); 
									printf("."); 
									fflush(stdout);
								}
							}
							fclose(dat_fd); 
							motor_setDrive(-0.7*friction);
							snprintf(g_stat, CMD_SIZ, "move done \n"); 
						}
					}
					else if(cmd[0] == 'I' && cmd[1] == 'd'){
						motor_setDrive(0.0); //idle the motor.
						snprintf(g_stat, CMD_SIZ, "move done \n"); 
					}
					else if(cmd[0] == 'Q' && cmd[1] == 't'){
						g_die = true; 
						snprintf(g_stat, CMD_SIZ, "quitting \n"); 
					}
					else {
						snprintf(g_stat, CMD_SIZ, "unknown command \n"); 
					}
					if(g_statusSock > 0){
						n = sendto(g_statusSock,g_stat,std::min((int)strlen(g_stat), CMD_SIZ-1), 0, 
							(struct sockaddr*)&g_statusAddr, sizeof(g_statusAddr));
						if (n < 0) 
							printf("ERROR %d writing to Controller status socket\n",errno);
						else printf("%s", g_stat); //echo. 
					}
				}
				while(i < sizeof(g_cmdt) && g_cmdt[i] != '\n' && g_cmdt[i]) i++; 
				i++; //move to one past the newline. 
			}
		} // end n > 0
		else{
			usleep(16000); //wait for another command.
		}
	} //end while
	cleanup(); 
	free(sav); 
	close_socket(g_controlSock); 
	return 0;
}
