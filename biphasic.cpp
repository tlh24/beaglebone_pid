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
#include <errno.h>
#include <math.h>

#include "bbb-eqep.h"

using std::cout;
using std::endl;
using std::cerr;
using namespace BBB;

int	mem_fd; 
uint16_t* pwm_addr = 0; 
uint32_t* gpio_addr = 0; 
uint32_t* timer_addr = 0; 
uint32_t* pwmss_addr = 0; 

uint32_t saved[0x1000/4]; 

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
	gpio_addr[0x190 / 4] = 0x1 << 5; //clear data out
	gpio_addr[0x194 / 4] = 0x1 << 4; //set data out
}
void motor_reverse(){
	gpio_addr[0x190 / 4] = 0x1 << 4; //clear data out
	gpio_addr[0x194 / 4] = 0x1 << 5; //set data out
}
void motor_stop(){
	gpio_addr[0x190 / 4] = 0x1 << 4; //clear data out
	gpio_addr[0x190 / 4] = 0x1 << 5; //clear data out
}
void motor_setPWM(float duty){
	pwm_addr[9] = (int)(duty * 20e3); 
	//see setup.sh -- 20kHz PWM cycle. 
}
void motor_setDrive(float dr){
	if(dr < 0.0) motor_reverse(); 
	if(dr > 0.0) motor_forward();
	if(dr == 0.0) motor_stop(); 
	motor_setPWM(fabs(dr)); 
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

void cleanup(){
	motor_stop(); 
	motor_setPWM(0.0); 
	munmap(gpio_addr, 0x200);
	munmap(timer_addr, 0x200);
	munmap(pwmss_addr, 0x1000); 
	close (mem_fd); 
}

int main (int argc, char const *argv[])
{
	struct timeval tv1, tv2;
	uint32_t eqep_pos, eqep0_pos;
	
	mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mem_fd < 0){
		printf("Can't open /dev/mem\n");
		return 1;
	}
	
	gpio_addr = map_register(0x44e07000, 0x200); //gpio0.
	timer_addr = map_register(0x48044000, 0x200); // timer 4.
	pwmss_addr = map_register(0x48300000, 0x1000); //see page 180 in the TRM.
	
	gpio_addr[0x134 / 4] &= 0xffffffff ^ ((0x1 << 4) | (0x1 << 5)); //enable output (should enabled by setup.sh) 
	printf("GPIO0_REV 0x%X\n", gpio_addr[0]); 
	if(gpio_addr[0] == 0x50600801) 
		printf(" .. looks OK\n"); 
	printf("GPIO0_OE 0x%X\n", gpio_addr[0x134 / 4]); 
	printf("GPIO0_DI 0x%X\n", gpio_addr[0x138 / 4]); 
	for(int i=0; i<10000; i++){
		gpio_addr[0x190 / 4] = 0x1 << 4; //clear data out
		usleep(100); 
		gpio_addr[0x194 / 4] = 0x1 << 4; //set data out
		usleep(10); 
	} 
	
	printf("TIMER4_TIDR 0x%X\n", timer_addr[0]);
	printf("TIMER4_TIOCP_CFG 0x%X\n", timer_addr[0x10 / 4]);
	printf("TIMER4_TCLR 0x%X\n", timer_addr[0x38 / 4]); 
	printf("TIMER4_TCRR 0x%X\n", timer_addr[0x3c / 4]);  //counter register. 
	printf("TIMER4_TLDR 0x%X\n", timer_addr[0x40 / 4]); //load on overflow; defalut to zero. 
	fflush(stdout);
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
	pwm_addr[5] = 5000; // PRD. 20kHz from the 100Mhz clock.
	pwm_addr[7] = 0x0; // CMPCTL.  enable shadowm load when counter=0. 
	pwm_addr[9] = 2500; //CMPA. 50% duty cycle.
	pwm_addr[11] = 0x12;  // set when the counter = 0; clear when counter = CMPA
 	pwm_addr[12] = 0; //disable output B. 
 	pwm_addr[0] = 0xc0; // up count mode, software sync.
	printf("PWM TBCTL 0x%X\n", pwm_addr[0]); 
	printf("PWM TBSTS 0x%X\n", pwm_addr[1]); 
	printf("PWM TBCNT 0x%X\n", pwm_addr[4]); 
	printf("PWM TBPRD 0x%X (%d)\n", pwm_addr[5], pwm_addr[5]); 
	printf("PWM CMPCTL 0x%X\n", pwm_addr[7]); 
	printf("PWM CMPA 0x%X (%d)\n", pwm_addr[9], pwm_addr[9]);
	printf("PWM CMPB 0x%X (%d)\n", pwm_addr[10], pwm_addr[10]);  
	printf("PWM AQCTLA 0x%X\n", pwm_addr[11]); 
	//pwm_addr[9] = 200; //works!!  sets the duty cycle!

	//time a read-loop to assess speed
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
	motor_setPWM(0.01); 
	motor_forward(); 
	int sta = eqep.getPosition(); 
	sleep(1); 
	int dd = eqep.getPosition() - sta; 
	if(dd < 0){
	 printf("Motor polarity looks reversed, %d.  Check your wiring.\n", dd); 
	 cleanup(); 
	 return 0; 
	}else{
		printf("Motor forward direction looks OK.\n"); 
	}
	int fin = eqep.getPosition(); 
	motor_reverse(); 
	sta = eqep.getPosition(); 
	sleep(1); 
	dd = eqep.getPosition() - sta; 
	if(dd > 0){
	 printf("Motor polarity looks reversed, %d.  Check your wiring.\n", dd); 
	 cleanup(); 
	 return 0; 
	}else{
		printf("Motor reverse direction looks OK.\n"); 
	}
	sta = eqep.getPosition();  //top of cylinder
	motor_forward(); 
	sleep(1); 
	fin = eqep.getPosition(); //bottom of cylinder, retract. 
	
	//try a negative step ('up')
	//full-speed acceleration to midpoint of trajectory. 
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
	float* sav = (float*)malloc(sizeof(float) * 5 * 1e6); 
		//20MB .. should be more than enough. 
	int savn = 0; 
	auto update_velocity = [&] (int nn, float lerp) -> void {
		t = get_time(); 
		x = eqep.getPosition() - fin;
		if(nn == 0){
			x_old = x; 
			t_vold = t; 
		}else{
			if(x != x_old){
				//update the velocity. 
				float dt_ = t - t_vold;
				v_ = (float)(x - x_old) / dt_; 
				v = lerp*v_ + (1.0 - lerp)*v; //simple fading-memory filter 
					//(with shorter timeconstant at high speed)
				t_vold = t; 
				x_old = x;
			}
		}
	};
	auto save_dat = [&] (int nn) -> void {
		sav[savn*5+0] = t; 
		sav[savn*5+1] = (float)x; 
		sav[savn*5+2] = v; 
		sav[savn*5+3] = v_; 
		sav[savn*5+4] = dr; 
		savn++; 
	};
	for(int j=0; j<1; j++){
		timer_addr[0x44 / 4] = 0xffffffff; //reload (zero) the TCRR from the TLDR.
		//this may take a little bit ...
		n=0; 
		// compress the spring. 
		t = get_time(); 
		printf("start time %f\n", t); //dummy wait / syscall.
		t = get_time(); 
		update_velocity(0, 0.0);
		while(t < 0.1){
			update_velocity(n, 0.1);
			
			if(t < 0.008){
				dr = 1.0; //compress the spring down
			}else if(t < 0.016 || x > -1000){
				if(x > -300)
				dr = -1.0; //drive up
				else dr = -0.03; //coast up
			}else if(t < 0.030){
				if(v < -300*200)
					dr = 0.06; //decelerate down
				else 
					dr = 0.018; // decelerate less
			}else{
				dr = -0.004; //hold (up)
			}
			motor_setDrive(dr); 
			save_dat(n); 
			n++; 
		}
		//reset motor positon. 
		motor_setDrive(-0.006); 
		sleep(1); 
		motor_setDrive(0.015); 
		sleep(1); 
		fin = eqep.getPosition(); //bottom of cylinder, retract. 
		printf("fin %d\n", fin); 
	}
	FILE* dat_fd = fopen("pid.dat", "w"); 
	for(int j=0; j<savn; j++){
			for(int k=0; k<5; k++){
				fprintf(dat_fd, "%e\t", sav[j*5+k]); 
			fprintf(dat_fd, "\n"); 
		}
	};
	fclose(dat_fd); 
	free(sav); 

	cleanup(); 
	return 0;
}
