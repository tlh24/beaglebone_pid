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

//need to mmap the gpio registers as well. 
void map_gpio1_register(){
	int masked_address = 0x4804c000 & ~(getpagesize()-1);
	gpio_addr = (uint32_t*)mmap(NULL, PWM_BLOCK_LENGTH,
		PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, masked_address);
	if (gpio_addr == MAP_FAILED )
	{
		printf("Memory Mapping failed for 0x%04x register\n", masked_address);
		printf("ERROR: (errno %d)\n", errno);
		return;
	}
	printf("gpio at address 0x%08x mapped\n",
		masked_address);
}

void map_timer_register(){
	int masked_address = 0x48044000 & ~(getpagesize()-1);
	timer_addr = (uint32_t*)mmap(NULL, PWM_BLOCK_LENGTH,
		PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, masked_address);
	if (timer_addr == MAP_FAILED )
	{
		printf("Memory Mapping failed for 0x%04x register\n", masked_address);
		printf("ERROR: (errno %d)\n", errno);
		return;
	}
	printf("timer at address 0x%08x mapped\n",
		masked_address);
}

void motor_forward(){
	gpio_addr[0x190 / 4] = 0x1 << 19; //clear data out
	gpio_addr[0x194 / 4] = 0x1 << 16; //set data out
}
void motor_reverse(){
	gpio_addr[0x190 / 4] = 0x1 << 16; //clear data out
	gpio_addr[0x194 / 4] = 0x1 << 19; //set data out
}
void motor_stop(){
	gpio_addr[0x190 / 4] = 0x1 << 16; //clear data out
	gpio_addr[0x190 / 4] = 0x1 << 19; //clear data out
}
void motor_setPWM(float duty){
	pwm_addr[9] = (int)(duty * 20e3); 
	//see setup.sh -- 20kHz PWM cycle. 
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
	munmap(gpio_addr, PWM_BLOCK_LENGTH);
	munmap(timer_addr, PWM_BLOCK_LENGTH);
	close (mem_fd); 
}

int main (int argc, char const *argv[])
{
	struct timeval tv1, tv2;
	int eqep_num;
	uint32_t eqep_pos, eqep0_pos;

	if(argc < 2)
	{
		cout << "Usage: " << argv[0] << " 0|1|2" << endl;
		cout << "Requires the number for which eQEP to open\n";
		return 1;
	}

	if (strtol(argv[1],NULL,0) >= 0 && strtol(argv[1],NULL,0) <= 2) {
		eqep_num = strtol(argv[1],NULL,0);
	} else {
		cout << "Try again." << endl;
		return 1;
	}
  
	eQEP eqep(eqep_num);
	printf("base address (mmaped) 0x%X\n", eqep.getPWMSSPointer()); 
	printf("SYSCONFIG 0x%X\n", *(uint32_t*)(eqep.getPWMSSPointer() + PWM_SYSCONFIG));
	printf("CLKCONFIG 0x%X\n", *(uint32_t*)(eqep.getPWMSSPointer()+PWM_CLKCONFIG));
	printf("QEPCTL0   0x%X\n",  eqep.getControl());
	printf("QDECCTL0  0x%X\n",  eqep.getDecoderControl());
	printf("QEINT0    0x%X\n",  eqep.getInterruptEnable());
	printf("QUPRD0    0x%u\n", eqep.getUnitPeriod());
	printf("QPOSMAX0  0x%X\n", eqep.getMaxPos());
	printf("QEPSTS0   0x%X\n",  eqep.getStatus());

	// check the PWM module associated with this eQEP. 
	pwm_addr = (uint16_t*)(eqep.getPWMSSPointer() + 0x200); //got the offset from the device tree.
	printf("PWM TBCTL 0x%X\n", pwm_addr[0]); 
	printf("PWM TBSTS 0x%X\n", pwm_addr[1]); 
	printf("PWM TBCNT 0x%X\n", pwm_addr[4]); 
	printf("PWM TBPRD 0x%X (%d)\n", pwm_addr[5], pwm_addr[5]); 
	printf("PWM CMPCTL 0x%X\n", pwm_addr[6]); 
	printf("PWM CMPA 0x%X (%d)\n", pwm_addr[9], pwm_addr[9]);
	printf("PWM CMPB 0x%X (%d)\n", pwm_addr[10], pwm_addr[10]);  
	
	pwm_addr[9] = 200; //works!!  sets the duty cycle!

	mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mem_fd < 0)
	{
		printf("Can't open /dev/mem\n");
		return 1;
	}

	map_gpio1_register(); 
	printf("GPIO1_REV 0x%X\n", gpio_addr[0]); 
	printf("GPIO1_OE 0x%X\n", gpio_addr[0x134 / 4]); 
	printf("GPIO1_DI 0x%X\n", gpio_addr[0x138 / 4]); 
	motor_forward(); 

	map_timer_register(); 
	printf("TIMOER0_TIDR 0x%X\n", timer_addr[0]);
	printf("TIMOER0_TIOCP_CFG 0x%X\n", timer_addr[0x10 / 4]);
	printf("TIMOER0_TCLR 0x%X\n", timer_addr[0x38 / 4]); 
	printf("TIMOER0_TCRR 0x%X\n", timer_addr[0x3c / 4]);  //counter register. 
	printf("TIMOER0_TLDR 0x%X\n", timer_addr[0x40 / 4]); //load on overflow; defalut to zero. 
	timer_addr[0x10 / 4] = 0xa; //1010, smart-idle, emufree, no reset.
	timer_addr[0x44 / 4] = 0xffffffff; //reload (zero) the TCRR from the TLDR. 
	timer_addr[0x38 / 4] = 0x3 ; //0000 0000 0000 0011
	printf("TIMOER0_TCRR 0x%X\n", timer_addr[0x3c / 4]);  //counter register. 
	eqep0_pos = eqep;

	//time a read-loop to assess speed
	int num_reads = 1000000;
	int i;
	gettimeofday(&tv1,NULL);
	for(i=0;i<num_reads;i++){
		eqep_pos = eqep.getPosition();
	}
	gettimeofday(&tv2,NULL);
	int timer1_s = timer_addr[0x3c / 4]; 

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
	sta = eqep.getPosition(); 
	motor_stop(); 
	sleep(1); 

	//run PID loop.  
	float t = 0.f; 
	timer_addr[0x44 / 4] = 0xffffffff; //reload (zero) the TCRR from the TLDR.
	motor_forward(); 
	float oldcmd = 0.001; 
	float integral = 0.f; 
	int x_old = 0; 
	float t_old = 0.f; 
	float vel = 0.f; 
	int n = 0; 
	FILE* dat_fd = fopen("pid.dat", "w"); 
	while(t < 1.f){
		t = get_time(); 
		float targ = step_seg((float)(fin-sta) * 0.5f, 0.075f, t); 
		int x = eqep.getPosition() - sta;
		float dt = t - t_old; 
		t_old = t; 
		float d = targ - (float)x;
		float vel_ = (float)(x - x_old) / dt; 
		x_old = x; 
		//velocity filtering needs to be normalized to dt -- want the timeconstant to be about 2ms. 
		float dtv = dt < 5e-4? dt : 5e-4; 
		vel = vel_ + vel*(1.f - dtv / 5e-4); 
		float dti = dt < 1e-3? dt : 1e-3; 
		integral += d*(dti / 1e-3); //shorter integration timecostant.
		integral = integral* (1.f - dt / 0.02);  //integration fadeconstant.
		float newcmd = d * 0.06 + integral*1.5e-2 - vel * 1.5e-6; 
		if(newcmd < 0.f && oldcmd >= 0.f) motor_reverse(); 
		if(newcmd > 0.f && oldcmd <= 0.f) motor_forward();
		float drive = newcmd > 50.f ? 50.f : newcmd; 
		drive = drive < -50.f ? -50.f : drive; 
		motor_setPWM(fabs(drive * 0.02)); 
		oldcmd = newcmd; 
		if(n % 100 == 0){
			fprintf(dat_fd, "%f\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", t, x, targ, d*0.06, integral*1.5e-2, vel*2.2e-6, newcmd, drive*20.f); 
		}
		n++; 
	}
	fclose(dat_fd); 

	cleanup(); 
	return 0;
}
