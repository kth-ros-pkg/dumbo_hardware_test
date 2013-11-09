/*
 * powerarm.c
 *
 *  Created on: Oct 2005
 *  Authors:   Christian Smith
 *            ccs <at> kth.se 
 */

/* Copyright (c) 2005, Christian Smith, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Written by Christian Smith, Oct 2005*/
/* Adapted to Schunk dual arm setup by Francisco Vina, May 2011*/
/*
  Simple program to test the powercubes' CAN responses, and move arm. For Schunk arms. Inverse kinematics not appropriate for this arm - fix later?
 */
// gcc -Wall -O2  -D_REENTRANT -I../../include -L .. schunk_tester.c powercube_commands.c schunk_kinematics.c -o schunk_tester -lcanlib -lpthread -lcv -lcxcore -lhighgui -lm


#include <kvaser_canlib/canlib.h>
#include <stdio.h> 
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <dumbo_powercube_chain/powercube_defines.h>
#include <dumbo_powercube_chain/powercube_commands.h>
#include <dumbo_arm_test/schunk_kinematics.h>
#include <dumbo_arm_test/schunk_limits.h>
#include <string.h>
#include <unistd.h>
#include <termio.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <time.h>
//#include "dumbo_kinematics.h"

#ifndef PI
#define PI (3.1415926)
#endif

int i = 0;
int arm_select=-1;
char string_LR[2][6]={{'l','e','f','t','\0','\0'},
		{'r','i','g','h','t','\0'}};

unsigned char willExit = 0;
long this_cubes_id = 0;
static struct termio save_term;

void sighand (int sig) {
	static int last;
	switch (sig){
	case SIGINT:
		willExit = 1;
		alarm(0);
		break;
	case SIGALRM:
		if (i != last) printf("rx : %d total: %d\n", i-last, i);
		last = i;
		alarm(1);
		break;
	}
	return;
}

/*
  Sets the keyboard in 'raw' mode (no buffering)
  fd is a file pointer to the terminal we want to change
 */
void _set_keyboard_raw(int fd){
	struct termio s;
	(void)ioctl(fd, TCGETA, &s);
	save_term = s;
	s.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOK|ECHONL);
	s.c_oflag |= (OPOST|ONLCR|TAB3);
	s.c_oflag &= ~(OCRNL|ONOCR|ONLRET);
	s.c_cc[VMIN] = 1;
	s.c_cc[VTIME] = 0;
	(void)ioctl(fd, TCSETAW, &s);
}

/*
  Resets the keyboard from 'raw' mode
  fd is a file pointer to the terminal we want to change
 */
void _reset_keyboard(int fd){
	struct termio s;
	s = save_term;
	(void)ioctl(fd, TCSETAW, &s);
}


int main (int argc, char *argv[])
{
	struct module cube[8];
	float cubelimit_lower[7];
	float cubelimit_upper[7];
	int current_cube = 0;
	canHandle h;
	int ret = -1;
	int fd;
	long id;
	unsigned char msg[8];
	unsigned int dlc;
	unsigned int flag;
	unsigned long time;
	long int send_data1 = 1234;
	long int send_data2;
	float send_data1f = 1234.5;
	float send_data2f;
	unsigned short int send_datau16 = 1234;
	short int send_datai16 = -1234;
	float send_dataf = 1234.5;
	float ret_dataf;
	int channel = 0;
	int bitrate1M = BAUD_1M;
	int bitrate250k = BAUD_250K;
	int bitrate500k = BAUD_500K;
	int j,mod_number;
	struct timeval time_now;
	struct timezone tz;
	pthread_t watchdog_thread[6];
	struct timespec nsleep_time;
	struct timespec remain;
	FILE *read_log_file = fopen("logs/schunk_tester.log","a");
	unsigned char msgserial[8];
	unsigned long int serial;

	do{
		printf("Select Schunk arm.\n0 : left arm.\n1 : right arm.\n");
		scanf("%d",&arm_select);
	}while((arm_select!=SELECT_RIGHT_ARM)&&(arm_select!=SELECT_LEFT_ARM));

	for(i=0;i<7;i++){
		cubelimit_lower[i] = joint_limits_min[arm_select].angle[i];
		cubelimit_upper[i] = joint_limits_max[arm_select].angle[i];
	}

	//initialize positions
	errno = 0;

	/* Use sighand as our signal handler */
	signal(SIGALRM, sighand);
	signal(SIGINT, sighand);
	alarm(1);

	/* Allow signals to interrupt syscalls(in canReadBlock) */
	siginterrupt(SIGINT, 1);

	/* Open channels, parameters and go on bus */
	printf("Opening CAN cards\n");
	fflush(stdout);



	printf("canOpenChannel %d ", channel);
	fflush(stdout);

	for(i=0;i<8;i++){
		cube[i].canID = i+1;
	}

	msg[0] = PC_COMMAND_GET_EXTENDED;
	msg[1] = PC_PARA_DEF_CUBE_SERIAL;

	for(channel=0;channel<6;channel++){
		h = canOpenChannel(channel, canWANT_EXCLUSIVE |canWANT_EXTENDED);

		if (h < 0)
		{
			printf("CAN Open Channel %d failed\n", channel);
		}

		else
		{
			printf("CAN Open Channel %d succesful\n", channel);
			canSetBusParams(h, BAUD_500K,4,3,1,1,0);
			canBusOn(h);

			for(i=0;i<7;i++){
				cube[i].handle = h;
			}

			pc_request_value_from_module(cube[0],msg);
			mod_number = pc_listen_for_response(cube[0].handle,(void *)msgserial);
			get_data_uint32(msgserial,&serial);
			printf("Found serial number %d, mod_number %d\n",
					(int)serial,mod_number);

			switch(arm_select) {
			case(SELECT_LEFT_ARM):
			  printf("Trying to detect left arm (%u)\n",DEF_SCHUNK_LEFTARM_SERIAL_NUMBER);
			if((((int)serial)==DEF_SCHUNK_LEFTARM_SERIAL_NUMBER)&&(mod_number==1))
			{
				if((msgserial[0]!=PC_COMMAND_GET_EXTENDED)||(msgserial[1]!=PC_PARA_DEF_CUBE_SERIAL))
				{
					printf("Error (ack) detecting serial number of left arm.\n");
					exit(EXIT_FAILURE);
				}
				else
				{
					printf("Left arm found in CAN channel %d.\n",channel);
					channel = 20;
				}
			}
			else if(channel==6)
			{
				printf("Left arm not found.\n");
				return -1;
			}
			break;

			case(SELECT_RIGHT_ARM):
			  printf("Trying to detect right arm\n");
			if(((int)serial==DEF_SCHUNK_RIGHTARM_SERIAL_NUMBER)&&(mod_number==1))
			{
				if((msgserial[0]!=PC_COMMAND_GET_EXTENDED)||(msgserial[1]!=PC_PARA_DEF_CUBE_SERIAL))
				{
					printf("Error (ack) detecting serial number of right arm.\n");
					exit(EXIT_FAILURE);
				}
				else
				{
					printf("Right arm found in CAN channel %d.\n",channel);
					channel = 20;
				}
			}
			else if(channel==6)
			{
				printf("Right arm not found.\n");
			}
			break;
			}
		}
	}
	cube[7].handle = cube[0].handle;

	printf(" OK\n");
	//return 0;
	//canSetBusParams(h[channel], bitrate250k, 4, 3, 1, 1, 0);
	canSetBusParams(h, bitrate500k, 4, 3, 1, 1, 0);
	//canSetBusParams(h[channel], bitrate1M, 4, 3, 1, 1, 0);
	canBusOn(h);

	// make sure all modules start at 500K
	printf("Resetting PC baudrate %d....", j);
	fflush(stdout);

	//canSetBusParams(h[j], bitrate1M, 4, 3, 1, 1, 0);
	//pc_set_baud_all_on_bus(h[j], PC_BAUDRATE_1M);
	//canSetBusParams(h[j], bitrate500k, 4, 3, 1, 1, 0);
	pc_set_baud_all_on_bus(h, PC_BAUDRATE_500K);

	//pc_set_baud_all_on_bus(h[j], PC_BAUDRATE_250K);
	//canSetBusParams(h[j], bitrate250k, 4, 3, 1, 1, 0);

	printf("Done\n");


	// Initialize modules.
	printf("initializing modules:\n");
	for(j=0;j<7;j++){
		cube[j].canID=j+1;
		cube[j].handle = h;
	}

	printf("Initialization results:\n");
	for(j=0;j<7;j++){
		printf("  Cube %d has CAN channel %ld\n",j,cube[j].canID);
	}

	//return 1;

	i = 0;

	fd = 0; // stdin
	_set_keyboard_raw(fd);

	//pc_set_baud_all(h[0],h[1],h[2],h[3], PC_BAUDRATE_500K);

	/*
    for(channel=0;channel<4;channel++){
    canBusOff(h[channel]);
    canSetBusParams(h[channel], bitrate500k, 4, 3, 1, 1, 0);
    canBusOn(h[channel]);
    watchdog_thread[channel] = pc_start_watchdog(&(cube[channel]));
    }
	 */
	//pc_set_baud_all_on_bus(h1, PC_BAUDRATE_250K);
	//canSetBusParams(h1, bitrate250k, 4, 3, 1, 1, 0);


	// Speed and acc initialization
	/*for(j=0;j<7;j++){
    float fvalue;
    unsigned char nulmsg[8];
    msg[0]=PC_COMMAND_SET_EXTENDED;
    msg[1]=PC_PARA_TARGET_ACC;
    fvalue = 0.2;
    dlc = set_data_float(msg,&fvalue);
    pc_send_command_to_module(cube[j], msg, dlc);
    pc_listen_for_response(cube[j].handle,&nulmsg);
    msg[0]=PC_COMMAND_SET_EXTENDED;
    msg[1]=PC_PARA_TARGET_VEL;
    fvalue = 0.03;
    dlc = set_data_float(msg,&fvalue);
    pc_send_command_to_module(cube[j], msg, dlc);
    pc_listen_for_response(cube[j].handle,&nulmsg);
  } */


	while (!willExit){
		char inchar;
		printf("Enter command (module %d) (? for help) ",(int)cube[current_cube].canID);
		fflush(stdout);
		willExit = 1 - read(fd, &inchar, 1);
		printf("\n");
		fflush(stdout);
		switch(inchar){
		case '?':
			printf("   ?    - display this Help.\n");
			printf("<space> - HALT MODULES.\n");
			//printf("   c    - select Canbus #.\n");
			printf("   0-9    - go to pos 0-9.\n");
			printf("   a    - set velocity to 3 deg/s.\n");
			printf("   A    - set velocity to -3 deg/s.\n");
			printf("   b    - set Baudrate.\n");
			printf("   c    - Communication speed test.\n");
			printf("   C    - Communication speed test (with motion).\n");
			printf("   d    - Kill home switch.\n");
			printf("   e    - Move step extended.\n");
			printf("   f    - set speed Fast (set slow with reset).\n");
			printf("   F    - set speed FAST (set slow with reset).\n");
			printf("   g    - Get module status.\n");
			printf("   h    - Home module.\n");
			printf("   H    - Home all modules.\n");
			printf("   i    - set current to 1 A.\n");
			printf("   I    - set current to 0 A.\n");
			printf("   l    - set position Limits.\n");
			printf("   L    - set position Limits for all modules.\n");
			printf("   m    - send Motion command (position).\n");
			printf("   M    - send Motion command (position + time).\n");
			printf("   o    - select mOdule #.\n");
			printf("   p    - read Parameter value.\n");
			printf("   r    - Reset module.\n");
			printf("   R    - Reset module.\n");
			printf("   s    - Set module parameter.\n");
			printf("   t    - linear Tracking (straight).\n");
			printf("   T    - linear Tracking (diagonal).\n");
			printf("   v    - set Velocity and acc\n");
			printf("   V    - set Velocity and acc all\n");
			printf("   w    - Whole arm pos1\n");
			printf("   W    - Whole arm pos2\n");
			printf("   x    - get Xyz pta position.\n");
			printf("   x    - set Xyz pta position.\n");
			printf("   z    - set zero position of the module.");
			printf("   q    - Quit.\n");
			printf("\n");
			break;

		case 'i':
			if (current_cube>0){  // dangerous to test with larger cubes!!
				int overrun_counter = 0;
				float set_cur = 1.0;
				float cur_pos=-1;
				unsigned char ret_msg[8];
				msg[0] = PC_COMMAND_SET_MOTION;
				msg[1] = PC_MOTION_FCUR_ACK;
				dlc = set_data_float(msg, &set_cur);
				pc_send_command_to_module(cube[current_cube], msg, dlc);
				pc_listen_for_response(cube[current_cube].handle,&ret_msg);
				get_data_float(ret_msg, &cur_pos);
				printf("Response from module is:\n");
				printf("ComID:%#02X, MotID:%#02X, pos:%f\n", ret_msg[0],ret_msg[1], cur_pos*180/PI);
				if(msg[6]&PC_ACK_SHORT_NOT_OK){
					printf("ERROR REPORTED! [%#02X] (press g for details)\n",ret_msg[6]);
				}
				while(cur_pos<0){
					unsigned char temp_msg[8];
					temp_msg[0] = PC_COMMAND_GET_EXTENDED;
					temp_msg[1] = PC_PARA_CUR;
					pc_request_value_from_module(cube[current_cube], temp_msg);
					pc_listen_for_response(cube[current_cube].handle,&ret_msg);
					get_data_float(ret_msg, &cur_pos);
					printf(" current : %f",cur_pos);
					msg[0] = PC_COMMAND_GET_EXTENDED;
					msg[1] = PC_PARA_ACT_POS;
					pc_request_value_from_module(cube[current_cube], msg);
					pc_listen_for_response(cube[current_cube].handle,&ret_msg);
					get_data_float(ret_msg, &cur_pos);
					printf(" pos : %f \n",cur_pos);
					if(overrun_counter++>2000) cur_pos = 1;
				}
				msg[0] = PC_COMMAND_GET_EXTENDED;
				msg[1] = PC_PARA_IPOL_VEL;
				pc_request_value_from_module(cube[current_cube], msg);
				pc_listen_for_response(cube[current_cube].handle,&ret_msg);
				get_data_float(ret_msg, &cur_pos);
				printf("Response from module is:\n");
				printf("ComID:%#02X, MotID:%#02X, vel:%f\n", ret_msg[0],ret_msg[1], cur_pos*180/PI);
				if(ret_msg[6]&PC_ACK_SHORT_NOT_OK){
					printf("ERROR REPORTED! [%#02X] (press g for details)\n",msg[6]);
				}
				msg[0] = PC_COMMAND_GET_EXTENDED;
				msg[1] = PC_PARA_CUR;
				pc_request_value_from_module(cube[current_cube], msg);
				pc_listen_for_response(cube[current_cube].handle,&ret_msg);
				get_data_float(ret_msg, &cur_pos);
				printf("Response from module is:\n");
				printf("ComID:%#02X, MotID:%#02X, cur:%f\n", ret_msg[0],ret_msg[1], cur_pos);
				if(ret_msg[6]&PC_ACK_SHORT_NOT_OK){
					printf("ERROR REPORTED! [%#02X] (press g for details)\n",msg[6]);
				}
				msg[0] = PC_COMMAND_SET_MOTION;
				msg[1] = PC_MOTION_FCUR_ACK;
				set_cur = 0.0;
				dlc = set_data_float(msg, &set_cur);
				pc_send_command_to_module(cube[current_cube], msg, dlc);
				pc_listen_for_response(cube[current_cube].handle,&ret_msg);
				get_data_float(ret_msg, &cur_pos);
				printf("Response from module is:\n");
				printf("ComID:%#02X, MotID:%#02X, pos:%f\n", ret_msg[0],ret_msg[1], cur_pos*180/PI);
				if(msg[6]&PC_ACK_SHORT_NOT_OK){
					printf("ERROR REPORTED! [%#02X] (press g for details)\n",ret_msg[6]);
				}
				break;
			}

		case 'I':{
			float cur_pos;
			float set_cur = 0;
			unsigned char ret_msg[8];
			msg[0] = PC_COMMAND_SET_MOTION;
			msg[1] = PC_MOTION_FCUR_ACK;
			dlc = set_data_float(msg, &set_cur);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle,&msg);
			get_data_float(msg, &cur_pos);
			printf("Response from module is:\n");
			printf("ComID:%#02X, MotID:%#02X, pos:%f\n", msg[0],msg[1], cur_pos*180/PI);
			if(msg[6]&PC_ACK_SHORT_NOT_OK){
				printf("ERROR REPORTED! [%#02X] (press g for details)\n",msg[6]);
			}
			break;
		}
		case 'a':
		{  // dangerous to test with larger cubes!!
			float set_vel = 3*PI/180;
			float cur_pos;
			unsigned char ret_msg[8];
			msg[0] = PC_COMMAND_SET_MOTION;
			msg[1] = PC_MOTION_FVEL_ACK;
			dlc = set_data_float(msg, &set_vel);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle,&msg);
			get_data_float(msg, &cur_pos);
			printf("Response from module is:\n");
			printf("ComID:%#02X, MotID:%#02X, pos:%f\n", msg[0],msg[1], cur_pos*180/PI);
			if(msg[6]&PC_ACK_SHORT_NOT_OK){
				printf("ERROR REPORTED! [%#02X] (press g for details)\n",msg[6]);
			}
			break;
		}

		case 'A':{
			float cur_pos;
			float set_vel = -3*PI/180;
			unsigned char ret_msg[8];
			msg[0] = PC_COMMAND_SET_MOTION;
			msg[1] = PC_MOTION_FVEL_ACK;
			dlc = set_data_float(msg, &set_vel);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle,&msg);
			get_data_float(msg, &cur_pos);
			printf("Response from module is:\n");
			printf("ComID:%#02X, MotID:%#02X, pos:%f\n", msg[0],msg[1], cur_pos*180/PI);
			if(msg[6]&PC_ACK_SHORT_NOT_OK){
				printf("ERROR REPORTED! [%#02X] (press g for details)\n",msg[6]);
			}
			break;
		}

		case 'F':{
			for(j=0;j<7;j++){
				float accfactor[] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0};
				float fvalue;
				unsigned char nulmsg[8];
				msg[0]=PC_COMMAND_SET_EXTENDED;
				msg[1]=PC_PARA_TARGET_ACC;
				fvalue = 100*PI/180*accfactor[j];
				printf("Setting very high speed for module %d  (%f  deg/s2).\n",j+1,fvalue);
				dlc = set_data_float(msg,&fvalue);
				pc_send_command_to_module(cube[j], msg, dlc);
				pc_listen_for_response(cube[j].handle,&nulmsg);
				msg[0]=PC_COMMAND_SET_EXTENDED;
				msg[1]=PC_PARA_TARGET_VEL;
				fvalue = 200*PI/180;
				dlc = set_data_float(msg,&fvalue);
				pc_send_command_to_module(cube[j], msg, dlc);
				pc_listen_for_response(cube[j].handle,&nulmsg);
			}
			break;
		}
		case 'f':{
			for(j=0;j<7;j++){
				float fvalue;
				unsigned char nulmsg[8];
				printf("Setting high speed for module %d.\n",j+1);
				msg[0]=PC_COMMAND_SET_EXTENDED;
				msg[1]=PC_PARA_TARGET_ACC;
				fvalue = 100*PI/180;
				dlc = set_data_float(msg,&fvalue);
				pc_send_command_to_module(cube[j], msg, dlc);
				pc_listen_for_response(cube[j].handle,&nulmsg);
				msg[0]=PC_COMMAND_SET_EXTENDED;
				msg[1]=PC_PARA_TARGET_VEL;
				fvalue = 100*PI/180;
				dlc = set_data_float(msg,&fvalue);
				pc_send_command_to_module(cube[j], msg, dlc);
				pc_listen_for_response(cube[j].handle,&nulmsg);
			}
			break;
		}
		case 'X':{
			printf("Setting pos not implemented yet... \n");
			break;
		}
		case 'x':{
			printf("Current position NOT IMPLEMENTED:\n");
			//print_pos_euler(cur_pose);
			break;
		}
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
		{
			printf("Command not yet implemented.\n");
		}
		break;
		case 'w':{
			printf("Command not yet implemented.\n");
			break;
		}
		case 'W':{
			printf("Command not yet implemented.\n");
			break;
		}

		case 'T':{
			printf("Command not yet implemented.\n");
			break;
		}

		case 't':{
			printf("Command not yet implemented.\n");
			break;
		}

		case 'C':{
			int counter1;
			unsigned char msg1[8];
			unsigned char msg2[8];
			int usecs = 0;
			int secs = 0;
			int ntest = 10000;
			long int dlc1;
			float new_pos = 10.0;
			int temp_cube;
			temp_cube = current_cube;
			msg1[0] = PC_COMMAND_SET_MOTION;
			msg1[1] = PC_MOTION_FRAMP_ACK;
			printf("Starting moving speed test...");
			fflush(stdout);
			gettimeofday(&time_now,&tz);
			usecs = time_now.tv_usec;
			secs = time_now.tv_sec;

			for(counter1=0;counter1<ntest;counter1++){
				new_pos = 10.0 - (20.0*(ntest-counter1))/(ntest*1.0);
				if(new_pos>10) new_pos=10.0;
				if(new_pos<(-10)) new_pos=-10.0;
				new_pos = new_pos*PI/180;
				msg1[0] = PC_COMMAND_SET_MOTION;
				msg1[1] = PC_MOTION_FRAMP_ACK;
				dlc1 = set_data_float(msg1, &new_pos);
				for(temp_cube = 0; temp_cube < 3; temp_cube++){
					pc_send_command_to_module(cube[temp_cube], msg1, dlc1);
				}
				for(temp_cube = 0; temp_cube < 3; temp_cube++){
					pc_listen_for_response(cube[temp_cube].handle, &msg2);
					/*	  msg1[0] = PC_COMMAND_GET_EXTENDED;
		  msg1[1] = PC_PARA_IPOL_VEL;
		  pc_request_value_from_module(cube[temp_cube],msg1);
		  pc_listen_for_response(cube[temp_cube].handle, &msg2);*/
				}
			}
			gettimeofday(&time_now,&tz);
			printf("%d request/response-pairs took %ld ms. (=%f Hz)\n",ntest,((time_now.tv_usec-usecs)+(time_now.tv_sec-secs)*1000000)/1000, 1000.0*(float)ntest/((float)((time_now.tv_usec-usecs)+(time_now.tv_sec-secs)*1000000)/1000));
			fflush(stdout);
			break;
		}
		case 'c':{
			int counter1;
			unsigned char msg1[8];
			unsigned char msg2[8];
			int usecs = 0;
			int secs = 0;
			int ntest = 10000;
			long int dlc1;
			msg1[0]=PC_COMMAND_GET_EXTENDED;
			msg1[1]=PC_PARA_CONFIG;
			printf("Starting speed test...");
			fflush(stdout);
			gettimeofday(&time_now,&tz);
			usecs = time_now.tv_usec;
			secs = time_now.tv_sec;
			for(counter1=0;counter1<ntest;counter1++){
				pc_request_value_from_module(cube[current_cube], msg1);
				pc_listen_for_response(cube[current_cube].handle, &msg2);
			}
			gettimeofday(&time_now,&tz);
			printf("%d request/response-pairs took %ld ms.\n",ntest,((time_now.tv_usec-usecs)+(time_now.tv_sec-secs)*1000000)/1000);
			fflush(stdout);
			break;
		}
		case 'l':{
			unsigned char ret_msg[8];
			int correct_input = 0;
			float lower_limit;
			float upper_limit;
			unsigned char input_msg_buff[80];

			_reset_keyboard(fd);
			while(!correct_input && !willExit){
				printf("Enter min pos limit [deg]: ");
				fgets(input_msg_buff,80,stdin);
				lower_limit = (float)(strtod(input_msg_buff,NULL));
				if(!((lower_limit>180) || (lower_limit<-180))){
					correct_input = 1;
				}
			}
			if(willExit) break;

			correct_input = 0;
			while(!correct_input && !willExit){
				printf("Enter max pos limit [deg]: ");
				fgets(input_msg_buff,80,stdin);
				upper_limit = (float)(strtod(input_msg_buff,NULL));
				if(!((upper_limit>180) || (upper_limit<lower_limit))){
					correct_input = 1;
				}
			}
			if(willExit) break;
			lower_limit = lower_limit*PI/180;
			upper_limit = upper_limit*PI/180;

			msg[0]=PC_COMMAND_SET_EXTENDED;
			msg[1]=PC_PARA_MIN_POS;
			dlc = set_data_float(msg,&lower_limit);
			pc_send_command_to_module(cube[current_cube], msg, dlc);

			ret_msg[2]=0;
			pc_listen_for_response(cube[current_cube].handle, &ret_msg);
			if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
					ret_msg[1]==PC_PARA_MIN_POS &&
					ret_msg[2]==0X64)){
				printf("Failed! (none/incorrect response from cube)\n");
			}else{
				printf("OK...");
			}

			msg[1]=PC_PARA_MAX_POS;
			dlc = set_data_float(msg,&upper_limit);
			pc_send_command_to_module(cube[current_cube], msg, dlc);


			ret_msg[2]=0;
			pc_listen_for_response(cube[current_cube].handle, &ret_msg);
			if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
					ret_msg[1]==PC_PARA_MAX_POS &&
					ret_msg[2]==0X64)){
				printf("Failed! (none/incorrect response from cube)\n");
			}else{
				printf("OK\n");
			}
			_set_keyboard_raw(fd);
			break;
		}
		case 'L':{
			unsigned char ret_msg[8];
			int correct_input = 0;
			float lower_limit;
			float upper_limit;

			for(j=0;j<7;j++){
				lower_limit = cubelimit_lower[j];
				upper_limit = cubelimit_upper[j];

				msg[0]=PC_COMMAND_SET_EXTENDED;
				msg[1]=PC_PARA_MIN_POS;
				dlc = set_data_float(msg,&lower_limit);
				pc_send_command_to_module(cube[j], msg, dlc);

				ret_msg[2]=0;
				pc_listen_for_response(cube[j].handle, &ret_msg);
				if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
						ret_msg[1]==PC_PARA_MIN_POS &&
						ret_msg[2]==0X64)){
					printf("Failed! (none/incorrect response from cube)\n");
				}else{
					printf("OK...");
				}

				msg[1]=PC_PARA_MAX_POS;
				dlc = set_data_float(msg,&upper_limit);
				pc_send_command_to_module(cube[j], msg, dlc);


				ret_msg[2]=0;
				pc_listen_for_response(cube[j].handle, &ret_msg);
				if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
						ret_msg[1]==PC_PARA_MAX_POS &&
						ret_msg[2]==0X64)){
					printf("Failed! (none/incorrect response from cube)\n");
				}else{
					printf("OK\n");
				}
			}
			break;
		}

		case 's':{
			unsigned char ret_msg[8];
			int correct_input = 0;
			unsigned char par_num;
			unsigned char input_msg_buff[80];
			float fvalue = 0;
			unsigned long int u32value = 0;
			short int i16value= 0;

			_reset_keyboard(fd);
			while(!correct_input && !willExit){
				printf("Enter (HEX/DEC) number of parameter to set: ");
				fgets(input_msg_buff,80,stdin);
				par_num = (unsigned char)(strtol(input_msg_buff,NULL,0));
				if(par_num <= 0X59){
					correct_input = 1;
				}
			}
			if(willExit) break;

			msg[0]=PC_COMMAND_SET_EXTENDED;
			msg[1]=par_num;
			printf("Setting parameter %#02X [ %s ]\n",par_num,pc_par2str(par_num));
			switch(pc_get_data_type(PC_COMMAND_SET_EXTENDED,par_num)){
			case PC_DATA_TYPE_FLOAT:
				printf("Enter floating point value: ");
				fgets(input_msg_buff,80,stdin);
				fvalue = (float)(strtod(input_msg_buff,NULL));
				dlc = set_data_float(msg,&fvalue);
				break;
			case PC_DATA_TYPE_INT16:
				printf("Enter int16 value: ");
				fgets(input_msg_buff,80,stdin);
				i16value = (short int)(strtol(input_msg_buff,NULL,0));
				dlc = set_data_int16(msg,&i16value);
				break;
			case PC_DATA_TYPE_UINT32:
				printf("Enter uint32 value: ");
				fgets(input_msg_buff,80,stdin);
				u32value = (unsigned long int)(strtol(input_msg_buff,NULL,0));
				dlc = set_data_uint32(msg,&u32value);
				break;
			default:
				printf("Illegal parameter type. Setting read-only value?\n");
				dlc = 0;
				break;
			}
			printf("Setting value %f....",(float)i16value+(float)u32value+fvalue);

			pc_send_command_to_module(cube[current_cube], msg, dlc);
			ret_msg[2]=0;
			pc_listen_for_response(cube[current_cube].handle, &ret_msg);
			if(!(ret_msg[0]==PC_COMMAND_SET_EXTENDED &&
					ret_msg[1]==par_num &&
					ret_msg[2]==0X64)){
				printf("Failed! (none/incorrect response from cube)\n");
			}else{
				printf("OK\n");
			}
			_set_keyboard_raw(fd);
			break;
		}

		case 'p':{
			unsigned char ret_msg[8];
			int correct_input = 0;
			unsigned char par_num;
			unsigned char input_msg_buff[80];
			_reset_keyboard(fd);
			while(!correct_input && !willExit){
				printf("Enter (HEX/DEC) number of parameter to read: ");
				fgets(input_msg_buff,80,stdin);
				par_num = (unsigned char)(strtol(input_msg_buff,NULL,0));
				if(par_num <= 0X59){
					correct_input = 1;
				}
			}
			if(willExit) break;

			printf("Reading parameter %#02X [ %s ]\n",par_num,pc_par2str(par_num));
			msg[0]=PC_COMMAND_GET_EXTENDED;
			msg[1]=par_num;
			pc_request_value_from_module(cube[current_cube], msg);
			pc_listen_for_response(cube[current_cube].handle, &ret_msg);
			pc_display_message(ret_msg);
			printf("\n");
			_set_keyboard_raw(fd);
			break;
		}
		case 'h':
			printf("Homing module %d.\n",current_cube+1);
			pc_home_module(cube[current_cube]);
			pc_listen_for_response(cube[current_cube].handle,&msg);
			break;
		case 'H':
			for(j=0;j<7;j++){
				printf("Homing module %d.\n",j+1);
				pc_home_module(cube[j]);
				pc_listen_for_response(cube[j].handle,&msg);
			}
			break;
		case 'r':
			printf("Resetting module %d.\n",current_cube+1);
			pc_reset_module(cube[current_cube]);
			pc_listen_for_response(cube[current_cube].handle,&msg);
			break;
		case 'd':
		{
			unsigned long int __uvalue;
			unsigned char ret_msg[8];
			printf("Killing home switch for module %d.\n",current_cube+1);
			msg[0]=PC_COMMAND_GET_EXTENDED;
			msg[1]=57;
			pc_request_value_from_module(cube[current_cube], msg);
			pc_listen_for_response(cube[current_cube].handle, &ret_msg);
			get_data_uint32(ret_msg,&__uvalue);
			__uvalue = __uvalue & ~PC_CONFIGID_MOD_SWR_ENABLED;
			msg[0]=PC_COMMAND_SET_EXTENDED;
			msg[1]=57;
			dlc = set_data_uint32(msg,&__uvalue);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle, &ret_msg);
			break;
		}
		case 'R':
		{
			int jj;
			for(jj=0;jj<7;jj++){
				float fvalue;
				unsigned char nulmsg[8];
				printf("Resetting module %d.\n",jj+1);
				pc_reset_module(cube[jj]);
				pc_listen_for_response(cube[jj].handle,&msg);
				msg[0]=PC_COMMAND_SET_EXTENDED;
				msg[1]=PC_PARA_TARGET_ACC;
				fvalue = 0.5;
				dlc = set_data_float(msg,&fvalue);
				pc_send_command_to_module(cube[jj], msg, dlc);
				pc_listen_for_response(cube[jj].handle,&nulmsg);
				msg[0]=PC_COMMAND_SET_EXTENDED;
				msg[1]=PC_PARA_TARGET_VEL;
				fvalue = 0.1;
				dlc = set_data_float(msg,&fvalue);
				pc_send_command_to_module(cube[jj], msg, dlc);
				pc_listen_for_response(cube[jj].handle,&nulmsg);

				// Empty CAN queue
				printf("If queue was empty, next line is error: (should be)\n");
				pc_listen_for_response(cube[jj].handle,&nulmsg);
			}
		}
		break;

		case 'm':{
			char input_msg_buff[80];
			int correct_input = 0;
			float new_pos;
			float cur_pos;
			float cur_vel;
			float cur_curr;
			char state;
			char dio_state;
			unsigned long int cube_state;
			unsigned char ret_msg[8];
			msg[0] = PC_COMMAND_GET_EXTENDED;

			msg[1] = PC_PARA_ACT_POS;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_pos);

			msg[1] = PC_PARA_ACT_VEL;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_vel);

			msg[1] = PC_PARA_CUR;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_curr);

			msg[1] = PC_PARA_CUBE_STATE;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_uint32(ret_msg,&cube_state);

			printf("Current module number is %d\n", current_cube+1);
			if(current_cube==7){
				printf("Pos = %f m, ", cur_pos);
				printf("Vel = %f m/s, ", cur_vel);
				printf("Curr = %f A\n", cur_curr);
			}else{
				printf("Pos = %f deg, ", 180*cur_pos/PI);
				printf("Vel = %f deg/s, ", 180*cur_vel/PI);
				printf("Curr = %f A\n", cur_curr);
			}
			_reset_keyboard(fd);
			while(!correct_input && !willExit){
				if(current_cube==7){
					printf("Enter new pos {0, 60}[mm]: ");
				}else{
					printf("Enter new angle {-180, 180}[deg]: ");
				}
				fgets(input_msg_buff,80,stdin);
				new_pos = (float)(strtod(input_msg_buff,NULL));
				if((new_pos >= -180) && (new_pos <= 180)){
					correct_input = 1;
				}
			}
			if(willExit) break;
			_set_keyboard_raw(fd);
			if(current_cube==7){
				printf("Setting grip to %f mm\n", new_pos);
				new_pos = new_pos/1000;
			}else{
				printf("Setting angle to %f deg\n", new_pos);
				new_pos = (new_pos/180)*PI;
			}

			msg[0] = PC_COMMAND_SET_MOTION;
			msg[1] = PC_MOTION_FRAMP_ACK;
			dlc = set_data_float(msg, &new_pos);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle,&msg);
			get_data_float(msg, &cur_pos);
			printf("Response from module is:\n");
			printf("ComID:%#02X, MotID:%#02X, Pos:%f\n", msg[0],msg[1], cur_pos*180/PI);
			if(msg[6]&PC_ACK_SHORT_NOT_OK){
				printf("ERROR REPORTED! [%#02X] (press g for details)\n",msg[6]);
			}
			break;
		}
		case 'M':{
			char input_msg_buff[80];
			int correct_input = 0;
			float new_pos;
			unsigned short int new_time;
			float cur_pos;
			float cur_vel;
			float cur_curr;
			char state;
			char dio_state;
			unsigned long int cube_state;
			unsigned char ret_msg[8];
			msg[0] = PC_COMMAND_GET_EXTENDED;

			msg[1] = PC_PARA_ACT_POS;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_pos);

			msg[1] = PC_PARA_ACT_VEL;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_vel);

			msg[1] = PC_PARA_CUR;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_curr);

			msg[1] = PC_PARA_CUBE_STATE;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_uint32(ret_msg,&cube_state);

			printf("Current module number is %d\n", current_cube+1);
			printf("Pos = %f deg, ", 180*cur_pos/PI);
			printf("Vel = %f deg/s, ", 180*cur_vel/PI);
			printf("Curr = %f A\n", cur_curr);

			_reset_keyboard(fd);
			while(!correct_input && !willExit){
				printf("Enter new angle {-180, 180}[deg]: ");
				fgets(input_msg_buff,80,stdin);
				new_pos = (float)(strtod(input_msg_buff,NULL));
				if((new_pos >= -180) && (new_pos <= 180)){
					correct_input = 1;
				}
			}
			if(willExit) break;
			correct_input = 0;
			while(!correct_input && !willExit){
				printf("Enter target time {100-10000}[ms]: ");
				fgets(input_msg_buff,80,stdin);
				new_time = (unsigned short int)(strtod(input_msg_buff,NULL));
				if((new_time >= 100) && (new_time <= 10000)){
					correct_input = 1;
				}
			}
			if(willExit) break;
			_set_keyboard_raw(fd);
			printf("Setting angle to %f degrees in %d ms\n", new_pos,new_time);
			new_pos = (new_pos/180)*PI;

			msg[0] = PC_COMMAND_SET_MOTION;
			msg[1] = PC_MOTION_FSTEP_ACK;
			dlc = set_data_float_uint16(msg, &new_pos, &new_time);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle,&msg);
			get_data_float(msg, &cur_pos);
			printf("Response from module is:\n");
			printf("ComID:%#02X, MotID:%#02X, Pos:%f\n", msg[0],msg[1], cur_pos*180/PI);
			if(msg[6]&PC_ACK_SHORT_NOT_OK){
				printf("ERROR REPORTED! [%#02X] (press g for details)\n",msg[6]);
			}
			break;
		}


		case 'v':{
			char input_msg_buff[80];
			int correct_input = 0;
			float new_vel;
			float max_vel;
			float max_acc;
			float new_acc;
			char state;
			char dio_state;
			unsigned long int cube_state;
			unsigned char ret_msg[8];
			msg[0] = PC_COMMAND_GET_EXTENDED;

			msg[1] = PC_PARA_MAX_VEL;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&max_vel);

			msg[1] = PC_PARA_MAX_ACC;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&max_acc);

			msg[1] = PC_PARA_CUBE_STATE;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_uint32(ret_msg,&cube_state);

			printf("Current module number is %d\n", current_cube+1);
			printf("MaxVel = %f deg/s, ", 180*max_vel/PI);
			printf("MaxAcc = %f deg/s2\n", 180*max_acc/PI);

			_reset_keyboard(fd);
			while(!correct_input && !willExit){
				printf("Enter new vel {0, %f}[deg/s]: ",180*max_vel/PI);
				fgets(input_msg_buff,80,stdin);
				new_vel = (float)(strtod(input_msg_buff,NULL));
				if((new_vel >= 0) && (new_vel <= 180*max_vel/PI)){
					correct_input = 1;
				}
			}
			if(willExit) break;
			correct_input = 0;

			while(!correct_input && !willExit){
				printf("Enter new acc {0, %f}[deg/s]: ",180*max_acc/PI);
				fgets(input_msg_buff,80,stdin);
				new_acc = (float)(strtod(input_msg_buff,NULL));
				if((new_acc >= 0) && (new_acc <= 180*max_acc/PI)){
					correct_input = 1;
				}
			}

			if(willExit) break;

			_set_keyboard_raw(fd);
			new_vel = (new_vel/180)*PI;
			new_acc = (new_acc/180)*PI;

			msg[0] = PC_COMMAND_SET_EXTENDED;
			msg[1] = PC_PARA_TARGET_VEL;
			dlc = set_data_float(msg, &new_vel);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle,&msg);

			msg[0] = PC_COMMAND_SET_EXTENDED;
			msg[1] = PC_PARA_TARGET_ACC;
			dlc = set_data_float(msg, &new_acc);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle,&msg);

			break;
		}
		case 'g':{
			float cur_pos=1337.0;
			float cur_vel=1337.0;
			float cur_curr=1337.0;
			unsigned long int cube_state=0.0;
			unsigned char ret_msg[8];
			//for(i=0;i<8;i++){
			//  ret_msg[i]=0;
			//}
			msg[0] = PC_COMMAND_GET_EXTENDED;

			msg[1] = PC_PARA_ACT_POS;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_pos);

			msg[1] = PC_PARA_IPOL_VEL;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_vel);

			msg[1] = PC_PARA_CUR;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_curr);

			msg[1] = PC_PARA_CUBE_STATE;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_uint32(ret_msg,&cube_state);

			printf("Current module number is %d\n", current_cube+1);
			printf("Current module pos is %f deg\n", 180*cur_pos/PI);
			printf("Current module vel is %f deg/s\n", 180*cur_vel/PI);
			printf("Current module current is %f A\n", cur_curr);
			printf("The following state flags (see manual) are set:\n");
			if(cube_state & PC_STATE_ERROR)
				printf("GENERAL ERROR! (bit 0)\n");
			if(cube_state & PC_STATE_HOME_OK)
				printf("HOMING SUCCESFUL! (bit 1)\n");
			if(cube_state & PC_STATE_HALTED)
				printf("MODULE HALTED! (bit 2)\n");
			if(cube_state & PC_STATE_POWERFAULT)
				printf("POWER FAULT! (bit 3)\n");

			if(cube_state & PC_STATE_TOW_ERROR)
				printf("TOW ERROR! (bit 4)\n");
			if(cube_state & PC_STATE_COMM_ERROR)
				printf("COMM ERROR! (bit 5)\n");
			if(cube_state & PC_STATE_SWR)
				printf("HOME SWITCH (bit 6)\n");
			if(cube_state & PC_STATE_SW1)
				printf("LIMIT 1 (bit 7)\n");

			if(cube_state & PC_STATE_SW2)
				printf("LIMIT 2 (bit 8)\n");
			if(cube_state & PC_STATE_BRAKE_ACTIVE)
				printf("BRAKE ACTIVE (bit 9)\n");
			if(cube_state & PC_STATE_CUR_LIMIT)
				printf("CURRENT LIMIT REACHED (bit 10)\n");
			if(cube_state & PC_STATE_MOTION)
				printf("IN MOTION (bit 11)\n");

			if(cube_state & PC_STATE_RAMP_ACC)
				printf("RAMP ACCELERATE (bit 12)\n");
			if(cube_state & PC_STATE_RAMP_STEADY)
				printf("RAMP STEADY (bit 13)\n");
			if(cube_state & PC_STATE_RAMP_DEC)
				printf("RAMP DECELERATE (bit 14)\n");
			if(cube_state & PC_STATE_RAMP_END)
				printf("RAMP ENDED (bit 15)\n");

			if(cube_state & PC_STATE_IN_PROGRESS)
				printf("STEP IN PROGRESS (bit 16)\n");
			if(cube_state & PC_STATE_FULLBUFFER)
				printf("STEP BUFFER FULL (bit 17)\n");
			if(cube_state & PC_STATE_POW_VOLT_ERROR)
				printf("VOLTAGE ERROR! (bit 18)\n");
			if(cube_state & PC_STATE_POW_FET_TEMP)
				printf("TRANSISTOR OVERHEAT! (bit 19)\n");

			if(cube_state & PC_STATE_POW_WDG_TEMP)
				printf("MOTOR OVERHEAT! (bit 20)\n");
			if(cube_state & PC_STATE_POW_SHORT_CURR)
				printf("SHORT CIRCUIT! (bit 21)\n");
			if(cube_state & PC_STATE_POW_HALLERR)
				printf("HALL EFFECT SENSOR ERROR! (bit 22)\n");
			if(cube_state & PC_STATE_POW_INTEGRAL_ERR)
				printf("INTEGRAL ERROR! (bit 23)\n");

			if(cube_state & PC_STATE_CPU_OVERLOAD)
				printf("CPU OVERLOAD! (FATAL!) (bit 24)\n");
			if(cube_state & PC_STATE_BEYOND_HARD)
				printf("HARD LIMIT PASSED! (bit 25)\n");
			if(cube_state & PC_STATE_BEYOND_SOFT)
				printf("SOFT LIMIT PASSED! (bit 26)\n");
			if(cube_state & PC_STATE_POW_SETUP_ERR)
				printf("CURRENT SETUP ERROR! (FATAL!) (bit 27)\n");

			break;
		}
		case 'b':{
			unsigned char inchar2[2];
			int baud_number = 0;
			inchar2[1]=0;
			_reset_keyboard(fd);
			printf("Enter baud number (1=250K 2=500K 3=1M): ");
			while(!((inchar2[0]=getchar())<'4' && inchar2[0]>='1')){
				printf("Enter baud number (1=250K 2=500K 3=1M): ");
			}
			getchar();  // removes trailing return
			_set_keyboard_raw(fd);
			baud_number=atoi(inchar2);
			baud_number=((baud_number-1)%3)+1;
			switch(baud_number){
			case 1:
				pc_set_baud_all_on_bus(cube[current_cube].handle, PC_BAUDRATE_250K);
				canBusOff(cube[current_cube].handle);
				canSetBusParams(cube[current_cube].handle, bitrate250k, 4, 3, 1, 1, 0);
				canBusOn(cube[current_cube].handle);
				printf("Current baud rate is now: 250K\n");
				break;
			case 2:
				pc_set_baud_all_on_bus(cube[current_cube].handle, PC_BAUDRATE_500K);
				canBusOff(cube[current_cube].handle);
				canSetBusParams(cube[current_cube].handle, bitrate500k, 4, 3, 1, 1, 0);
				canBusOn(cube[current_cube].handle);
				printf("Current baud rate is now: 500K\n");
				break;
			case 3:
				pc_set_baud_all_on_bus(cube[current_cube].handle, PC_BAUDRATE_1M);
				canBusOff(cube[current_cube].handle);
				canSetBusParams(cube[current_cube].handle, bitrate1M, 4, 3, 1, 1, 0);
				canBusOn(cube[current_cube].handle);
				printf("Current baud rate is now: 1M\n");
				break;
			default:
				printf("Baud resetting error - invalid baudrate");
				break;
			}
			break;
		}
		case 'o':{
			unsigned char inchar2[2];
			inchar2[1] = 0;
			_reset_keyboard(fd);
			printf("\nEnter module number {1-8}: ");
			while(!((inchar2[0]=getchar())<'9' && inchar2[0]>='1')){
				printf("\nEnter module number {1-8}: ");
			}
			getchar();  // removes trailing return
			_set_keyboard_raw(fd);
			current_cube=atoi(inchar2);
			printf("%d",current_cube);
			--current_cube;
			printf("%d",current_cube);
			printf("Current module number is now: %d (%d).\n",current_cube+1, (int)inchar2[0]);
			break;
		}
		case 'q':
		case 'Q':
			printf("Quiting\n");
			fflush(stdout);
			willExit = 1;
			break;
		case ' ':
			for(channel=0;channel<7;channel++){
				printf("Halting module %d ....",channel+1);
				pc_halt_module(cube[channel]);
				msg[0] = 0;
				j=pc_listen_for_response(cube[channel].handle, &msg);
				if(msg[0]==PC_COMMAND_HALT){
					printf("successful\n");
				}else{
					printf("failed! response from %d was %X\n", j, msg[0]);
				}
			}
			break;
		case 'z':{
			unsigned int t;
			float current_pos;
			char press_char;

			printf("WARNING: use with caution. This command sets the zero angle position of the joint. Recalibration of the kinematics will be needed.\n");
			printf("Press 'e' to continue, any other key to quit...\n");
			read(fd,&press_char,1);
			if(press_char=='e'){
				msg[0] = PC_COMMAND_HALT;
				pc_send_command_to_module(cube[current_cube],msg,1);
				//pc_halt_module(cube[current_cube]);
				msg[0] = 0;
				j = pc_listen_for_response(cube[current_cube].handle, &msg);
				if(msg[0]==PC_COMMAND_HALT){
					printf("successful\n");
				}
				else{
					printf("failed! response from %d was %X\n",j,msg[0]);
				}
				t = 1;
				while(t!=0)t  = sleep(t);

				msg[0] = PC_COMMAND_SET_EXTENDED;
				msg[1] = PC_PARA_ACT_POS;
				dlc = 6;
				msg[2] = 0;
				msg[3] = 0;
				msg[4] = 0;
				msg[5] = 0;
				msg[6] = 0;
				msg[7] = 0;
				pc_send_command_to_module(cube[current_cube], msg,6);
				pc_listen_for_response(cube[current_cube].handle,&msg);
				get_data_float(msg, &current_pos);
				printf("Power off the arm for 5 seconds and turn back on...\n");
			}
			break;
		}
		case 'e':
		{
			char input_msg_buff[80];
			int correct_input = 0;
			float new_pos;
			float cur_pos;
			float cur_vel;
			float cur_curr;
			float max_vel;
			float target_vel;
			char state;
			char dio_state;
			unsigned long int cube_state;
			unsigned long int confword;
			unsigned char ret_msg[8];
			unsigned short int time4motion;
			msg[0] = PC_COMMAND_GET_EXTENDED;

			msg[1] = PC_PARA_ACT_POS;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_pos);

			msg[1] = PC_PARA_ACT_VEL;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_vel);

			msg[1] = PC_PARA_CUR;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&cur_curr);

			msg[1] = PC_PARA_CUBE_STATE;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_uint32(ret_msg,&cube_state);

			msg[1] = PC_PARA_MAX_VEL;
			pc_request_value_from_module(cube[current_cube],msg);
			pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_float(ret_msg,&max_vel);

			msg[1] = PC_PARA_CONFIG;
			pc_request_value_from_module(cube[current_cube], msg);
			ret = pc_listen_for_response(cube[current_cube].handle,&ret_msg);
			get_data_uint32(ret_msg, &confword);


			printf("Current module number is %d\n", current_cube+1);
			if(current_cube==7){
				printf("Pos = %f m, ", cur_pos);
				printf("Vel = %f m/s, ", cur_vel);
				printf("Curr = %f A\n", cur_curr);
			}else{
				printf("Pos = %f deg, ", 180*cur_pos/PI);
				printf("Vel = %f deg/s, ", 180*cur_vel/PI);
				printf("Curr = %f A ,", cur_curr);
				printf("MaxVel = %f deg/s, \n", 180*max_vel/PI);
			}
			_reset_keyboard(fd);
			printf("Move step extended command\n");
			while(correct_input<2 && !willExit){
				correct_input = 0;
				if(current_cube==7){
					printf("Enter new pos {0, 60}[mm]: ");
				}else{
					printf("Enter new angle {-180, 180}[deg]: ");
				}
				fgets(input_msg_buff,80,stdin);
				new_pos = (float)(strtod(input_msg_buff,NULL));
				if((new_pos >= -180) && (new_pos <= 180)){
					correct_input += 1;
				}

				printf("Enter time [ms]: ");
				fgets(input_msg_buff,80,stdin);
				time4motion = (unsigned short int)(strtod(input_msg_buff,NULL));
				target_vel = (((new_pos*PI/180)-cur_pos)/(((float)time4motion)/1000));

				if(target_vel<0) target_vel *= -1.0;
				if(target_vel < max_vel){
					correct_input += 1;
				}
			}
			if(willExit) break;
			_set_keyboard_raw(fd);
			if(current_cube==7){
				printf("Setting grip to %f mm\n", new_pos);
				new_pos = new_pos/1000;
			}else{
				printf("Setting angle to %f deg\n", new_pos);
				new_pos = (new_pos/180)*PI;
			}

			// set sync motion
			confword = confword | PC_CONFIGID_MOD_SYNC_MOTION;
			msg[0] = PC_COMMAND_SET_EXTENDED;
			msg[1] = PC_PARA_CONFIG;
			dlc = set_data_uint32(msg, &confword);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle,&msg);


			msg[0] = PC_COMMAND_SET_MOTION;
			msg[1] = PC_MOTION_FSTEP_ACK;
			dlc = set_data_float_uint16(msg, &new_pos, &time4motion);
			pc_send_command_to_module(cube[current_cube], msg, dlc);
			pc_listen_for_response(cube[current_cube].handle,&msg);
			get_data_float(msg, &cur_pos);
			printf("Response from module is:\n");
			printf("ComID:%#02X, MotID:%#02X, Pos:%f\n", msg[0],msg[1], cur_pos*180/PI);
			printf("Module State: %#02X");
			if(msg[6]&PC_ACK_SHORT_NOT_OK){
				printf("ERROR REPORTED! [%#02X] (press g for details)\n",msg[6]);
			}

			pc_sync_all_on_bus(cube[current_cube].handle);
			break;

		}

		default:
			printf("You pressed %c\n",inchar);
			break;
		}
	}

	for(current_cube=0;current_cube<4;current_cube++){
		printf("For cube nr %d",current_cube+1);
		printf("  Killing watchdog....");
		fflush(stdout);
		//pc_kill_watchdog(watchdog_thread[current_cube]);
		printf("Done\n");

		printf("  Resetting PC baudrate....");
		fflush(stdout);
		//canSetBusParams(cube[current_cube].handle, bitrate1M, 4, 3, 1, 1, 0);
		//pc_set_baud_all_on_bus(cube[current_cube].handle, PC_BAUDRATE_1M);
		//canSetBusParams(cube[current_cube].handle, bitrate500k, 4, 3, 1, 1, 0);
		//pc_set_baud_all_on_bus(cube[current_cube].handle, PC_BAUDRATE_250K);
		//canSetBusParams(cube[current_cube].handle, bitrate250k, 4, 3, 1, 1, 0);
		//pc_set_baud_all_on_bus(cube[current_cube].handle, PC_BAUDRATE_250K);
		printf("Done\n");

		fflush(stdout);
		canClose(cube[current_cube].handle);
	}
	printf("  Resetting Keyboard....");
	fflush(stdout);
	_reset_keyboard(fd);

	printf("Done\n");

	fclose(read_log_file);
	sighand(SIGALRM);
	printf("Ready\n");
	return 0;
}





