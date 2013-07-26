/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file multirotor_pos_control.c
 *
 * Skeleton for multirotor position controller
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/sensor_combined.h> // lagt til 
#include <systemlib/systemlib.h>

#include "hold_pos_control_params.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int multirotor_hold_pos_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int multirotor_pos_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int multirotor_hold_pos_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("multirotor pos control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("multirotor pos control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 60,
					 4096,
					 multirotor_pos_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tmultirotor pos control app is running\n");
		} else {
			printf("\tmultirotor pos control app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int
multirotor_pos_control_thread_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[multirotor pos control] Control started, taking over position control\n");

	/* structures */
	struct vehicle_status_s state;
	struct vehicle_attitude_s att;
	//struct vehicle_global_position_setpoint_s global_pos_sp;
	struct vehicle_local_position_setpoint_s local_pos_sp;
	struct vehicle_vicon_position_s local_pos;
	struct manual_control_setpoint_s manual;
	struct vehicle_attitude_setpoint_s att_sp;
	struct sensor_combined_s sensor_data; // lagt til

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	//int global_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int sensor_data_sub = orb_subscribe(ORB_ID(sensor_combined)); 

	/* publish attitude setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	thread_running = true;

	int loopcounter = 0;

	struct multirotor_position_control_params p;
	struct multirotor_position_control_param_handles h;
	parameters_init(&h);
	parameters_update(&h, &p);
	
	// HOLD altitude x meter //
	/* get a local copy of sensor data */
	orb_copy(ORB_ID(sensor_combined), sensor_data_sub, &sensor_data); //lagt til
	float wanted_height = sensor_data.baro_alt_meter; 
	printf("altitude start = %f \n", wanted_height);
	wanted_height += 0.1f; //blir mer - vet ikke hvorfor
	printf("wanted height start = %f \n", wanted_height);
	float current_error = 0.5f; 
	float prev_error = 0.5f;	
	int K_p = 8; 
	int K_d = 10;
	float PD_output;
	att_sp.thrust = 0.2f;
	att_sp.pitch_body = 0.0f;
	att_sp.roll_body = 0.0f;
	att_sp.yaw_body = 0.0f;
	att_sp.timestamp = hrt_absolute_time(); 
	// HOLD altitude x meter //	

	att_sp.thrust = 0.2f; 
	int manual_thrust_state = 0; // 0 = nothing, 1 = upp, 2 = down
	

	while (!thread_should_exit) {
		/* get a local copy of the vehicle state */
		orb_copy(ORB_ID(vehicle_status), state_sub, &state);
		/* get a local copy of manual setpoint */
		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
		/* get a local copy of local position */
		orb_copy(ORB_ID(vehicle_vicon_position), local_pos_sub, &local_pos);
		/* get a local copy of local position setpoint */
		orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);
		/* get a local copy of sensor data */
		orb_copy(ORB_ID(sensor_combined), sensor_data_sub, &sensor_data); //lagt til

		if (loopcounter == 500) {
			parameters_update(&h, &p);
			loopcounter = 0;
		}

		if (state.state_machine == SYSTEM_STATE_AUTO) {

			
			// HOLD altitude x meters - OBS ikke testet!! //
			printf("altitude = %f \n", sensor_data.baro_alt_meter);
			printf("wanted height = %f \n", wanted_height);
			if(att_sp.thrust < 0.5){
				att_sp.thrust += 0.01f; 	
				usleep(200000);			
			}
			else{	
				current_error = wanted_height - sensor_data.baro_alt_meter;
				printf("current error = %f \n", current_error);
				PD_output = K_p*current_error + K_d*(current_error-prev_error);	
				printf("PD_output = %f \n", PD_output);		
				prev_error = current_error;
				if(PD_output == 0 || (current_error < 0.25 && current_error > -0.25)){
					printf("at target\n");
					att_sp.thrust = 6.6f;
				}
				else if(PD_output > 0){
					printf("too low, go higher\n");
					att_sp.thrust = (((float)PD_output/8) * 0.03f) + 0.66f;
					if(att_sp.thrust > 0.75){
						att_sp.thrust = 0.75f;
					} 
					printf("thrust power %f\n", att_sp.thrust);
				}
				else{
					printf("too high, go lower\n");
					att_sp.thrust = 0.66f + (((float)PD_output/8) * 0.01f);
					if(att_sp.thrust < 0.64){
						att_sp.thrust = 0.64f;
					} 
					printf("thrust power %f\n", att_sp.thrust);
				}
			}
			att_sp.timestamp = hrt_absolute_time();	
			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
			// end of HOLD altitude x meters - OBS ikke testet!! //
			

			/*
			// Testing different thrust powers //
			// kommenter inn for teste ulike thrust powers(kommenter ut hold altitude x meters)//
			//start motor gradually 
			if(att_sp.thrust <= 0.49){
				att_sp.thrust += 0.01f; 	
				usleep(200000);			
			}
			//att_sp.thrust = 0.67f;
			else{
				if(manual.throttle >= 0.9 && manual_thrust_state!=1){
					manual_thrust_state = 1; //thrust power increase with 0.01
					att_sp.thrust += 0.01f;
				}
				else if(manual.throttle <= 0.1 && manual_thrust_state!=2){
					manual_thrust_state = 2; //thrust power decrease with 0.01  
					att_sp.thrust -= 0.01f;
				}
				else if(manual.throttle > 0.1 && manual.throttle < 0.9){
					manual_thrust_state = 0; 
				}			
			}
			printf("thrust power %f\n", att_sp.thrust);
			att_sp.pitch_body = 0.0f;
			att_sp.roll_body = 0.0f;
			att_sp.yaw_body = 0.0f;
			att_sp.timestamp = hrt_absolute_time();

			// publish new attitude setpoint *
			//printf("publishing attitude setpoint\n");
			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
						
			// End of Testing different thrust power //
			*/
		} 
		/* run at approximately 50 Hz */
		usleep(20000);
		loopcounter++;

	}

	printf("[multirotor pos control] ending now...\n");

	thread_running = false;

	fflush(stdout);
	return 0;
}
