/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file lqr_control_main.c
 * Under-development LQR controller for PX4 autopilot
 *
 * @author Philipp Foehn <foehn@ifi.uzh.ch>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
// #include <uORB/topics/sensor_combined.h>
// #include <uORB/topics/vehicle_attitude.h>
// #include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/control_state.h>

extern "C" __EXPORT int lqr_control_main(int argc, char *argv[]);

int lqr_control_main(int argc, char *argv[])
{
	PX4_INFO("LQR controller playground");

	/* subscribe to sensor_combined topic */
	// int sub_attitude = orb_subscribe(ORB_ID(vehicle_attitude));
	// int sub_gyro = orb_subscribe(ORB_ID(sensor_gyro));
	int sub_control_state = orb_subscribe(ORB_ID(control_state));

	/* limit the update rate to 5 Hz */
	// orb_set_interval(sub_attitude, 20);
	// orb_set_interval(sub_gyro, 1);


	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		// { .fd = sub_attitude,   .events = POLLIN },
		// { .fd = sub_gyro,	.events = POLLIN },
		{ .fd = sub_control_state,	.events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;
	// uint64_t last_gyro_time = 0;
	// float dt = 0.001;

	for (int i = 0; i < 1000; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 2, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			// if (fds[0].revents & POLLIN) {
			// 	/* obtained data for the first file descriptor */
			// 	struct vehicle_attitude_s att;
			// 	/* copy sensors raw data into local buffer */
			// 	orb_copy(ORB_ID(vehicle_attitude), sub_attitude, &att);
			// 	PX4_INFO("Attitude:\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
			// 		 (double)att.q[0],
			// 		 (double)att.q[1],
			// 		 (double)att.q[2],
			// 		 (double)att.q[3]);

			// }

			if(fds[0].revents & POLLIN)
			{
				struct control_state_s ctrl_state;

				orb_copy(ORB_ID(control_state), sub_control_state, &ctrl_state);
				PX4_INFO("Position:\t%8.4f\t%8.4f\t%8.4f",
					 (double)ctrl_state.x_pos,
					 (double)ctrl_state.y_pos,
					 (double)ctrl_state.z_pos);
				PX4_INFO("Attitude:\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
					 (double)ctrl_state.q[0],
					 (double)ctrl_state.q[1],
					 (double)ctrl_state.q[2],
					 (double)ctrl_state.q[3]);
				PX4_INFO("Velocity:\t%8.4f\t%8.4f\t%8.4f",
					 (double)ctrl_state.x_vel,
					 (double)ctrl_state.y_vel,
					 (double)ctrl_state.z_vel);
				PX4_INFO("Bodyrates:\t%8.4f\t%8.4f\t%8.4f",
					 (double)ctrl_state.roll_rate,
					 (double)ctrl_state.pitch_rate,
					 (double)ctrl_state.yaw_rate);
			}

			// if (fds[1].revents & POLLIN) {
			// 	/* obtained data for the first file descriptor */
			// 	struct sensor_gyro_s gyro;
			// 	/* copy sensors raw data into local buffer */
			// 	orb_copy(ORB_ID(sensor_gyro), sub_gyro, &gyro);
			// 	// PX4_INFO("Gyro:\t%8.4f\t%8.4f\t%8.4f",
			// 	// 	 (double)gyro.x,
			// 	// 	 (double)gyro.y,
			// 	// 	 (double)gyro.z);
			// 	if(last_gyro_time!=0)
			// 	{
			// 		dt = 0.99f*dt + 0.01f *(gyro.timestamp - last_gyro_time)/1000000.0f;
			// 		PX4_INFO("Gyrotime:\t%12.12f", (double)dt);
			// 	}
			// 	last_gyro_time = gyro.timestamp;
			// }
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
