/****************************************************************************
 *
 *   Copyright (c) 2020 ThunderFly s.r.o. All rights reserved.
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
 * @file prerotator.cpp

 * This module is controller of rotor prerotator. It ensures correct prerotation
 * of the rotor. It is useful for autogyros, equipped with a prerotator.
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 * @url https://www.thunderfly.cz/
 */


#include "follow_me_master.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rpm.h>

#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>


int FollowMeMaster::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Current state %d", _current_state);
	// TODO: print additional runtime information about the state of the prerotator

	return 0;
}

int FollowMeMaster::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		("not running");
		return 1;
	}
	 */

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "set_step")) {
		PX4_INFO("SET STEP Command");
		return 0;
	}

	return print_usage("unknown command");
}


int FollowMeMaster::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("prerotator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

FollowMeMaster *FollowMeMaster::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	FollowMeMaster *instance = new FollowMeMaster(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

FollowMeMaster::FollowMeMaster(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}



void
FollowMeMaster::vehicle_status_poll()
{
	bool updated;
	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}

void
FollowMeMaster::vehicle_global_position_poll()
{
	bool updated;
	orb_check(_vehicle_global_position_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_global), _vehicle_global_position_sub, &_vehicle_global_position);
	}
}

void
FollowMeMaster::vehicle_local_position_poll()
{
	bool updated;
	orb_check(_vehicle_local_position_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_po), _vehicle_local_position_sub, &_vehicle_local_position);
	}
}


void
FollowMeMaster::run()
{
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	orb_set_interval(_vehicle_status_sub, 100);
	orb_set_interval(_vehicle_global_position_sub, 100);
	orb_set_interval(_vehicle_local_position_sub, 100);

	vehicle_status_poll();
	vehicle_global_position_poll();
	vehicle_local_position_poll();

	uORB::Publication<follow_target_s> control_pub{ORB_ID(follow_target)};

	px4_pollfd_struct_t fds[] = {
	    { .fd = _vehicle_status_sub,   .events = POLLIN },
	    { .fd = _vehicle_global_position_sub,   .events = POLLIN },
	    { .fd = _vehicle_local_position_sub,   .events = POLLIN },
	};


    hrt_abstime _state_changed{0};

	// initialize parameters
	parameters_update(parameter_update_sub, true);

	while (!should_exit()) {
		int pret = px4_poll(fds, 1, 300);
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		rpm_poll();
		airspeed_poll();
		vehicle_status_poll();

		current_target.lat = _vehicle_global_position.lat * 1e7;
		current_target.lon = _vehicle_global_position.lon * 1e7;
		current_target.alt = _vehicle_global_position.terrain_alt;
		current_target.vx = _vehicle_local_position.vx;
		current_target.vy = _vehicle_local_position.vy;
		current_target.vz = _vehicle_local_position.z;
		current_target.est_cap = 0;

		//PX4_INFO("Data: %.1f, stav: %d, rpm: %.3f", (double)control.control[4], _current_state, (double)_rpm.indicated_frequency_rpm);

		uint64_t timestamp_us = hrt_absolute_time();
		current_target.timestamp = timestamp_us;

		follow_target_pub.publish(current_target);
		// fetch parameter update
		parameters_update(parameter_update_sub, false);

	}

	orb_unsubscribe(_vehicle_global_position_sub);
	orb_unsubscribe(_vehicle_local_position_sub);
	orb_unsubscribe(_vehicle_status_sub);
}

void
FollowMeMaster::parameters_update(int parameter_update_sub, bool force)
{
    bool updated;
    struct parameter_update_s param_upd;

    // Check if any parameter updated
    orb_check(parameter_update_sub, &updated);

    // If any parameter updated copy it to: param_upd
    if (updated) {
        orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
    }

    if (force || updated) {
        // If any parameter updated, call updateParams() to check if
        // this class attributes need updating (and do so).
        updateParams();
    }
}

int
FollowMeMaster::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

### Examples
CLI usage example:
$ follow_me_master start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("follow_me_master", "");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int prerotator_main(int argc, char *argv[])
{
	return FollowMeMaster::main(argc, argv);
}
