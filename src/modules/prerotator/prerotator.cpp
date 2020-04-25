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


#include "prerotator.hpp"

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


int Prerotator::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Current state %d", _current_state);
	// TODO: print additional runtime information about the state of the prerotator

	return 0;
}

int Prerotator::custom_command(int argc, char *argv[])
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


int Prerotator::task_spawn(int argc, char *argv[])
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

Prerotator *Prerotator::instantiate(int argc, char *argv[])
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

	Prerotator *instance = new Prerotator(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Prerotator::Prerotator(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void
Prerotator::rpm_poll()
{
	bool updated;
	orb_check(_rpm_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(rpm), _rpm_sub, &_rpm);
	}
}

void
Prerotator::airspeed_poll()
{
	bool updated;
	orb_check(_airspeed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
	}
}



void
Prerotator::vehicle_status_poll()
{
	bool updated;
	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}


void
Prerotator::run()
{
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	_rpm_sub = orb_subscribe(ORB_ID(rpm));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));

	orb_set_interval(_rpm_sub, 100);
	orb_set_interval(_vehicle_status_sub, 100);
	orb_set_interval(_airspeed_sub, 100);

	rpm_poll();
	airspeed_poll();
	vehicle_status_poll();

	actuator_controls_s control{};
	uORB::Publication<actuator_controls_s> control_pub{ORB_ID(actuator_controls_1)};


	px4_pollfd_struct_t fds[] = {
	    { .fd = _rpm_sub,   .events = POLLIN },
	    { .fd = _vehicle_status_sub,   .events = POLLIN },
	    { .fd = _airspeed_sub,   .events = POLLIN },
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

		if (_current_state != prerotation_states::START &&
			_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED){
			_current_state = prerotation_states::START;
			PX4_INFO("Nastavuji start: %d", _vehicle_status.arming_state);
		}

		switch (_current_state) {
			case prerotation_states::START:
				if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED){
					_current_state = prerotation_states::WAIT_FOR_PREROTATION;
					PX4_INFO("Phase 1: Waiting for prerotation");
				}

				control.control[4] = -1.0f;
				break;

			case prerotation_states::WAIT_FOR_PREROTATION:
				control.control[4] = -1.0f;
				if(_rpm.indicated_frequency_rpm >= _param_wait_rpm.get())
				{
					_current_state = prerotation_states::PREROTATION;
                    _state_changed = hrt_absolute_time();
					PX4_INFO("Krok 2: Roztaceni");
				}
				break;

			case prerotation_states::PREROTATION:
				if((hrt_elapsed_time(&_state_changed) / 1e6f) < _param_ramp_time.get()){
                    control.control[4] = _param_pwm_min.get();
                    control.control[4] += (_param_pwm_max.get() - _param_pwm_min.get()) *
                        ((hrt_elapsed_time(&_state_changed) / 1e6f) /
                        _param_ramp_time.get());
                    control.control[4] -= 1500.0f;
                    control.control[4] /= 500.0f;
                }
                else{
	                control.control[4] = (_param_pwm_idle.get() - 1500.0f) / 500.0f;
                }

                if(_rpm.indicated_frequency_rpm > _param_target_rpm.get())
				{
					_current_state = prerotation_states::TARGET_SPEED;
					PX4_INFO("Krok 3: Vypinam prerotator, cekam na rozjezd");
				}
				break;

			case prerotation_states::TARGET_SPEED:
				control.control[4] = (_param_pwm_idle.get() - 1500.0f) / 500.0f;
				if(_airspeed.indicated_airspeed_m_s > _param_end_airspeed.get()){
					PX4_INFO("Vypnuti predtaceni");
					_current_state = prerotation_states::OFF;
				}

				break;

			case prerotation_states::HOLD:
				break;

			case prerotation_states::OFF:
				control.control[4] = -1.0f;
				break;
		}

		PX4_INFO("Data: %.1f, stav: %d, rpm: %.3f", (double)control.control[4], _current_state, (double)_rpm.indicated_frequency_rpm);

		uint64_t timestamp_us = hrt_absolute_time();
		control.timestamp = timestamp_us;
		control.timestamp_sample = timestamp_us;

		control_pub.publish(control);
		// fetch parameter update
		parameters_update(parameter_update_sub, false);

	}

	orb_unsubscribe(_rpm_sub);
	orb_unsubscribe(_airspeed_sub);
	orb_unsubscribe(_vehicle_status_sub);
}

void
Prerotator::parameters_update(int parameter_update_sub, bool force)
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
Prerotator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Application for rotor speed control

This application control prerotation of rotor. It is useful for prerotation of
autogyro or helicopter rotor.

### Examples
CLI usage example:
$ prerotator start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("prerotator", "");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int prerotator_main(int argc, char *argv[])
{
	return Prerotator::main(argc, argv);
}
