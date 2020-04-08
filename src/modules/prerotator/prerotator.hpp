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


#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/rpm.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/parameter_update.h>

extern "C" __EXPORT int prerotator_main(int argc, char *argv[]);


class Prerotator : public ModuleBase<Prerotator>, public ModuleParams
{
public:
	Prerotator(int example_param, bool example_flag);

	virtual ~Prerotator() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Prerotator *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);


	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:


    void parameters_update(int parameter_update_sub, bool force = false);
	int _current_state = 0;

	int _rpm_sub{-1};
	int _vehicle_status_sub{-1};
	int _airspeed_sub{-1};

	rpm_s _rpm{};
	vehicle_status_s _vehicle_status{};
	airspeed_s _airspeed{};

	void rpm_poll();
	void airspeed_poll();
	void vehicle_status_poll();

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PREROT_WAIT_RPM>) _param_wait_rpm,
		(ParamFloat<px4::params::PREROT_RAMP_T>) _param_ramp_time,
		(ParamFloat<px4::params::PREROT_PWM_MIN>) _param_pwm_min,
		(ParamFloat<px4::params::PREROT_PWM_MAX>) _param_pwm_max,
		(ParamFloat<px4::params::PREROT_PWM_IDLE>) _param_pwm_idle,
		(ParamFloat<px4::params::PREROT_TRG_RPM>) _param_target_rpm,
		(ParamFloat<px4::params::PREROT_END_ASPD>) _param_end_airspeed
	)

	enum prerotation_states{
		START,
		WAIT_FOR_PREROTATION,
		PREROTATION,
		TARGET_SPEED,
		HOLD,
		OFF
	};

	// Subscriptions
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};

};
