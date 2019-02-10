

#ifndef RWPREROTATE_HPP_
#define RWPREROTATE_HPP_

#pragma once

#include <px4_module.h>
#include <px4_module_params.h>

struct PREROTATOR_ControlData {
    float controll_input;
    float current_state;
	float airspeed_min;
	float airspeed_max;
	float airspeed;
	float rotorspeed_min;
	float rotorspeed_max;
	float rotorspeed;
	float scaler;
	float groundspeed;
	float groundspeed_scaler;
	bool lock_integrator;
};

enum PREROTATE_STATE {
    PREROTATE_STATE_DISABLED = 0,
    PREROTATE_STATE_READY = 1,
    PREROTATE_STATE_START = 2,
    PREROTATE_STATE_RAMPUP = 3,
    PREROTATE_STATE_HOLD = 4,
    PREROTATE_STATE_DONE = 5,
};


extern "C" __EXPORT int rw_prerotate_main(int argc, char *argv[]);

class RW_Prerotate: public ModuleBase<RW_Prerotate>, public ModuleParams
{
public:
	RW_Prerotate();
	virtual ~RW_Prerotate() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static RW_Prerotate *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

    static float update_state();

	/* Setters */
	void set_target_rpm(float rpm);

	/* Getters */
	float get_desired_prerotator();
    int get_prerotating_state();

private:
   void	parameters_update();

protected:
    float _control_input;
	float _target_rpm;
    float _current_rpm;
    float _debug_rpm;
    int _current_state;
};


#endif
