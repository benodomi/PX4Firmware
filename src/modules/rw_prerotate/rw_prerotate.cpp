//#include <px4_config.h>
//#include <px4_defines.h>
//#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_log.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
//#include <poll.h>
#include <drivers/drv_hrt.h>
//#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
//#include <uORB/topics/control_state.h>
//#include <uORB/topics/vehicle_control_mode.h>
//#include <uORB/topics/vehicle_status.h>

//#include <uORB/topics/rotor_frequency.h>
//#include <uORB/topics/actuator_controls.h>
//#include <uORB/topics/actuator_controls_rw.h>

#include "rw_prerotate.h"




RW_Prerotate::RW_Prerotate() : ModuleParams(nullptr)
{
}

float RW_Prerotate::update_state(){
    return 0.0f;
}

void RW_Prerotate::run(){


}

/* Setters */
void RW_Prerotate::set_target_rpm(float rpm)
{
    _target_rpm = rpm;
}

float RW_Prerotate::get_desired_prerotator()
{

    return -1.0f;
}

int RW_Prerotate::get_prerotating_state()
{

    return -1;
}

void RW_Prerotate::parameters_update()
{
    uint64_t start;
    switch (_current_state) {

        /*
            State of nothing to do
        */
        case PREROTATE_STATE_DISABLED:
            break;

        /*
            Prepare rotor head and flags for prerotation
        */
        case PREROTATE_STATE_READY:
            PX4_WARN("Zacatek: PREROTATE_STATE_READY");
            //_vehicle_control_mode.flag_control_manual_enabled = false;
            //_vehicle_control_mode.flag_control_attitude_enabled = false;
            //orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
            PX4_WARN("nastaveny control mode");
            // _actuators.control[actuator_controls_rw_s::INDEX_ROTOR_ROLL] = 0;
            // _actuators.control[actuator_controls_rw_s::INDEX_ROTOR_PITCH] = 1;
            // _actuators.control[actuator_controls_rw_s::INDEX_THROTTLE] = -1;
            // _actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR] = -1;

            // _vehicle_rates_setpoint.roll = 1;
            // _vehicle_rates_setpoint.pitch = -1;
            // _vehicle_rates_setpoint.yaw = -1;
            // _vehicle_rates_setpoint.thrust = 1;
            // orb_publish(ORB_ID(vehicle_rates_setpoint), _vehicle_rates_setpoint_pub, &_vehicle_rates_setpoint);


            PX4_WARN("Nastaveny vystupy");
            start = hrt_absolute_time();
            _current_state = PREROTATE_STATE_START;
            //PX4_WARN(start);
            break;

        /*
            Because of high mass of rotor and not so powerfull prerotator motors
            this state puts some initial PPM value to ESC input. Itihiar PPM
            value is set in parametr ''. Then it wait to prerotate to some
            target rotor speed stored in parametr ''.
        */
        case PREROTATE_STATE_START:
            // _actuators.control[actuator_controls_rw_s::INDEX_ROTOR_ROLL] = 0;
            // _actuators.control[actuator_controls_rw_s::INDEX_ROTOR_PITCH] = 1;
            // _actuators.control[actuator_controls_rw_s::INDEX_THROTTLE] = -1;
            // _actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR] = prerotator_start_up/50.0f - 1.0f;

            //
            // _vehicle_rates_setpoint.roll = 1;
            // _vehicle_rates_setpoint.pitch = -1;
            // _vehicle_rates_setpoint.yaw = -1;
            // _vehicle_rates_setpoint.thrust = 1;
            // orb_publish(ORB_ID(vehicle_rates_setpoint), _vehicle_rates_setpoint_pub, &_vehicle_rates_setpoint);


            if (hrt_elapsed_time(&start) > 2000000.0f) {   // Wait 5 second
                _current_state = PREROTATE_STATE_RAMPUP;
                start = hrt_absolute_time();
                PX4_WARN("Prechazim na dalsi krok: RAMPUP.");
            }

            //if (_rf_report.indicated_frequency_rpm > _prer_min_rpm){
            //  start = hrt_absolute_time();
            //  _current_state = PREROTATE_STATE_RAMPUP;
            //  errx(1, "Prechazim na dalsi krok: start.");
            //}
            break;

        /*
            In this state the rotor is rotating at some minimal speed ('')
            Rotor is getting higher speed until the target speed ('') is
            reached. Every ('')
            Every ('') seconds it wait rotor speed stabilisation to vertify
            that actual is speed corresponding with set output. It waiting for
            acceleration is smaller then parametr ('').
        */
        case PREROTATE_STATE_RAMPUP:

            //TF-TODO: nahradit za parametr
            // _actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR] = (hrt_absolute_time()-start)/1000000.0f * 2.0f - 1.0f;
            //PX4_WARN("PREROTATOR RUMPUP %.4f %.4f> %.4f", (double)(hrt_absolute_time()-start), (double)start, (double)_actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR]);

            //_actuators.timestamp = hrt_absolute_time();


            if (hrt_elapsed_time(&start) > 15000000) {   // Wait 15 second
                _current_state = PREROTATE_STATE_HOLD;
                start = hrt_absolute_time();
                PX4_WARN("Prechazim na dalsi krok: HOLD.");
            }

            break;

        /*
            In this state it hold prerotator output with the same value.
        */
        case PREROTATE_STATE_HOLD:

            //_actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR] = 0;
            _current_state = PREROTATE_STATE_DONE;
            start = hrt_absolute_time();
            PX4_WARN("Prechazim na dalsi krok: DONE.");

            break;

        /*
            Aircraft flags are set to previous configuration.
        */
        case PREROTATE_STATE_DONE:
            //TF-TODO: opravdu nastavit predchozi hodnotu.
            // _current_state = PREROTATE_STATE_DISABLED;
            // PX4_WARN("NASTAVUJI PUVODNI DATA");
            // _vehicle_control_mode.flag_control_manual_enabled = true;
            // _vehicle_control_mode.flag_control_attitude_enabled = true;
            // orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
            // PX4_WARN("DONE.....");
            break;

        }


        // if (_actuators_0_pub != nullptr) {
        //     PX4_WARN("ODESLANI DATnull");
        //     orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
        //
        // } else if (_actuators_id) {
        //     PX4_WARN("ODESLANI DAT1");
        //     _actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
        // }

        // run at roughly 100 hz
        //usleep(sleeptime_us);

}


int RW_Prerotate::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
        # RW_Prerotate

        ### Description
        RW_Prerotate is controller for prerotator of rotor.

        )DESCR_STR");


	PRINT_MODULE_USAGE_NAME("rw_prerotate", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


RW_Prerotate *RW_Prerotate::instantiate(int argc, char *argv[])
{
	return new RW_Prerotate();
}


int RW_Prerotate::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("rw_prerotate",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_POSITION_CONTROL,
				      1810,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}


int RW_Prerotate::print_status()
{
	PX4_INFO("Running (mozna)");
	return 0;
}


int rw_prerotate_main(int argc, char *argv[])
{
	PX4_INFO("RW_PREROTATE_MAIN");
    return RW_Prerotate::main(argc, argv);

}



int RW_Prerotate::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}
