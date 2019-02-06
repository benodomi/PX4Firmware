#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
//#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/rotor_frequency.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_rw.h>

/*
 * rw_prerotate.cpp
 *
 */


extern "C" __EXPORT int rw_prerotate_main(int argc, char *argv[]);

class RwPrerotate
{
public:
    /* Constructor */
    RwPrerotate();

    /* Destructor, also kills the main task */
    ~RwPrerotate();

    /* Start the flip state switch task */
    /* @return OK on success*/
    int start();

    int prerotate_start();
    /**
     * This function handles the Mavlink command long messages
     * It will execute appropriate actions according to input
     */
    void handle_command(struct vehicle_command_s *cmd);

    /*  little function to print current flip state */
    void print_state();

    /* check for changes in vehicle control mode */
    void vehicle_control_mode_poll();

private:
    bool 		_task_should_exit; 		/**< if true, main task should exit */
    int 		_prerotate_task;				/**< task handle */

    enum PREROTATE_STATE {
        PREROTATE_STATE_DISABLED = 0,
        PREROTATE_STATE_READY = 1,
        PREROTATE_STATE_START = 2,
        PREROTATE_STATE_RAMPUP = 3,
        PREROTATE_STATE_HOLD = 4,
        PREROTATE_STATE_DONE = 5,
    }_prerotate_state;


    /* subscriptions */
    int 		_command_sub;
    int 		_vehicle_control_mode_sub;
    int 		_vehicle_attitude_sub;
    int 		_rf_report_sub;

    /* publications */
    orb_advert_t 	_vehicle_control_mode_pub;
    orb_advert_t 	_vehicle_rates_setpoint_pub;
	orb_advert_t	_actuators_0_pub{nullptr};		/**< actuator control group 0 setpoint */

	orb_id_t _actuators_id{nullptr};	// pointer to correct actuator controls0 uORB metadata structure

    struct vehicle_command_s 		_command;				/**< vehicle commands */
    struct vehicle_control_mode_s 	_vehicle_control_mode; 	/**< vehicle control mode */
    struct vehicle_attitude_s 		_attitude;				/**< vehicle attitude */
    struct vehicle_rates_setpoint_s _vehicle_rates_setpoint;			/**< vehicle rate setpoint */
    struct actuator_controls_rw_s	_actuators;		/**< actuator control inputs */
    struct rotor_frequency_s        _rf_report = {};

    /**
     * Shim for calling task_main from task_create
     */
    static void task_main_trampoline(int argc, char *argv[]);

    /**
     * Main attitude control task
     */
    void 		task_main();
};

namespace rw_prerotate
{
RwPrerotate *g_prerotate;
}

RwPrerotate::RwPrerotate() :
        _task_should_exit(false),
        _prerotate_task(-1),
        _prerotate_state(PREROTATE_STATE_DISABLED),

        _command_sub(-1),
        _vehicle_control_mode_sub(-1),
        _vehicle_attitude_sub(-1),
        _rf_report_sub(-1),

        _vehicle_control_mode_pub(nullptr),
        _vehicle_rates_setpoint_pub(nullptr)

{
    memset(&_command, 0, sizeof(_command));
    memset(&_vehicle_control_mode, 0, sizeof(_vehicle_control_mode));
    memset(&_attitude, 0, sizeof(_attitude));
    memset(&_vehicle_rates_setpoint, 0, sizeof(_vehicle_rates_setpoint));
}

RwPrerotate::~RwPrerotate()
{
    _task_should_exit = true;
    rw_prerotate::g_prerotate = nullptr;
}

void RwPrerotate::print_state()
{
    warnx("Current prerotation state is %d", _prerotate_state);
}

/*
    Zde se prijimaji Mavlink prikazy.
*/
void RwPrerotate::handle_command(struct vehicle_command_s *cmd)
{
    switch (cmd->command) {
    case vehicle_command_s::VEHICLE_CMD_PREROTATOR_START:

        warnx("Prerotation initiated");
        _prerotate_state = PREROTATE_STATE_READY;
        warnx("Prerotation initiated 2");
        break;
    case vehicle_command_s::VEHICLE_CMD_PREROTATOR_TERMINATE:

        warnx("Prerotation terminated");

        _prerotate_state = PREROTATE_STATE_DONE;
        break;
    }
}

void RwPrerotate::vehicle_control_mode_poll()
{
    bool updated;

    /* check if vehicle control mode has changed */
    orb_check(_vehicle_control_mode_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub, &_vehicle_control_mode);
    }
}

void RwPrerotate::task_main_trampoline(int argc, char *argv[])
{
    rw_prerotate::g_prerotate->task_main();
}

void RwPrerotate::task_main()
{
    /* make sure prerotate_state is disabled at initialization */
    _prerotate_state = PREROTATE_STATE_DISABLED;

    // use this to check if a topic is updated
    bool updated = false;

    // inner loop sleep time
    //FT-TODO: nahradit za nový parametr POOL_INTERVAL
    const unsigned sleeptime_us = 50000;


    int poll_interval = 100;

    /* subscribe to vehicle command topic */
    _command_sub = orb_subscribe(ORB_ID(vehicle_command));

    /* subscribe to vehicle control mode topic */
    _vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

    /* subscribe to vehicle attitude topic */
    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    /* subscribe to rotor freq */
    //_rf_report = orb_subscribe(ORB_ID(rotor_frequency));
    _rf_report_sub = orb_subscribe(ORB_ID(rotor_frequency));


    /* advertise control mode topic */
    _vehicle_control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &_vehicle_control_mode);

    /* advertise rate setpoint topic */
    _vehicle_rates_setpoint_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_vehicle_rates_setpoint);


    float prerotator_start_up;
    float prerotator_target_rpm;
    float prerotator_acceleration;


    if(param_find("PREROT_START_UP")!=PARAM_INVALID)
        param_get(param_find("PREROT_START_UP"),&prerotator_start_up);

    if(param_find("PREROT_TRG_RPM")!=PARAM_INVALID)
        param_get(param_find("PREROT_TRG_RPM"),&prerotator_target_rpm);

    if(param_find("PREROT_ACCEL")!=PARAM_INVALID)
        param_get(param_find("PREROT_ACCEL"),&prerotator_acceleration);



    /*
     * declare file descriptor structure, # in the [] means the
     * # of topics, here is 1 since we are only
     * polling vehicle_command
     */
    px4_pollfd_struct_t fds[1];

    /*
     * initialize file descriptor to listen to vehicle_command
     */
    fds[0].fd = _command_sub;
    fds[0].events = POLLIN;

    /* start main slow loop */
    while (!_task_should_exit) {

        /* set the poll target, number of file descriptor, and poll interval */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), poll_interval);

        /*
         * this means no information is coming from the topic in the set interval
         * skip loop
         */
        if (pret == 0) {
            continue;
        }

        /*
         * this means some error happened, I don't know what to do
         * skip loop
         */
        if (pret < 0) {
            warn("poll error %d %d", pret, errno);
            continue;
        }

        /*
         * if everything goes well, copy the command into our variable
         * and handle command
         */
        if (fds[0].revents & POLLIN) {
            /*
             * copy command structure from the topic to our local structure
             */
            orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);

            handle_command(&_command);
        }

        /*
         * check for updates in other topics
         */
        vehicle_control_mode_poll();

        uint64_t start = hrt_absolute_time();
        /*
         * switch to faster update during the flip
         */
        _vehicle_control_mode.flag_control_prerotator_enabled = true;
        warnx("Pomala");
        if ((_prerotate_state > PREROTATE_STATE_DISABLED)&&(!_vehicle_control_mode.flag_control_prerotator_enabled)){
            errx(1,"Neni zapnuty PREROTATE mode.");
            _prerotate_state = PREROTATE_STATE_DISABLED;
        }
        while ((_prerotate_state > PREROTATE_STATE_DISABLED)&&(_vehicle_control_mode.flag_control_prerotator_enabled)){
        //while ((_prerotate_state > PREROTATE_STATE_DISABLED)){
            warnx("smycka");
            // update commands
            orb_check(_command_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
                handle_command(&_command);
            }

            orb_check(_rf_report_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_command), _rf_report_sub, &_rf_report);
            }

            bool topic_changed = false;
            // copy vehicle control mode topic if updated
            vehicle_control_mode_poll();

            // disable _v_control_mode.flag_control_manual_enabled
            if (_vehicle_control_mode.flag_control_manual_enabled) {
                _vehicle_control_mode.flag_control_manual_enabled = false;
                warnx("ZMENA FLAGU..");
                topic_changed = true;
            }

            // disable _v_control_mode.flag_conttrol_attitude_enabled
            if (_vehicle_control_mode.flag_control_attitude_enabled) {
                _vehicle_control_mode.flag_control_attitude_enabled = false;
                warnx("ZMENA FLAGU..");
                topic_changed = true;
            }

            if (topic_changed) {
                orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
            }

            // update vehicle attitude
            orb_check(_vehicle_attitude_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_attitude);
            }

            switch (_prerotate_state) {

            /*
                State of nothing to do
            */
            case PREROTATE_STATE_DISABLED:
                break;

            /*
                Prepare rotor head and flags for prerotation
            */
            case PREROTATE_STATE_READY:
                warnx("Zacatek: PREROTATE_STATE_READY");
                //_vehicle_control_mode.flag_control_manual_enabled = false;
                //_vehicle_control_mode.flag_control_attitude_enabled = false;
                //orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
                warnx("nastaveny control mode");
                _actuators.control[actuator_controls_rw_s::INDEX_ROTOR_ROLL] = 0;
                _actuators.control[actuator_controls_rw_s::INDEX_ROTOR_PITCH] = 1;
                _actuators.control[actuator_controls_rw_s::INDEX_THROTTLE] = -1;
                _actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR] = -1;

                // _vehicle_rates_setpoint.roll = 1;
                // _vehicle_rates_setpoint.pitch = -1;
                // _vehicle_rates_setpoint.yaw = -1;
                // _vehicle_rates_setpoint.thrust = 1;
                // orb_publish(ORB_ID(vehicle_rates_setpoint), _vehicle_rates_setpoint_pub, &_vehicle_rates_setpoint);


                warnx("Nastaveny vystupy");
                start = hrt_absolute_time();
                _prerotate_state = PREROTATE_STATE_START;
                //warnx(start);
                break;

            /*
                Because of high mass of rotor and not so powerfull prerotator motors
                this state puts some initial PPM value to ESC input. Itihiar PPM
                value is set in parametr ''. Then it wait to prerotate to some
                target rotor speed stored in parametr ''.
            */
            case PREROTATE_STATE_START:
                _actuators.control[actuator_controls_rw_s::INDEX_ROTOR_ROLL] = 0;
                _actuators.control[actuator_controls_rw_s::INDEX_ROTOR_PITCH] = 1;
                _actuators.control[actuator_controls_rw_s::INDEX_THROTTLE] = -1;
                _actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR] = prerotator_start_up/50.0f - 1.0f;

                //
                // _vehicle_rates_setpoint.roll = 1;
                // _vehicle_rates_setpoint.pitch = -1;
                // _vehicle_rates_setpoint.yaw = -1;
                // _vehicle_rates_setpoint.thrust = 1;
                // orb_publish(ORB_ID(vehicle_rates_setpoint), _vehicle_rates_setpoint_pub, &_vehicle_rates_setpoint);


                if (hrt_elapsed_time(&start) > 2000000) {   // Wait 5 second
                    _prerotate_state = PREROTATE_STATE_RAMPUP;
                    start = hrt_absolute_time();
                    warnx("Prechazim na dalsi krok: RAMPUP.");
                }

                //if (_rf_report.indicated_frequency_rpm > _prer_min_rpm){
                //  start = hrt_absolute_time();
                //  _prerotate_state = PREROTATE_STATE_RAMPUP;
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


                // _vehicle_rates_setpoint.roll = -1;
                // _vehicle_rates_setpoint.pitch = 1;
                // _vehicle_rates_setpoint.yaw = 1;
                // _vehicle_rates_setpoint.thrust = -1;
                // orb_publish(ORB_ID(vehicle_rates_setpoint), _vehicle_rates_setpoint_pub, &_vehicle_rates_setpoint);


                //TF-TODO: nahradit za parametr
                _actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR] = (hrt_absolute_time()-start)/1000000.0f * 2.0f - 1.0f;
                warnx("PREROTATOR RUMPUP %.4f %.4f> %.4f", (double)(hrt_absolute_time()-start), (double)start, (double)_actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR]);

                _actuators.timestamp = hrt_absolute_time();

                if (_actuators_0_pub != nullptr) {
					orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

				} else if (_actuators_id) {
					_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
				}

                if (hrt_elapsed_time(&start) > 15000000) {   // Wait 15 second
                    _prerotate_state = PREROTATE_STATE_HOLD;
                    start = hrt_absolute_time();
                    warnx("Prechazim na dalsi krok: HOLD.");
                }

                break;

            /*
                In this state it hold prerotator output with the same value.
            */
            case PREROTATE_STATE_HOLD:

                _actuators.control[actuator_controls_rw_s::INDEX_PREROTATOR] = 0;
                _prerotate_state = PREROTATE_STATE_DONE;
                start = hrt_absolute_time();
                warnx("Prechazim na dalsi krok: DONE.");

                break;

            /*
                Aircraft flags are set to previous configuration.
            */
            case PREROTATE_STATE_DONE:
                //TF-TODO: opravdu nastavit predchozi hodnotu.
                _prerotate_state = PREROTATE_STATE_DISABLED;
                warnx("NASTAVUJI PUVODNI DATA");
                _vehicle_control_mode.flag_control_manual_enabled = true;
                _vehicle_control_mode.flag_control_attitude_enabled = true;
                orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
                warnx("DONE.....");
                break;

            }


            if (_actuators_0_pub != nullptr) {
                warnx("ODESLANI DATnull");
                orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

            } else if (_actuators_id) {
                warnx("ODESLANI DAT1");
                _actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
            }

            // run at roughly 100 hz
            usleep(sleeptime_us);
        }
    }
}

int RwPrerotate::prerotate_start()
{
    warnx("Prerotate_start initiated");
    _prerotate_state = PREROTATE_STATE_READY;

    return OK;
}

int RwPrerotate::start()
{
    //ASSERT(_prerotate_task == -1);

    /*start the task */
    _prerotate_task = px4_task_spawn_cmd("RwPrerotate",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT,
                                    2048,
                                    (px4_main_t)&RwPrerotate::task_main_trampoline,
                                    nullptr);
    warnx("START funkce");

    if (_prerotate_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;

}

int rw_prerotate_main(int argc, char *argv[])
{
    /* warn if no input argument */
    if (argc < 2) {
        warnx("usage: rw_prerotate {start|stop|status|state|prerotate}");
        return 1;
    }

    /* start rw_prerotate manually */
    if (!strcmp(argv[1],"start")) {

        if (rw_prerotate::g_prerotate != nullptr) {
            warnx("already running");
            return 1;
        }

        rw_prerotate::g_prerotate = new RwPrerotate;

        if (rw_prerotate::g_prerotate == nullptr) {
            warnx("allocation failed");
            return 1;
        }

        if (OK != rw_prerotate::g_prerotate->start()) {
            delete rw_prerotate::g_prerotate;
            rw_prerotate::g_prerotate = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    /* stop rw_prerotate manually */
    if (!strcmp(argv[1], "stop")) {
        if (rw_prerotate::g_prerotate == nullptr) {
            warnx("not running");
            return 1;
        }

        delete rw_prerotate::g_prerotate;
        rw_prerotate::g_prerotate = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "prerotate")) {
        if (rw_prerotate::g_prerotate == nullptr) {
            warnx("not running");
            return 1;
        }

        rw_prerotate::g_prerotate->prerotate_start();

        return 0;
    }

    /* return running status of the application */
    if (!strcmp(argv[1], "status")) {
        if (rw_prerotate::g_prerotate) {
            warnx("running");
            return 0;
        } else {
            warnx("not running");
            return 1;
        }
    }

    /* print current flip_state */
    if (!strcmp(argv[1], "state")) {
        rw_prerotate::g_prerotate->print_state();

        return 0;
    }


    /* if argument is not in one of the if statement */
    warnx("unrecognized command");

    return 0;
}
