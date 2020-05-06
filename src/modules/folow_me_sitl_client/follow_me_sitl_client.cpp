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
 * @file balbalbal.cpp
 * This module is for blablabla.
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 * @url https://www.thunderfly.cz/
 */


#include "follow_me_sitl_client.hpp"

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


#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <termios.h>


static int _fd;
static unsigned char _buf[2048];
static sockaddr_in _srcaddr;
static unsigned _addrlen = sizeof(_srcaddr);


int FollowMeSitlClient::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Current state %d", _current_state);
	// TODO: print additional runtime information about the state of the prerotator

	return 0;
}

int FollowMeSitlClient::custom_command(int argc, char *argv[])
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


int FollowMeSitlClient::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("folowmeclient",
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

FollowMeSitlClient *FollowMeSitlClient::instantiate(int argc, char *argv[])
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

	FollowMeSitlClient *instance = new FollowMeSitlClient(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

FollowMeSitlClient::FollowMeSitlClient(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void
FollowMeSitlClient::run()
{
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	uORB::Publication<follow_target_s> control_pub{ORB_ID(follow_target)};

    //hrt_abstime _state_changed{0};

	// initialize parameters
	parameters_update(parameter_update_sub, true);

    int _port=15555;

	struct sockaddr_in _myaddr {};
	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_port);

	if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
    {
		PX4_ERR("Creating UDP socket failed: %s", strerror(errno));
		return;
	}

	if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) 
    {
		PX4_ERR("bind for UDP port %i failed (%i)", _port, errno);
		::close(_fd);
		return;
	}



	while (!should_exit()) 
    {
    	int len = ::recvfrom(_fd, _buf, sizeof(_buf), 0,(struct sockaddr *)&_srcaddr, (socklen_t *)&_addrlen);
		if (len > 0)
        {
			mavlink_message_t msg;
            mavlink_status_t mavlink_status;
			for (int i = 0; i < len; i++) 
            {
				if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &mavlink_status)) 
                {
                    if(msg.msgid==MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
                    {
                        mavlink_global_position_int_t pos;
                        mavlink_msg_global_position_int_decode(&msg, &pos);
                        
                        //PX4_INFO("lat: %d lon: %d vx: %d vy: %d",pos.lat, pos.lon, pos.vx, pos.vy);

		                current_target.lat = pos.lat;
		                current_target.lon = pos.lon;
		                current_target.alt = pos.alt;
		                current_target.vx = pos.vx;
		                current_target.vy = pos.vy;
		                current_target.vz = pos.vz;
		                current_target.est_cap = 0;

		                uint64_t timestamp_us = hrt_absolute_time();
		                current_target.timestamp = timestamp_us;

		                control_pub.publish(current_target);
                            
                    }					
				}
			}
		}

		// fetch parameter update
		parameters_update(parameter_update_sub, false);

	}

}

void
FollowMeSitlClient::parameters_update(int parameter_update_sub, bool force)
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
FollowMeSitlClient::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
### Examples
CLI usage example:
$ follow_me_sitl_client start
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("follow_me_sitl_client", "");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int follow_me_sitl_client_main(int argc, char *argv[])
{
	return FollowMeSitlClient::main(argc, argv);
}
