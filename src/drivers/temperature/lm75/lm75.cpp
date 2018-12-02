/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file lm75.cpp
 *
 * @author Vít Hanousek <slimonslimon@gmail.com>
 *
 * Driver for I2C Temperature sensor LM75 - tested on npx lm75a
 */

#include <board_config.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>


#include <px4_log.h>

#include <drivers/device/i2c.h>


#include <sys/types.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/temperature.h>

#include <drivers/drv_hrt.h>



/* Configuration Constants */
#define LM75_BUS_DEFAULT 		      PX4_I2C_BUS_EXPANSION
#define LM75_DEVICE_PATH	         "/dev/lm75"


//loaded parameters defaluts
#define LM75_BASEADDR_DEFAULT 	            0x48     //LM75_ADDR param default value //TODO
#define LM75_POOL_INTERVAL_DAFAULT	         1000000  //LM75_POOL param defalut value

#define LM75_POOL_INTERVAL_MAX              100000000 //100s max - limiter for setting by ioctl


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


class LM75 : public device::I2C
{
public:
	LM75(int bus = LM75_BUS_DEFAULT,
	      int address = LM75_BASEADDR_DEFAULT);
	virtual ~LM75();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			   ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	int            _pool_interval; //Interval of reading counter and publishing frequency
   int            _pool_interval_default; //Interval of reading counter and publishing frequency
   float          _measured_temperature;

	work_s			_work{};

	orb_advert_t		_temperature_topic;

	//perf_counter_t		_sample_perf;
	//perf_counter_t		_comms_errors;


	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void stop();

   /**
      set cycle trampoline in workqueue
   */
   void schedule_measurement();

	/**
	* Perform a poll cycle; collect from the previous measurement.
	*/
	void cycle();

   /**
   *Publis rotor_frequency uORB mesage
   */
   void publish();

   float getTemperature();
   void setRegister(uint8_t reg, uint8_t value);
   uint16_t readRegister16(uint8_t reg);

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int lm75_main(int argc, char *argv[]);

LM75::LM75(int bus, int address) :
	I2C("LM75", LM75_DEVICE_PATH, bus, address, 400000),
	_pool_interval(0),//us-0=disabled
   _pool_interval_default(LM75_POOL_INTERVAL_DAFAULT),//us
	_measured_temperature(0.0),
	_temperature_topic(nullptr)
	//_sample_perf(perf_alloc(PC_ELAPSED, "sf1xx_read")),
	//_comms_errors(perf_alloc(PC_COUNT, "sf1xx_com_err"))

{
}

LM75::~LM75()
{
	/* make sure we are truly inactive */
	stop();

	if (_temperature_topic != nullptr) {
		orb_unadvertise(_temperature_topic);
	}

	/* free perf counters */
	//perf_free(_sample_perf);
	//perf_free(_comms_errors);
}

int
LM75::init()
{
	int ret = PX4_ERROR;

   //load parameters 
	int address=LM75_BASEADDR_DEFAULT;
   if(param_find("LM75_ADDR")!=PARAM_INVALID)
   	param_get(param_find("LM75_ADDR"), &address);

   set_device_address(address); 

   if(param_find("LM75_POOL")!=PARAM_INVALID)
      param_get(param_find("LM75_POOL"),&_pool_interval_default);

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

   //setup device mode
   setRegister(0x01,0x00);

	/* get a publish handle on the range finder topic */
	struct temperature_s rf_report = {};
	_temperature_topic=orb_advertise(ORB_ID(temperature), &rf_report);

	if (_temperature_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	return PX4_OK;
}

float
LM75::getTemperature()
{
      int16_t r = (int16_t)readRegister16(0x00);
      //temperature calculation register_value * 0.00390625; (Sensor is a big-endian but SMBus is little-endian by default)
      return (float)r / 256.0f ;
}

uint16_t 
LM75::readRegister16(uint8_t reg)
{
      uint8_t rcv[2];
      int ret=transfer(&reg, 1, rcv, 2); //TODO: check endians

      if (OK != ret) {
	      PX4_DEBUG("LM75::readRegister16 : i2c::transfer returned %d", ret); 
      }

      //PX4_INFO( "%x, %x",rcv[0], rcv[1]);

      uint16_t res=0x0;
      res=rcv[0]<<8 | rcv[1];
      return res;
}

void 
LM75::setRegister(uint8_t reg, uint8_t value)
{
      uint8_t buff[2];
      buff[0]=reg;
      buff[1]=value;
      int ret=transfer(buff, 2, nullptr, 0);

      if (OK != ret) {
		      PX4_DEBUG("LM75::setRegister : i2c::transfer returned %d", ret); 
	      }
}

int
LM75::probe()
{
	//return measure();??
   return PX4_OK;
}

int
LM75::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				 _pool_interval = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:
			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = ( _pool_interval == 0);

					/* set interval for next measurement to minimum legal value */
               
					 _pool_interval = _pool_interval_default;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_pool_interval == 0);

					/* convert hz to interval in microseconds */
					int interval = 1000000 / arg; 

					/* check against maximum rate */
					if (interval > 0 && interval < LM75_POOL_INTERVAL_MAX) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_pool_interval = interval;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_pool_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000000 / _pool_interval); //us -> Hz

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
LM75::read(device::file_t *filp, char *buffer, size_t buflen)
{
	int ret = 0;

	/* buffer must be large enough */
   if(buflen < sizeof(float))
   {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_pool_interval> 0) {
      *((float*)buffer)=_measured_temperature;
      ret+=sizeof(float);
		/* if there was no data, warn the caller */
		return ret;
	}

	/* manual measurement - run one conversion */
   //TODO:ResetCounter      
	/* wait for it to complete */
	//usleep(LM75_POOL_INTERVAL);
   //TODO: getCounter
   
	return ret;
}

void LM75::publish()
{
   //PX4_INFO("Measurement: freq:%.2f, acc: %.2f, relacc: %.0f\%, count: %d", (double)_indicated_frequency, (double) _estimated_accurancy,
    //      (double)(_estimated_accurancy/_indicated_frequency*100), _count);

   //PX4_ERR("Measurement: freq:%f, count: %d", (double)_indicated_frequency, _count);

	struct temperature_s msg;
	msg.timestamp = hrt_absolute_time();
   msg.temperature = _measured_temperature;
   msg.sensor=1; //TODO: mulutiple sensors 

	// publish it, if we are the primary 
	if (_temperature_topic != nullptr) {
		orb_publish(ORB_ID(temperature), _temperature_topic, &msg);
	}

	// notify anyone waiting for data 
	poll_notify(POLLIN);

}


void
LM75::start()
{
	/* schedule a cycle to start things */
   _measured_temperature=getTemperature();
	schedule_measurement();
}

void LM75::schedule_measurement()
{
   work_queue(HPWORK, &_work, (worker_t)&LM75::cycle_trampoline, this, USEC2TICK(_pool_interval));
}

void
LM75::stop()
{
	work_cancel(HPWORK, &_work);
}

void
LM75::cycle_trampoline(void *arg)
{
	LM75 *dev = (LM75 *)arg;
	dev->cycle();
   //start new cycle
   dev-> schedule_measurement();
}

void
LM75::cycle()
{
   _measured_temperature=getTemperature();
   publish();
}


void
LM75::print_info()
{
	//perf_print_counter(_sample_perf);
	//perf_print_counter(_comms_errors);
	printf("poll interval:  %d us\n", _pool_interval);
}

/**
 * Local functions in support of the shell command.
 */
namespace lm75
{

LM75	*g_dev;

int 	start_bus( int i2c_bus);
int 	stop();
int 	info();

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(int i2c_bus)
{
	int fd = -1;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new LM75( i2c_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(LM75_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
      PX4_INFO("Cannot open device %s",LM75_DEVICE_PATH);
		goto fail;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
      PX4_INFO("error setting PollRate...");
		px4_close(fd);
		goto fail;
	}

	px4_close(fd);
   PX4_INFO("lm75 for bus: %d started.",i2c_bus );
	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

} /* namespace */


static void
lm75_usage()
{
	PX4_INFO("usage: lm75 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", LM75_BUS_DEFAULT);
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|info");
}

int
lm75_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	int i2c_bus = LM75_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) 
   {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option!");
			goto out_error;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
			return lm75::start_bus(i2c_bus);
		}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return lm75::stop();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return lm75::info();
	}

out_error:
	lm75_usage();
	return PX4_ERROR;
}
