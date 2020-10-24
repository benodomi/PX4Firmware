#include "timer.h"
#include "rtc-board.h"
#include "utilities.h"

#include <drivers/drv_hrt.h>

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static volatile bool RtcTimeoutPendingInterrupt = false;
static volatile bool RtcTimeoutPendingPolling = false;

typedef enum AlarmStates_e
{
    ALARM_STOPPED = 0,
    ALARM_RUNNING = !ALARM_STOPPED
} AlarmStates_t;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static hrt_abstime context;

uint32_t RtcSetTimerContext(void)
{
	context = hrt_absolute_time();
	return (uint32_t) context;
}

uint32_t RtcGetTimerContext(void)
{
    return (uint32_t) context;
}

uint32_t RtcGetTimerElapsedTime(void)
{
    return (uint32_t) hrt_elapsed_time_atomic(&context);
}

uint32_t RtcMs2Tick( TimerTime_t milliseconds )
{
	return milliseconds * 1000;
}

uint32_t RtcGetTimerValue()
{
	return hrt_absolute_time();
}

TimerTime_t RtcTick2Ms(uint32_t tick)
{
	return tick/1000;
}

uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
    uint32_t abstime = RtcGetTimerValue() / 1000;
    *milliseconds = abstime % 1000;
    return abstime / 1000;
}

uint32_t RtcGetMinimumTimeout(void)
{
    return 1;
}

uint32_t alarm_timeout = 0;

void RtcStopAlarm( void )
{
    alarm_timeout = 0;
}

void RtcSetAlarm( uint32_t timeout )
{
    alarm_timeout = timeout;
}

void RtcStartAlarm( uint32_t timeout )
{
    alarm_timeout = timeout;
}

#include <stdio.h>

uint32_t RtcAlarmRemaining()
{
    if (alarm_timeout == 0)
        return 1000*1000*1000;

    uint32_t elapsed = RtcGetTimerElapsedTime();
    if (elapsed > alarm_timeout)
        return 0;
    return alarm_timeout - elapsed;
}

void RtcProcess()
{
    if (RtcGetTimerElapsedTime() > alarm_timeout) {
        alarm_timeout = 0;
        TimerIrqHandler();
    }
}
