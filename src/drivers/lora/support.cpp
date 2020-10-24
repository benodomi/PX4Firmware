#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>

#include <px4_arch/micro_hal.h>
#include <px4_platform_common/getopt.h>

#include "stm32_exti.h"

#include "vendor/sx126x/sx126x.h"
#include "vendor/sx126x-board.h"
#include "vendor/utilities.h"
#include "vendor/delay.h"

#include "rtc-board.h"

void Delay(float s)       { px4_usleep(s*1000000); }
void DelayMs(uint32_t ms) { px4_usleep(ms*1000); }

struct spi_dev_s *spi = NULL;

void SX126xIoInit( void )
{
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
}

void SX126xIoDeInit( void )
{
}

void SX126xIoDbgInit( void )
{
}

void SX126xIoTcxoInit( void )
{
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return 0;
}

void SX126xReset( void )
{
}

void SX126xWaitOnBusy( void )
{
    //while( GpioRead( &SX126x.BUSY ) == 1 );
    px4_usleep(1);
}

void SX126xWakeup( void )
{
    CRITICAL_SECTION_BEGIN( );

    SPI_SELECT(spi, 0x500, true);

    SPI_SEND(spi, RADIO_GET_STATUS);
    SPI_SEND(spi, 0x00);

    SPI_SELECT(spi, 0x500, false);

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    CRITICAL_SECTION_END( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    SPI_SELECT(spi, 0x500, true);
    SPI_SEND(spi, command);
    SPI_SNDBLOCK(spi, buffer, size);
    SPI_SELECT(spi, 0x500, false);

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

uint8_t SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint8_t status = 0;

    SX126xCheckDeviceReady( );

    SPI_SELECT(spi, 0x500, true);
    SPI_SEND(spi, command);
    SPI_RECVBLOCK(spi, &status, 1);
    SPI_RECVBLOCK(spi, buffer, size);
    SPI_SELECT(spi, 0x500, false);

    SX126xWaitOnBusy( );

    return status;
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    SPI_SELECT(spi, 0x500, true);
    SPI_SEND(spi, RADIO_WRITE_REGISTER);
    SPI_SEND(spi, (address & 0xFF00) >> 8);
    SPI_SEND(spi, address & 0x00FF);
    SPI_SNDBLOCK(spi, buffer, size);
    SPI_SELECT(spi, 0x500, false);

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    SPI_SELECT(spi, 0x500, true);
    SPI_SEND(spi, RADIO_READ_REGISTER);
    SPI_SEND(spi, (address & 0xFF00) >> 8);
    SPI_SEND(spi, address & 0x00FF);
    SPI_SEND(spi, 0);
    SPI_RECVBLOCK(spi, buffer, size);
    SPI_SELECT(spi, 0x500, false);

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    SPI_SELECT(spi, 0x500, true);
    SPI_SEND(spi, RADIO_WRITE_BUFFER);
    SPI_SEND(spi, offset);
    SPI_SNDBLOCK(spi, buffer, size);
    SPI_SELECT(spi, 0x500, false);

    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    SPI_SELECT(spi, 0x500, true);
    SPI_SEND(spi, RADIO_READ_BUFFER);
    SPI_SEND(spi, offset);
    SPI_SEND(spi, 0);
    SPI_RECVBLOCK(spi, buffer, size);
    SPI_SELECT(spi, 0x500, false);

    SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetDeviceId( void )
{
    return SX1262;
}

void SX126xAntSwOn( void )
{
}

void SX126xAntSwOff( void )
{
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

/*!
 * \brief DIO 0 IRQ callback
 */
extern "C" void RadioOnDioIrq( void* context );

px4_sem_t irq_sem;
bool irq_flag = false;

static int irq_handler(int irq, void *context, void *arg)
{
    RadioOnDioIrq(NULL);
    //irq_flag = true;
    //px4_sem_post(&irq_sem);

    return 0;
}

uint32_t RtcAlarmRemaining();
/*
extern "C" void BoardLowPowerHandler()
{
    uint32_t remaining;
    remaining = RtcAlarmRemaining();
    if (remaining == 0)
        return;
    struct timespec ts;
    printf("%d\n", remaining);
    ts.tv_sec = remaining / 1000000;
    ts.tv_nsec = (remaining%1000000) * 1000;
    px4_sem_timedwait(&irq_sem, &ts);
}
*/

void lora_peripherals_init()
{
    px4_sem_init(&irq_sem, 1, 0);

    if (spi == NULL) {
        spi = px4_spibus_initialize(PX4_SPI_BUS_EXTERNAL1);
        SPI_SETFREQUENCY(spi, 10 * 1000 * 1000);
        SPI_SETBITS(spi, 8);
        SPI_SETMODE(spi, SPIDEV_MODE0);
    }

    stm32_gpiosetevent(
        (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTI|GPIO_PIN10),
        true, false, false, irq_handler, NULL
    );
}

/*!
 * Redefinition of rand() and srand() standard C functions.
 * These functions are redefined in order to get the same behavior across
 * different compiler toolchains implementations.
 */
// Standard random functions redefinition start
#define RAND_LOCAL_MAX 2147483647L

static uint32_t next = 1;

int32_t rand1( void )
{
    return ( ( next = next * 1103515245L + 12345L ) % RAND_LOCAL_MAX );
}

void srand1( uint32_t seed )
{
    next = seed;
}
// Standard random functions redefinition end

int32_t randr( int32_t min, int32_t max )
{
    return ( int32_t )rand1( ) % ( max - min + 1 ) + min;
}

void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    while( size-- )
    {
        *dst++ = *src++;
    }
}

void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    dst = dst + ( size - 1 );
    while( size-- )
    {
        *dst-- = *src++;
    }
}

void memset1( uint8_t *dst, uint8_t value, uint16_t size )
{
    while( size-- )
    {
        *dst++ = value;
    }
}

#include "systime.h"

SysTime_t SysTimeAdd( SysTime_t a, SysTime_t b )
{
    SysTime_t c =  { .Seconds = 0, .SubSeconds = 0 };

    c.Seconds = a.Seconds + b.Seconds;
    c.SubSeconds = a.SubSeconds + b.SubSeconds;
    if( c.SubSeconds >= 1000 )
    {
        c.Seconds++;
        c.SubSeconds -= 1000;
    }
    return c;
}


SysTime_t SysTimeSub( SysTime_t a, SysTime_t b )
{
    SysTime_t c = { .Seconds = 0, .SubSeconds = 0 };

    c.Seconds = a.Seconds - b.Seconds;
    c.SubSeconds = a.SubSeconds - b.SubSeconds;
    if( c.SubSeconds < 0 )
    {
        c.Seconds--;
        c.SubSeconds += 1000;
    }
    return c;
}

SysTime_t SysTimeGet( void )
{
    SysTime_t sysTime = { .Seconds = 0, .SubSeconds = 0 };

    sysTime.Seconds = RtcGetCalendarTime( ( uint16_t* )&sysTime.SubSeconds );

    return sysTime;
}

void SysTimeSet( SysTime_t sysTime )
{
    /* stub */
}

SysTime_t SysTimeGetMcuTime( void )
{
    SysTime_t calendarTime = { .Seconds = 0, .SubSeconds = 0 };

    // TODO
    calendarTime.Seconds = RtcGetCalendarTime( ( uint16_t* )&calendarTime.SubSeconds );

    return calendarTime;
}
