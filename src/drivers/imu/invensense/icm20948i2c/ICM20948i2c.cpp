/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "ICM20948i2c.hpp"
#include "InvenSense_ICM20948i2c_registers.hpp"

using namespace InvenSense_ICM20948;

#define ICM20948I2C_BASEADDR_DEFAULT 0x68

ICM20948i2c::ICM20948i2c(I2CSPIBusOption bus_option, const int bus, int bus_frequency) :
    I2C(DRV_IMU_DEVTYPE_ICM20948I2C, MODULE_NAME, bus, ICM20948I2C_BASEADDR_DEFAULT, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus)
{

}

int ICM20948i2c::init()
{
	set_device_address(ICM20948I2C_BASEADDR_DEFAULT);

	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	PX4_DEBUG("addr: %d", get_device_address());


    //wake up and auto select time source
    setRegister((uint8_t)REG_BANK_SEL_BIT::USER_BANK_0,
                (uint8_t)Register::BANK_0::PWR_MGMT_1,
                0x01);

    //disable I2C master, (and other stuff)
    setRegister((uint8_t)REG_BANK_SEL_BIT::USER_BANK_0,
                (uint8_t)Register::BANK_0::USER_CTRL,
                0x00);

    //enable I2C bridge
    setRegister((uint8_t)REG_BANK_SEL_BIT::USER_BANK_0,
                (uint8_t)Register::BANK_0::INT_PIN_CFG,
                0x02);


	ScheduleOnInterval(10000000);

	return PX4_OK;
}

void ICM20948i2c::exit_and_cleanup()
{
	PX4_DEBUG("exit and clean");
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM20948i2c::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("An Info");
}

int ICM20948i2c::probe()
{
	const uint8_t whoami = readRegister((uint8_t)REG_BANK_SEL_BIT::USER_BANK_0,(uint8_t)Register::BANK_0::WHO_AM_I);

	if (whoami != WHOAMI) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

    PX4_INFO("Who am I OK!");

	return PX4_OK;
}

void ICM20948i2c::RunImpl()
{
	//const hrt_abstime now = hrt_absolute_time();

	//PX4_INFO("Do measurement - i dont know what to do...");
}

void ICM20948i2c::setRegister(uint8_t reg, uint8_t value)
{
	uint8_t buff[2];
	buff[0] = reg;
	buff[1] = value;
	int ret = transfer(buff, 2, nullptr, 0);

	if (PX4_OK != ret) {
		PX4_DEBUG("setRegister : i2c::transfer returned %d", ret);
	}
}

uint8_t ICM20948i2c::readRegister(uint8_t reg)
{
	uint8_t rcv{};
	int ret = transfer(&reg, 1, &rcv, 1);

	if (PX4_OK != ret) {
		PX4_DEBUG("readRegister : i2c::transfer returned %d", ret);
	}

	return rcv;
}

void ICM20948i2c::setBank(uint8_t bank)
{
    setRegister((uint8_t) (Register::BANK_0::REG_BANK_SEL),bank);
}

void ICM20948i2c::setRegister(uint8_t bank, uint8_t reg, uint8_t value)
{
    setBank(bank);
    setRegister(reg,value);
}

uint8_t ICM20948i2c::readRegister(uint8_t bank, uint8_t reg)
{
    setBank(bank);
    return readRegister(reg);
}


