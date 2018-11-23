/**
 * MLAB temperature (i2c) pool interval 
 *
 * @reboot_required true
 * @group Sensors
 * @unit us
 */
PARAM_DEFINE_INT32(TEMPERATURE_POOL, 1000000);

/**
 * MLAB temperature (i2c) i2c address 
 *
 * @reboot_required true
 * @group Sensors
 * @value 81 0x51
 * @value 80 0x50
 */
PARAM_DEFINE_INT32(TEMPERATURE_ADDR, 81);


