/**
 * MLAB temperature (i2c) pool interval 
 *
 * @reboot_required true
 * @group Sensors
 * @unit us
 */
PARAM_DEFINE_INT32(LM75_POOL, 1000000);

/**
 * MLAB temperature (i2c) i2c address 
 *
 * @reboot_required true
 * @group Sensors
 * @value 72 0x48
 * @value 73 0x49
 */
PARAM_DEFINE_INT32(LM75_ADDR, 72);


