
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list


	drivers/batt_smbus
	drivers/vmount

	#
	# System commands
	#

	systemcmds/param
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/ver
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/shutdown
	systemcmds/topic_listener
	systemcmds/tune_control
	systemcmds/perf

   #testing
   examples/rotor_frequency_consumer

   drivers/mlab_dummy
   drivers/rotor_frequency/PCF8583
   drivers/temperature/LM75


	#
	# General system control
	#
	modules/mavlink

	#
	# Logging
	#
	modules/logger

	#
	# Library modules
	#
	modules/dataman

)
