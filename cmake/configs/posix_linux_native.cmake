
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list


	drivers/batt_smbus
	drivers/vmount

	#
	# System commands
	#
	#systemcmds/bl_update
	#systemcmds/config
	#systemcmds/dumpfile
	#systemcmds/esc_calib
	#systemcmds/led_control
	#systemcmds/mixer
	#systemcmds/motor_ramp
	#systemcmds/mtd
	#systemcmds/nshterm
	systemcmds/param
	systemcmds/perf
	#systemcmds/pwm
	systemcmds/reboot
	systemcmds/shutdown
	#systemcmds/sd_bench
	systemcmds/top
	systemcmds/topic_listener
	systemcmds/tune_control
	systemcmds/ver

   #testing
   #examples/px4_simple_app

   drivers/mlab


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
