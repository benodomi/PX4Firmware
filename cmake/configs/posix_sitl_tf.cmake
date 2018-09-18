#px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common IO px4io-v2)
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)


#set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# SIMULATOR support modules
	#

	#drivers/barometer
	drivers/differential_pressure
	drivers/distance_sensor
	#drivers/telemetry

	drivers/batt_smbus
	drivers/camera_trigger
	drivers/gps
	drivers/linux_gpio
	drivers/pwm_out_sim
	drivers/vmount

	modules/sensors

	platforms/posix/drivers/tonealrmsim


	#
	# System commands
	#
	#systemcmds/bl_update
	#systemcmds/config
	#systemcmds/dumpfile
	systemcmds/esc_calib
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/motor_ramp
	#systemcmds/mtd
	#systemcmds/nshterm
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/reboot
	systemcmds/shutdown
	systemcmds/sd_bench
	systemcmds/top
	systemcmds/topic_listener
	systemcmds/tune_control
	systemcmds/ver


	#
	# Testing
	#
	drivers/distance_sensor/sf0x/sf0x_tests
	#drivers/test_ppm
	#lib/controllib/controllib_test
	lib/rc/rc_tests
	modules/commander/commander_tests
	modules/mavlink/mavlink_tests
	#modules/mc_pos_control/mc_pos_control_tests
	modules/uORB/uORB_tests
	systemcmds/tests

	#
	# General system control
	#
	modules/camera_feedback
	modules/commander
	modules/events
	#modules/gpio_led
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator
	modules/replay
	modules/simulator
	#modules/uavcan

	#
	# Estimation modules
	#
	#modules/attitude_estimator_q
	modules/ekf2
	#modules/local_position_estimator
	#modules/position_estimator_inav
	#modules/landing_target_estimator
	#modules/wind_estimator

	#
	# Vehicle Control
	#
	modules/fw_att_control
	modules/fw_pos_control_l1
	
	modules/ag_att_control
	#modules/ag_pos_control_l1 #TF-TODO: tento modul jeste neexistuje.
	
	#modules/gnd_att_control
	#modules/gnd_pos_control
	#modules/mc_att_control
	#modules/mc_pos_control
	modules/vtol_att_control


	#
	# Logging
	#
	modules/logger

	#
	# Library modules
	#
	modules/dataman

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control
)

set(config_sitl_viewer jmavsim CACHE STRING "viewer for sitl")
set_property(CACHE config_sitl_viewer PROPERTY STRINGS "jmavsim;none")

set(config_sitl_debugger disable CACHE STRING "debugger for sitl")
set_property(CACHE config_sitl_debugger PROPERTY STRINGS "disable;gdb;lldb")

# If the environment variable 'replay' is defined, we are building with replay
# support. In this case, we enable the orb publisher rules.
set(REPLAY_FILE "$ENV{replay}")
if(REPLAY_FILE)
	message("Building with uorb publisher rules support")
	add_definitions(-DORB_USE_PUBLISHER_RULES)
endif()
