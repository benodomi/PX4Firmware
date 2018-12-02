#px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common IO px4io-v2)
#
# user-configurable UART ports
#set(board_serial_ports
#	GPS1:/dev/ttyS3
#	TEL1:/dev/ttyS1
#	TEL2:/dev/ttyS2
#	TEL4:/dev/ttyS6)
#
#set(px4_constrained_flash_build 1)
#
#set(config_uavcan_num_ifaces 2)
#
#set(config_bl_file ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/extras/px4fmuv3_bl.bin)
#
#set(config_module_list
#
#

px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v2
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	BOOTLOADER ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/extras/px4fmuv3_bl.bin
	IO px4_io-v2_default
	#TESTING
	CONSTRAINED_FLASH
	#UAVCAN_INTERFACES 2

	SERIAL_PORTS
		GPS1:/dev/ttyS0
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS3

	DRIVERS

        #barometer
        #differential_pressure

    #   differential_pressure/ms4525
    #   differential_pressure/ms5525
        differential_pressure/sdp3x

        #distance_sensor
        #magnetometer
        #telemetry

        rotor_frequency/pcf8583

        #imu/adis16448
        barometer/ms5611
        #blinkm
        #imu/bmi160
        #barometer/bmp280
        #bst
        #camera_trigger
        telemetry/frsky_telemetry
        gps
        #hott
        #iridiumsbd
        #irlock
        imu/l3gd20
        imu/lsm303d
        magnetometer/hmc5883
        magnetometer/lis3mdl
        #mb12xx
        #mkblctrl
        imu/mpu6000
    #    imu/mpu9250
        #oreoled
        #protocol_splitter
        pwm_input
        #pwm_out_sim
        px4flow
        px4fmu
        px4io
        rgbled
        stm32
        stm32/adc
        stm32/tone_alarm
        #tap_esc
        vmount


        # distance sensors
    #    distance_sensor/ll40ls
    #    #distance_sensor/mb12xx
    #    distance_sensor/sf0x
    #    distance_sensor/sf1xx
    #    distance_sensor/srf02
    #    distance_sensor/teraranger
    #    distance_sensor/tfmini
        #distance_sensor/ulanding

    #
    # System commands
    #
    SYSTEMCMDS
        bl_update
        #config
        #dumpfile
        #esc_calib
        hardfault_log
        #led_control
        mixer
        #motor_ramp
        #motor_test
        mtd
        #nshterm
        param
        perf
        pwm
        reboot
        #sd_bench
        top
        #topic_listener
        tune_control
        ver

    #
    # Testing
    #
    #distance_sensor/sf0x/sf0x_tests
    #test_ppm
    #lib/controllib/controllib_test
    #lib/rc/rc_tests

    MODULES

        sensors
        #commander/commander_tests
        #mavlink/mavlink_tests
        #mc_pos_control/mc_pos_control_tests
        #uORB/uORB_tests
        #tests

        #
        # General system control
        #
    #   camera_feedback
        commander
        events
        #gpio_led
        land_detector
        load_mon
        mavlink
        navigator
        #uavcan

        #
        # Estimation modules
        #
        #attitude_estimator_q
        ekf2
        #local_position_estimator
        #position_estimator_inav
        #landing_target_estimator
        #wind_estimator

        #
        # Vehicle Control
        #
        fw_att_control
        fw_pos_control_l1

        rw_att_control
        #rw_pos_control_l1

        #gnd_att_control
        #gnd_pos_control
        mc_att_control
        mc_pos_control
        #vtol_att_control


        #
        # Logging
        #
        logger
        #sdlog2

        #
        # Library modules
        #
        dataman

    EXAMPLES
		#bottle_drop # OBC challenge
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		#px4_mavlink_debug # Tutorial code from https://px4.io/dev/debug_values
		#px4_simple_app # Tutorial code from https://px4.io/dev/px4_simple_app
		#rover_steering_control # Rover example app
		#segway

        rotor_frequency_consumer
)



#set(flight_tasks_to_remove Orbit)
