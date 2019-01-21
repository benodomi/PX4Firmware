uint64 timestamp			# time since system start (microseconds)
uint8 NUM_ACTUATOR_CONTROLS = 8
uint8 NUM_ACTUATOR_CONTROL_GROUPS = 4
uint8 INDEX_ROTOR_ROLL = 0
uint8 INDEX_ROTOR_PITCH = 1
uint8 INDEX_YAW = 2
uint8 INDEX_THROTTLE = 3
uint8 INDEX_PREROTATOR = 4
uint8 INDEX_RELEASE = 5
uint8 INDEX_LANDING_GEAR = 6
uint8 INDEX_SPARE = 7
uint8 GROUP_INDEX_ATTITUDE = 0
uint8 GROUP_INDEX_ATTITUDE_ALTERNATE = 1
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled
float32[8] control

# TOPICS actuator_controls_rw actuator_controls_rw_0 actuator_controls_rw_1 actuator_controls_rw_2 actuator_controls_rw_3
# TOPICS actuator_controls_virtual_rw
