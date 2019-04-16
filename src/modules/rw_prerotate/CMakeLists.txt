px4_add_module(
    MODULE modules__rw_prerotate
    MAIN rw_prerotate
	COMPILE_FLAGS
    STACK_MAIN 1024
	STACK_MAX 4000
	COMPILE_FLAGS
		-Wno-cast-align # TODO: fix and enable
	INCLUDES
    SRCS
        rw_prerotate.cpp
    DEPENDS
        platforms__common
		modules__uORB
        version
	UNITY_BUILD
    )
