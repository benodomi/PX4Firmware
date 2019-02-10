px4_add_module(
    MODULE modules__rw_prerotate
    MAIN rw_prerotate
    STACK_MAIN 1024
    SRCS
        rw_prerotate.cpp
    DEPENDS
        platforms__common
		modules__uORB
    )
