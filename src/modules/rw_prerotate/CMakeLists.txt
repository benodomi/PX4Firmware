px4_add_module(
    MODULE modules__rw_prerotate
    MAIN rw_prerotate
    COMPILE_FLAGS -Os
    STACK_MAIN 1200
    SRCS
        rw_prerotate.cpp
    DEPENDS
        platforms__common
    )
