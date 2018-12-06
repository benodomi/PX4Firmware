px4_add_module(
    MODULE examples__rotorfreq_consumer
    MAIN rotorfreq_consumer
    STACK_MAIN 2000
    SRCS
        rotorfreq_consumer.c
    DEPENDS
        platforms__common
    )
