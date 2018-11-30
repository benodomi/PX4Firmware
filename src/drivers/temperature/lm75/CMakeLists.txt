px4_add_module(

	MODULE drivers__lm75
	MAIN lm75
	COMPILE_FLAGS
	SRCS
		lm75.cpp
	DEPENDS
      drivers__device
)
