px4_add_module(

	MODULE drivers__i2c_dummy
	MAIN i2c_dummy
	COMPILE_FLAGS
	SRCS
		i2c_dummy.cpp
	DEPENDS
)
