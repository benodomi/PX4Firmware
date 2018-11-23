px4_add_module(

	MODULE drivers__mlab_temperature
	MAIN mlab_temperature
	COMPILE_FLAGS
	SRCS
		mlab_temperature.cpp
	DEPENDS
      drivers__device
)
