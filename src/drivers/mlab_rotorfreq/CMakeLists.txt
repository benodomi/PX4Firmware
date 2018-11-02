px4_add_module(

	MODULE drivers__mlab_rotorfreq
	MAIN mlab_rotorfreq
	COMPILE_FLAGS
	SRCS
		mlab_rotorfreq.cpp
	DEPENDS
      drivers__device
)
