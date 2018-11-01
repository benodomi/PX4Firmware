px4_add_module(

	MODULE drivers__mlab_dummy
	MAIN mlab_dummy
	COMPILE_FLAGS
	SRCS
		mlab_dummy.cpp
	DEPENDS
)
