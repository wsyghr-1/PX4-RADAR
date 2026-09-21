px4_add_module(
	MODULE modules__motion_detector
	MAIN motion_detector
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		MotionDetector.cpp
		MotionDetector.hpp
	DEPENDS
		mathlib
		px4_work_queue
)
