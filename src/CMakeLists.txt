add_library( mylib SHARED
	bundle_adjust.cpp 
	camera.cpp
	frame.cpp
	map.cpp
	map_point.cpp
	viewer.cpp
	visual_odometry.cpp

	)

target_link_libraries( mylib ${THIRD_PARTY_LIBS})





