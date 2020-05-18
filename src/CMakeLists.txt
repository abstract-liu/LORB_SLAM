add_library( mylib SHARED
	bundle_adjust.cpp 
	camera.cpp
	frame.cpp
	local_mapping.cpp 
	map.cpp
	map_point.cpp
	matcher.cpp
	viewer.cpp
	visual_odometry.cpp
	ORBextractor.cpp
	)

target_link_libraries( mylib ${THIRD_PARTY_LIBS})





