cmake_minimum_required( VERSION 2.8)
project( Simple-ORB-SLAM)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")  


find_package( OpenCV 3.1 REQUIRED)
include_directories( ${OPENCV_INCLUDE_DIRS} ) 

find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR})

find_package(Pangolin REQUIRED)
include_directories( ${Pangolin_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS})

set( THIRD_PARTY_LIBS 
	${OpenCV_LIBS}
	${Pangolin_LIBRARIES}
	${EIGEN3_LIBS}
	${CERES_LIBRARIES}
	)




include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( example )
