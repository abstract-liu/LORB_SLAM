
add_executable( Simple-ORB-SLAM main.cpp)
target_link_libraries( Simple-ORB-SLAM mylib)

add_executable( slam-test test.cpp)
target_link_libraries( slam-test mylib)

