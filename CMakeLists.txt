project(marm_code)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_RPATH "/opt/unitree_robotics/lib")
set(CMAKE_INSTALL_RPATH "/opt/unitree_robotics/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2 /opt/unitree_robotics/include /opt/unitree_robotics/include/ddscxx)
link_directories(/opt/unitree_robotics/lib)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

add_executable(joint_angle_control src/joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(multiple_joint_angle_control src/multiple_joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(joint_enable_control src/joint_enable_control.cpp src/msg/ArmString_.cpp)
add_executable(arm_zero_control src/arm_zero_control.cpp src/msg/ArmString_.cpp)
add_executable(get_arm_joint_angle src/get_arm_joint_angle.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
add_executable(multi_joint_slider_control src/multi_joint_slider_control.cpp src/msg/ArmString_.cpp)
add_executable(fkine src/fkine.cpp src/msg/PubServoInfo_.cpp)