cmake_minimum_required(VERSION 2.8.3)
project(Acquisition_Platform_ros)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11 -march=native")

include(${CMAKE_CURRENT_SOURCE_DIR}/Vars.cmake)
add_definitions(${DEBUG_OPTS} ${SENSORS} ${ODOM} ${COMMON_VARS})

find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
	nav_msgs
	tf
	roscpp
)


find_package(Threads REQUIRED)

# Find Acquisition_Platform
find_package(Acquisition_Platform REQUIRED COMPONENTS
	acq_pl_library
)
include_directories(${Acquisition_Platform_INCLUDE_DIRS})


catkin_package(
#	INCLUDE_DIRS include src
#	LIBRARIES Acquisition_Platform_ros
	CATKIN_DEPENDS sensor_msgs nav_msgs roscpp tf Acquisition_Platform
#	DEPENDS system_lib
)

include_directories(
	${PROJECT_SOURCE_DIR}/src
	${catkin_INCLUDE_DIRS}
)


add_executable(acq_pl_node
	src/main.cpp src/host_ros.cpp
)
target_link_libraries(acq_pl_node
	${Acquisition_Platform_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	m	#Math Library
	${catkin_LIBRARIES}
)
