cmake_minimum_required(VERSION 2.8.3)
project(Acquisition_Platform)

find_package(catkin REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11 -march=native -Wall")

include(${CMAKE_CURRENT_SOURCE_DIR}/Vars.cmake)
add_definitions(${DEBUG_OPTS} ${SENSORS} ${COMMON_VARS})

find_package(Threads REQUIRED)


catkin_package(
	INCLUDE_DIRS src/host src/common
	LIBRARIES acq_pl_library
#	DEPENDS
)

include_directories(
	${catkin_INCLUDE_DIRS}
	src/common
	src/host
)

## Declare a C++ library
add_library(acq_pl_library SHARED
	src/host/serial/serial.c
	src/host/serial/serial_communication.c
	src/host/host.c
#	src/host/main.c
)
target_link_libraries(acq_pl_library ${catkin_LIBRARIES})

add_executable(acq_pl
	src/host/main.c
)
target_link_libraries(acq_pl
	acq_pl_library
	${CMAKE_THREAD_LIBS_INIT}
)
