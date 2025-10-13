cmake_minimum_required(VERSION 3.5)
project(yarp_exercises)

find_package(YARP REQUIRED)
include_directories(${YARP_INCLUDE_DIRS})

add_executable(real_time_sender ejercicio4_real_time_sender.cpp)
target_link_libraries(real_time_sender ${YARP_LIBRARIES})

add_executable(real_time_receiver ejercicio4_real_time_receiver.cpp)
target_link_libraries(real_time_receiver ${YARP_LIBRARIES})

