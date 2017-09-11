cmake_minimum_required(VERSION 2.8.3)
project(wm_estop)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs)

catkin_package(LIBRARIES wm_estop CATKIN_DEPENDS roscpp std_msgs std_srvs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(wm_estop_node
		src/wm_estop.cpp
		src/SerialManager.cpp
		src/serial/PosixSignalDispatcher.cpp
		src/serial/SerialPort.cpp
		src/serial/SerialStream.cc
		src/serial/SerialStreamBuf.cc
)

add_dependencies(wm_estop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(wm_estop_node
   ${catkin_LIBRARIES}
)

target_link_libraries(wm_estop_node pthread)