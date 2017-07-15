cmake_minimum_required(VERSION 2.8.3)
project(wm_supervisor)

find_package(catkin REQUIRED COMPONENTS roscpp std_srvs)

catkin_package(LIBRARIES wm_supervisor CATKIN_DEPENDS roscpp std_srvs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(wm_supervisor_node
		src/wm_supervisor.cpp
		src/SerialManager.cpp
		src/serial/PosixSignalDispatcher.cpp
		src/serial/SerialPort.cpp
		src/serial/SerialStream.cc
		src/serial/SerialStreamBuf.cc
)

add_dependencies(wm_supervisor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(wm_supervisor_node
   ${catkin_LIBRARIES}
)

target_link_libraries(wm_supervisor_node pthread)