
include_directories(${CMAKE_SOURCE_DIR}/util)
add_mex(manipulatorDynamics manipulatorDynamics.cpp)
target_link_libraries(manipulatorDynamics drakeMexUtil)