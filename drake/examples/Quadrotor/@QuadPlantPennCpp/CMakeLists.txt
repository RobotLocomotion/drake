
include_directories(${CMAKE_SOURCE_DIR}/util)
add_mex(dynamics_no_grad dynamics_no_grad.cpp)
target_link_libraries(dynamics_no_grad drakeMexUtil drakeGeometryUtil)