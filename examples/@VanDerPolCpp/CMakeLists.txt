
include_directories(${CMAKE_SOURCE_DIR}/util)
add_mex(dynamicsRHS dynamicsRHS.cpp)
target_link_libraries(dynamicsRHS drakeUtil)