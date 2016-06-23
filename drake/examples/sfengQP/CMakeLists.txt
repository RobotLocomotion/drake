add_executable(testQP QPController.cpp sfUtils.cpp Logger.cpp sfRobotState.cpp mrdplot.cpp)
add_definitions(-DVALKYRIE_URDF_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\")
target_link_libraries(testQP drakeRBSystem drakeOptimization)
