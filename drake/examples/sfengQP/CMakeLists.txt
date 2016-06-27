add_executable(testQP test.cpp QPEstimator.cpp QPController.cpp sfUtils.cpp MRDLogger.cpp sfRobotState.cpp EigenQuadProg.cpp)
add_definitions(-DVALKYRIE_URDF_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\")
target_link_libraries(testQP drakeRBSystem drakeOptimization)
