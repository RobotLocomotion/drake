add_executable(testQP test.cpp QPController.cpp RBTUtils.cpp HumanoidState.cpp)
add_definitions(-DVALKYRIE_URDF_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\")
target_link_libraries(testQP drakeRBSystem drakeOptimization)
