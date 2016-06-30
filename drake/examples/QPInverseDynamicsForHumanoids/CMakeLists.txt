add_executable(testQP test.cpp QPController.cpp RBTUtils.cpp HumanoidState.cpp)
target_link_libraries(testQP drakeRBSystem drakeOptimization)
