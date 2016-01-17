
include_directories(${PROJECT_SOURCE_DIR}/core)
include_directories(${PROJECT_SOURCE_DIR}/solvers)
include_directories(${PROJECT_SOURCE_DIR}/util)

add_executable(testOptimizationProblem testOptimizationProblem.cpp)
target_link_libraries(testOptimizationProblem drakeOptimization)
add_test(NAME testOptimizationProblem COMMAND testOptimizationProblem)
