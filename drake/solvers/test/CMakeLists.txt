
include_directories(${PROJECT_SOURCE_DIR}/core)
include_directories(${PROJECT_SOURCE_DIR}/solvers)

add_executable(testOptimizationProblem testOptimizationProblem.cpp)
add_test(NAME testOptimizationProblem COMMAND testOptimizationProblem)
