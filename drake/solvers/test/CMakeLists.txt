
include_directories(${PROJECT_SOURCE_DIR}/core)
include_directories(${PROJECT_SOURCE_DIR}/solvers)
add_executable(simpleFunctionTest simpleFunctionTest.cpp)

add_test(NAME simpleFunctionTest COMMAND simpleFunctionTest)