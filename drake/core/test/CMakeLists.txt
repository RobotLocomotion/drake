
include_directories(${PROJECT_SOURCE_DIR}/core)
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/util)
include_directories(${PROJECT_SOURCE_DIR}/examples/Pendulum)
add_executable(testVector testVector.cpp)

add_test(NAME testVector COMMAND testVector)