
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)

add_executable(runPassivePendulum runPassive.cpp)
target_link_libraries(runPassivePendulum drakeSystem)
add_test(NAME runPassivePendulum COMMAND runPassivePendulum)