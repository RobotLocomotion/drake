
include_directories(${PROJECT_SOURCE_DIR}/systems)
add_executable(testAffineSystem testAffineSystem.cpp)
add_test(NAME testAffineSystem COMMAND testAffineSystem)
