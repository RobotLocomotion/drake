
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)

add_executable(testDrakeSystemFrames testDrakeSystemFrames.cpp)
target_link_libraries(testDrakeSystemFrames drakeSystem)
add_test(NAME testDrakeSystemFrames COMMAND testDrakeSystemFrames)