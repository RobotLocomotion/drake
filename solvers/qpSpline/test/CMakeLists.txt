add_executable(testSplineGeneration testSplineGeneration.cpp)
target_link_libraries(testSplineGeneration splineGeneration)
include_directories( .. )
add_test(NAME testDrakeGradientUtil COMMAND testDrakeGradientUtil)
