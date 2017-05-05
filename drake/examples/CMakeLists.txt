drake_install_headers(
    examples_package_map.h)

add_executable(simple_continuous_time_system simple_continuous_time_system.cc)
target_link_libraries(simple_continuous_time_system
    drakeSystemAnalysis)
drake_add_test(NAME simple_continuous_time_system COMMAND simple_continuous_time_system)

add_executable(simple_discrete_time_system simple_discrete_time_system.cc)
target_link_libraries(simple_discrete_time_system
    drakeSystemAnalysis)
drake_add_test(NAME simple_discrete_time_system COMMAND simple_discrete_time_system)

add_executable(simple_mixed_continuous_and_discrete_time_system simple_mixed_continuous_and_discrete_time_system.cc)
target_link_libraries(simple_mixed_continuous_and_discrete_time_system
    drakeSystemAnalysis)
drake_add_test(NAME simple_mixed_continuous_and_discrete_time_system COMMAND simple_mixed_continuous_and_discrete_time_system)


# recurse into all example subdirectories
# Note: the GLOB workflow is typically not recommended due to the fact that
# CMake will not be able to detect changes, but we currently support users
# adding contributed examples by installing directly into
# examples/MY_EXAMPLE. See #1939.
file(GLOB children *)
foreach(child ${children})
  if(EXISTS ${child}/CMakeLists.txt)
    add_subdirectory(${child})
  endif()
endforeach()
