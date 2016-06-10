
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # TODO(#2372) These are warnings that we can't handle yet.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")
endif()

if(simulink_FOUND)
  add_mex(DCSFunction DCSFunction.cpp)
endif()

if(LCM_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem SOURCE_FILES LCMSystem.cpp)
  add_dependencies(drakeLCMSystem drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(drakeLCMSystem lcm)

  pods_install_libraries(drakeLCMSystem)
  drake_install_headers(LCMSystem.h)
  pods_install_pkg_config_file(drake-lcm-system
    LIBS -ldrakeLCMSystem
    REQUIRES lcm
    VERSION 0.0.1)
endif()

add_subdirectory(frames)
add_subdirectory(framework)
add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)

drake_install_headers(System.h LinearSystem.h Simulation.h)

add_subdirectory(test)
