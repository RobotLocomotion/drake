
if (simulink_FOUND)
  add_mex(DCSFunction DCSFunction.cpp)
endif()

include_directories(${PROJECT_SOURCE_DIR}/core)
include_directories(${PROJECT_SOURCE_DIR}/util)
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)

if (LCM_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem SOURCE_FILES LCMSystem.cpp)
  add_dependencies(drakeLCMSystem lcmtype_agg_hpp)
  target_link_libraries(drakeLCMSystem drake_lcmtypes)
  pods_use_pkg_config_packages(drakeLCMSystem lcm)

  pods_install_headers(LCMSystem.h DESTINATION drake)
endif()

add_subdirectory(frames)
add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)

pods_install_headers(System.h LinearSystem.h Simulation.h DESTINATION drake)

add_subdirectory(test)
