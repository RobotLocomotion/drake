
if (simulink_FOUND)
  add_mex(DCSFunction DCSFunction.cpp)
endif()

include_directories(${PROJECT_SOURCE_DIR}/core)
include_directories(${PROJECT_SOURCE_DIR}/util)
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)

if (LCM_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem SOURCE_FILES LCMSystem.cpp)
  add_dependencies(drakeLCMSystem drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(drakeLCMSystem lcm)

  # from http://stackoverflow.com/a/29871891
  find_package(Threads REQUIRED)
  if (THREADS_HAVE_PTHREAD_ARG)
    set_property(TARGET drakeLCMSystem PROPERTY COMPILE_OPTIONS "-pthread")
    set_property(TARGET drakeLCMSystem PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
  endif()
  if (CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(drakeLCMSystem "${CMAKE_THREAD_LIBS_INIT}")
  endif()

  pods_install_headers(LCMSystem.h DESTINATION drake)
endif()

add_subdirectory(frames)
add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)

pods_install_headers(System.h LinearSystem.h Simulation.h DESTINATION drake)

add_subdirectory(test)
