add_subdirectory(test)
if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)

  add_library_with_exports(LIB_NAME drakeSimpleCar SOURCE_FILES simple_car.cc)
  pods_install_libraries(drakeSimpleCar)
  drake_install_headers(
    simple_car.h
    gen/driving_command.h
    gen/euler_floating_joint_state.h
    gen/simple_car_state.h
    )
  pods_install_pkg_config_file(drake-simple-car
    LIBS -ldrakeSimpleCar
    REQUIRES
    VERSION 0.0.1)

  add_dependencies(drakeSimpleCar drake_lcmtypes lcmtype_agg_hpp)
  add_executable(simple_car_demo simple_car_demo.cc)
  target_link_libraries(simple_car_demo drakeSimpleCar drakeRBSystem drakeLCMSystem)

  add_executable(carSimLCM simulateLCM.cpp)
  add_dependencies(carSimLCM drake_lcmtypes lcmtype_agg_hpp)
  target_link_libraries(carSimLCM drakeRBSystem drakeLCMSystem)

if(roscpp_FOUND)
  # message(STATUS "=============================================================")
  # get_cmake_property(_variableNames VARIABLES)
  # foreach (_variableName ${_variableNames})
  #     message(STATUS "${_variableName}=${${_variableName}}")
  # endforeach()
  # message(STATUS "=============================================================")

  add_executable(carSimLCMandROS simulateLCMandROS.cpp)
  # add_dependencies(carSimLCMandROS ${PROJECT_NAME}_generate_messages)
  target_link_libraries(carSimLCMandROS drakeRBM drakeRBSystem drakeLCMSystem ${roscpp_LIBRARIES} ${tf_LIBRARIES})
endif()

  add_executable(publishDrivingCommand publishDrivingCommand.cpp)
  add_dependencies(publishDrivingCommand drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(publishDrivingCommand lcm)
  add_test(NAME publishDrivingCommand COMMAND publishDrivingCommand 1.0 0.0)
endif (LCM_FOUND)
