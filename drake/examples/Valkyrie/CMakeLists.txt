add_library_with_exports(LIB_NAME drakeValkyrieConstants SOURCE_FILES
    valkyrie_constants.cc)
target_link_libraries(drakeValkyrieConstants drakeCommon)
drake_install_libraries(drakeValkyrieConstants)
drake_install_headers(valkyrie_constants.h)

if (lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeRobotCommandToDesiredEffortConverter SOURCE_FILES
      robot_command_to_desired_effort_converter.cc)
  target_link_libraries(drakeRobotCommandToDesiredEffortConverter
      drakeLcmSystem)
  drake_install_libraries(drakeRobotCommandToDesiredEffortConverter)
  drake_install_headers(robot_command_to_desired_effort_converter.h)

  add_library_with_exports(LIB_NAME drakeActuatorEffortToRigidBodyPlantInputConverter SOURCE_FILES
      actuator_effort_to_rigid_body_plant_input_converter.cc)
  target_link_libraries(drakeActuatorEffortToRigidBodyPlantInputConverter
      drakeLcmSystem)
  drake_install_libraries(drakeActuatorEffortToRigidBodyPlantInputConverter)
  drake_install_headers(actuator_effort_to_rigid_body_plant_input_converter.h)

  add_library_with_exports(LIB_NAME drakeRobotStateEncoder SOURCE_FILES
      robot_state_encoder.cc)
  target_link_libraries(drakeRobotStateEncoder
      drakeGeometryUtil
      drakeLcmSystem
      drakeLCMUtil
      drakeRigidBodyPlant
      drakeSide
      drakeRobotStateLcmTranslator
      )
  drake_install_libraries(drakeRobotStateEncoder)
  drake_install_headers(robot_state_encoder.h)

  add_library_with_exports(LIB_NAME drakeRobotStateDecoder SOURCE_FILES
      robot_state_decoder.cc)
  target_link_libraries(drakeRobotStateDecoder
      drakeLcmSystem
      drakeLCMUtil
      drakeRigidBodyPlant
      drakeRobotStateLcmTranslator
      )
  drake_install_libraries(drakeRobotStateDecoder)
  drake_install_headers(robot_state_decoder.h)

  add_executable(valkyrie_simulation valkyrie_simulation.cc)
  target_link_libraries(valkyrie_simulation
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeRobotStateEncoder
      drakeSystemAnalysis
      drakeSystemFramework
      drakeSystemPrimitives
      drakeActuatorEffortToRigidBodyPlantInputConverter
      drakeRobotCommandToDesiredEffortConverter
      drakeValkyrieConstants)

  add_executable(valkyrie_pd_ff_controller valkyrie_pd_ff_controller.cc)
  target_link_libraries(valkyrie_pd_ff_controller
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeRobotStateDecoder
      drakeSystemFramework
      drakeValkyrieConstants)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
