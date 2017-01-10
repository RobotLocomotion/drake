if (lcm_FOUND)
  drake_add_cc_test(testMassSpringDamper EXTENSION cpp)
  target_link_libraries(testMassSpringDamper drakeRBSystem)

  if (Bullet_FOUND AND bot2-core_FOUND)
    drake_add_cc_test(lidarTest EXTENSION cpp)
    target_link_libraries(lidarTest drakeRBSystem drakeLCMTypes lcmtypes_bot2-core-cpp lcm)
  endif ()

  drake_add_cc_test(NAME testAccelerometer EXTENSION cpp)
  target_link_libraries(testAccelerometer drakeRBSystem)

  drake_add_cc_test(NAME testGyroscope EXTENSION cpp)
  target_link_libraries(testGyroscope drakeRBSystem)

  drake_add_cc_test(testMagnetometer EXTENSION cpp)
  target_link_libraries(testMagnetometer drakeRBSystem)

  add_executable(compareRigidBodySystems compareRigidBodySystems.cpp)
  target_link_libraries(compareRigidBodySystems drakeRBSystem GTest::GTest GTest::Main)
  drake_add_test(NAME compareAcrobotURDFtoSDF COMMAND compareRigidBodySystems Acrobot.urdf Acrobot.sdf WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/examples/Acrobot")

  if(snopt_FOUND)
    drake_add_test(NAME comparePriusURDFtoSDF COMMAND compareRigidBodySystems prius.sdf prius.urdf WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/automotive/models/prius" SIZE medium)
  endif ()

  drake_add_cc_test(NAME load_model_test SIZE medium)
  target_link_libraries(load_model_test drakeRBSystem)
endif ()

add_subdirectory(drake_joint)
add_subdirectory(rigid_body_system)
