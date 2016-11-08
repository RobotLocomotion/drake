set(qp_inverse_dynamics_controller_src
  humanoid_status.cc
  example_qp_input_for_valkyrie.cc
  qp_controller.cc
  rigid_body_tree_utils.cc)
set_source_files_properties(${qp_inverse_dynamics_controller_src}
  PROPERTIES COMPILE_FLAGS -g)

add_library(drakeQPController ${qp_inverse_dynamics_controller_src})
target_link_libraries(drakeQPController
  drakeOptimization
  drakeRBSystem
  drakeSide)

drake_find_package(gurobi PKG_CONFIG)
if(gurobi_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_test SIZE medium)
  target_link_libraries(valkyrie_balancing_test
    drakeQPController
    GTest::GTest GTest::Main)
endif()

if(lcm_FOUND)
  add_library(drakeQPControllerLcmTranslator lcm_utils.cc)
  target_link_libraries(drakeQPControllerLcmTranslator
    drakeLCMTypes
    drakeLCMUtil
    lcmtypes_bot2-core-cpp)
endif()

drake_add_cc_test(NAME setpoint_test)
target_link_libraries(setpoint_test
  drakeQPController
  GTest::GTest GTest::Main)

add_subdirectory(system)
