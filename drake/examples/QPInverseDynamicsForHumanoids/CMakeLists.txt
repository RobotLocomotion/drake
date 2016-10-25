set(qp_inverse_dynamics_controller_src
  qp_controller.cc
  rigid_body_tree_utils.cc
  humanoid_status.cc
  example_qp_input_for_valkyrie.cc)
set_source_files_properties(${qp_inverse_dynamics_controller_src} PROPERTIES COMPILE_FLAGS -g)

add_library(drakeQPController
  ${qp_inverse_dynamics_controller_src})
target_link_libraries(drakeQPController
  drakeRBSystem
  drakeOptimization
  drakeLCMTypes
  lcmtypes_bot2-core-cpp
  drakeSide)

drake_find_package(gurobi PKG_CONFIG)
if(gurobi_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_test)
  target_link_libraries(valkyrie_balancing_test
    drakeQPController
    GTest::GTest GTest::Main)
endif()

if(lcm_FOUND)
  add_library(drakeQPControllerLcmTranslator lcm_utils.cc)
  target_link_libraries(drakeQPControllerLcmTranslator lcmtypes_bot2-core-cpp)
endif()

add_subdirectory(system)
