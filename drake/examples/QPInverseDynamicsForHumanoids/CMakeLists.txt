set(qp_inverse_dynamics_controller_src
  qp_controller.cc
  rigid_body_tree_utils.cc
  humanoid_status.cc
  example_balancing_controller.cc
  lcm_utils.cc)
set_source_files_properties(${qp_inverse_dynamics_controller_src} PROPERTIES COMPILE_FLAGS -g)

add_library(drakeQPController
  ${qp_inverse_dynamics_controller_src})
target_link_libraries(drakeQPController
  drakeRBSystem
  drakeOptimization
  drakeLCMTypes
  lcmtypes_bot2-core-cpp
  drakeSide)

drake_find_package(gurobi_c CONFIG)
if(gurobi_c_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_test SIZE medium)
  target_link_libraries(valkyrie_balancing_test
    drakeQPController
    GTest::GTest GTest::Main)
endif()

add_subdirectory(system)
