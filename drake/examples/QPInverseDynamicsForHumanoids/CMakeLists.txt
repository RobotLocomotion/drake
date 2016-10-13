add_library(drakeQPController
  qp_controller.cc
  rigid_body_tree_utils.cc
  humanoid_status.cc)
target_link_libraries(drakeQPController
  drakeRBSystem
  drakeOptimization
  drakeSide)

drake_add_cc_test(NAME valkyrie_balancing_test SIZE large)
target_link_libraries(valkyrie_balancing_test
  drakeQPController
  GTest::GTest GTest::Main)

add_subdirectory(system)
