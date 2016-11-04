add_library(drakeQPController
  qp_controller.cc
  rigid_body_tree_utils.cc
  humanoid_status.cc)
target_link_libraries(drakeQPController
  drakeRBSystem
  drakeOptimization
  drakeSide)

if(snopt_FOUND)
  # drake_add_cc_test(NAME valkyrie_balancing_test SIZE medium)  # FIXME: see #4000
  # target_link_libraries(valkyrie_balancing_test
  #   drakeQPController
  #   GTest::GTest GTest::Main)
endif()

add_subdirectory(system)
