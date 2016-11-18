set(qp_inverse_dynamics_controller_src
  humanoid_status.cc
  example_qp_input_for_valkyrie.cc
  qp_controller.cc
  rigid_body_tree_utils.cc)

add_library(drakeQPController ${qp_inverse_dynamics_controller_src})
target_link_libraries(drakeQPController
  drakeOptimization
  drakeRBM
  drakeSide)

if(lcm_FOUND)
  add_library(drakeQPControllerLcmTranslator lcm_utils.cc)
  target_link_libraries(drakeQPControllerLcmTranslator
    drakeRBM
    drakeLCMTypes
    drakeLCMUtil
    lcmtypes_bot2-core-cpp)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
