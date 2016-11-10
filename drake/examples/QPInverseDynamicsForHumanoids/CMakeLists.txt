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

if(lcm_FOUND)
  add_library(drakeQPControllerLcmTranslator lcm_utils.cc)
  target_link_libraries(drakeQPControllerLcmTranslator
    drakeLCMTypes
    drakeLCMUtil
    lcmtypes_bot2-core-cpp)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(system)
