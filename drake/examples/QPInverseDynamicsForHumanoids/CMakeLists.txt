add_library(drakeQPControllerCommon qp_controller_common.cc)
target_link_libraries(drakeQPControllerCommon
  drakeRBM
  drakeSide)

if (yaml-cpp_FOUND)
  add_library(drakeQPController humanoid_status.cc qp_controller.cc)
  target_link_libraries(drakeQPController
    drakeOptimization
    drakeQPControllerCommon)
  add_subdirectory(param_parsers)
endif()

if(lcm_FOUND)
  add_library(drakeQPControllerLcmTranslator lcm_utils.cc)
  target_link_libraries(drakeQPControllerLcmTranslator
    drakeRBM
    drakeLCMTypes
    drakeLCMUtil
    drakeQPControllerCommon
    lcmtypes_bot2-core-cpp)
endif()

add_subdirectory(system)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

