# drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancing REQUIRES gurobi COMMAND runValkyrieBalancing)  # FIXME: see #2839

if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND UNIX) # FIXME: see #3147.
  drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancingPerturb REQUIRES gurobi lcm libbot yaml-cpp COMMAND runValkyrieBalancingPerturb)
endif()

if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND UNIX) # FIXME: see #3147.
  drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieWalking REQUIRES gurobi lcm libbot yaml-cpp COMMAND runValkyrieWalking SIZE large)
endif()

drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieVisualize OPTIONAL bullet gurobi snopt COMMAND runValkyrieVisualize)

add_subdirectory(test)

if(lcm_FOUND)
  add_executable(valkyrie_ik_test valkyrie_IK_test.cc)
  target_link_libraries(valkyrie_ik_test
          drakeCommon
          drakeIK
          drakeLcm
          drakeRigidBodyPlant
          drakeSystemFramework
          drakeSystemAnalysis
          drakeRBM)
endif()