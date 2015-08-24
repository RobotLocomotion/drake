
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)
include_directories(${PROJECT_SOURCE_DIR}/systems/plants)
include_directories(${PROJECT_SOURCE_DIR}/systems/plants/joints)
include_directories(${PROJECT_SOURCE_DIR}/util)

if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)
  add_executable(runPendulumDynamics runDynamics.cpp)
  target_link_libraries(runPendulumDynamics drakeSystem drakeRBM drakeGeometryUtil)
  add_test(NAME runPendulumDynamics COMMAND runPendulumDynamics WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Pendulum)

  add_executable(runPendulumEnergyShaping runEnergyShaping.cpp)
  target_link_libraries(runPendulumEnergyShaping drakeSystem)
  add_test(NAME runPendulumEnergyShaping COMMAND runPendulumEnergyShaping)
endif()