
include_directories(${PROJECT_SOURCE_DIR}/core)
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)
include_directories(${PROJECT_SOURCE_DIR}/systems/plants)
include_directories(${PROJECT_SOURCE_DIR}/systems/plants/joints)
include_directories(${PROJECT_SOURCE_DIR}/systems/controllers)
include_directories(${PROJECT_SOURCE_DIR}/util)

if (LCM_FOUND AND snopt_c_FOUND)
  add_executable(pendulumURDFDynamicsTest urdfDynamicsTest.cpp)
  target_link_libraries(pendulumURDFDynamicsTest drakeRBM)
  add_test(NAME pendulumURDFDynamicsTest COMMAND pendulumURDFDynamicsTest)
endif(LCM_FOUND AND snopt_c_FOUND)
