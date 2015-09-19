cmake_minimum_required (VERSION 2.6)

pods_find_pkg_config(bullet)

if (MATLAB_FOUND)
  add_rbm_mex(compareParsersmex)
  include_directories(${PROJECT_SOURCE_DIR}/util/test)
endif (MATLAB_FOUND)

add_executable(urdfKinTest urdfKinTest.cpp)
include_directories( .. )
target_link_libraries(urdfKinTest drakeRBM)
if (bullet_FOUND)
  add_executable(urdfCollisionTest urdfCollisionTest.cpp)
  include_directories( .. )
  target_link_libraries(urdfCollisionTest drakeRBM)
endif()

add_executable(urdfManipulatorDynamicsTest urdfManipulatorDynamicsTest.cpp)
target_link_libraries(urdfManipulatorDynamicsTest drakeRBM)

if (NOT WIN32)
  # this test does not compile on MSVC 2010 because of the use of variadic templates
  add_executable(testKinematicsCacheChecks testKinematicsCacheChecks.cpp)
  target_link_libraries(testKinematicsCacheChecks drakeRBM)
  add_test(NAME testKinematicsCacheChecks WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND testKinematicsCacheChecks)
endif()

macro(add_ik_cpp)
  add_executable(${ARGV} ${ARGV}.cpp)
  include_directories( .. )
  set_target_properties(${ARGV} PROPERTIES COMPILE_FLAGS "-fPIC")
  target_link_libraries(${ARGV} drakeRigidBodyConstraint drakeIK)
  add_test( NAME ${ARGV} WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND ${ARGV})
endmacro()

if (gurobi_FOUND)
  add_ik_cpp(testApproximateIK)
endif()

if (snopt_c_FOUND)
  add_ik_cpp(testIK)
  add_ik_cpp(testIKpointwise)
  add_ik_cpp(testIKtraj)
endif()

add_executable(debugManipulatorDynamics debugManipulatorDynamics.cpp)
include_directories( .. )
target_link_libraries(debugManipulatorDynamics drakeRBM)

include_directories(${PROJECT_SOURCE_DIR}/util/test)
add_executable(benchmarkAutoDiffScalar benchmarkAutoDiffScalar.cpp)
target_link_libraries(benchmarkAutoDiffScalar drakeRBM)

if (MATLAB_FOUND)
  macro(add_ikoptions_mex)
    add_mex(${ARGV} ${ARGV}.cpp)
    target_link_libraries(${ARGV} drakeRBM drakeMexUtil drakeIKoptions)
  endmacro()
  add_ikoptions_mex(testIKoptionsmex)
endif()
