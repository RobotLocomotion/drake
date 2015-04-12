cmake_minimum_required (VERSION 2.6)

pods_find_pkg_config(bullet)

if (eigen3_FOUND)
  add_rbm_mex(compareParsersmex)
  
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
  
  if (NOT MSVC10)
    # this test does not compile on MSVC 2010 because of the use of variadic templates
    add_executable(testKinematicsCacheChecks testKinematicsCacheChecks.cpp)
    target_link_libraries(testKinematicsCacheChecks drakeRBM)
    add_test(NAME testKinematicsCacheChecks WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" COMMAND testKinematicsCacheChecks)
  endif()
endif()

macro(add_ik_cpp)
  add_executable(${ARGV} ${ARGV}.cpp)
  include_directories( .. )
  set_target_properties(${ARGV} PROPERTIES COMPILE_FLAGS "-fPIC")
  target_link_libraries(${ARGV} drakeRigidBodyConstraint drakeIK)
  add_test( NAME ${ARGV} WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" COMMAND ${ARGV})
endmacro()

if (gurobi_FOUND AND Boost_FOUND)
  add_ik_cpp(testApproximateIK)
endif()

if (snopt_cpp_FOUND AND Boost_FOUND)
  add_ik_cpp(testIK)
  add_ik_cpp(testIKpointwise)
  add_ik_cpp(testIKtraj)
endif()

if (Boost_FOUND)
  add_executable(debugManipulatorDynamics debugManipulatorDynamics.cpp)
  include_directories( .. )
  target_link_libraries(debugManipulatorDynamics drakeRBM)
endif()
