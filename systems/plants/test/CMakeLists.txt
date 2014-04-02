cmake_minimum_required (VERSION 2.6)

if (eigen3_FOUND AND Boost_FOUND)
  add_executable(urdf_kin_test urdf_kin_test.cpp)
  include_directories( .. )
  target_link_libraries(urdf_kin_test drakeRBMurdf drakeURDFinterface)
endif()

macro(add_ik_cpp)
  add_mex(${ARGV} EXECUTABLE ${ARGV}.cpp)
  include_directories( .. )
  target_link_libraries(${ARGV} drakeRBMurdf drakeURDFinterface drakeRigidBodyConstraint drakeIK)
endmacro()

if (gurobi_FOUND)
  add_ik_cpp(testApproximateIK)
endif()

if (snopt_cpp_FOUND AND Boost_FOUND)
  add_ik_cpp(testIK)
  add_ik_cpp(testIKpointwise)
  add_ik_cpp(testIKtraj)
endif()
