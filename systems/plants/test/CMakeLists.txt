cmake_minimum_required (VERSION 2.6)

if (eigen3_FOUND AND Boost_FOUND)
  add_executable(urdf_kin_test urdf_kin_test.cpp)
  include_directories( .. )
  target_link_libraries(urdf_kin_test drakeRBMurdf drakeURDFinterface)
endif()

if (gurobi_FOUND)
  add_executable(testApproximateIK testApproximateIK.cpp)
  include_directories( .. )
  target_link_libraries(testApproximateIK drakeRBMurdf drakeURDFinterface drakeRigidBodyConstraint drakeIK ${MATLAB_ROOT}/bin/glnxa64/libmat.so ${MATLAB_ROOT}/bin/glnxa64/libmx.so)
endif()

if (snopt_cpp_FOUND)
  add_executable(testIK testIK.cpp)
  include_directories( .. )
  target_link_libraries(testIK drakeRBMurdf drakeURDFinterface drakeRigidBodyConstraint drakeIK ${MATLAB_ROOT}/bin/glnxa64/libmat.so ${MATLAB_ROOT}/bin/glnxa64/libmx.so)
endif()
