include_directories ( .. )
include_directories (../constraint)

set(drakeRigidBodyConstraint_SRC_FILES RigidBodyConstraint.cpp)

macro(add_constraint_mex)
  add_mex(${ARGV} ${ARGV}.cpp)
  target_link_libraries(${ARGV} drakeRBM drakeUtil drakeRigidBodyConstraint drakeConstructRigidBodyConstraint)
endmacro()


pods_find_pkg_config(eigen3)
if(eigen3_FOUND)
  add_mex(drakeRigidBodyConstraint SHARED ${drakeRigidBodyConstraint_SRC_FILES})
  target_link_libraries(drakeRigidBodyConstraint drakeRBM drakeQuatUtil)
  set_target_properties(drakeRigidBodyConstraint PROPERTIES COMPILE_FLAGS -fPIC )
  pods_install_libraries(drakeRigidBodyConstraint)
  pods_install_headers(RigidBodyConstraint.h DESTINATION drake)

  add_mex(drakeConstructRigidBodyConstraint SHARED constructPtrRigidBodyConstraint.cpp)
  target_link_libraries(drakeConstructRigidBodyConstraint drakeRigidBodyConstraint drakeRBM drakeUtil drakeQuatUtil)
  add_constraint_mex(deleteRigidBodyConstraintmex)
  add_constraint_mex(testSingleTimeKinCnstmex)
  add_constraint_mex(testMultipleTimeKinCnstmex)
  add_constraint_mex(testQuasiStaticConstraintmex)
  add_constraint_mex(testPostureConstraintmex)
  add_constraint_mex(testMultipleTimeLinearPostureConstraintmex)
  add_constraint_mex(testSingleTimeLinearPostureConstraintmex)
  add_constraint_mex(constraintTypemex)
  add_constraint_mex(constraintCategorymex)
  add_constraint_mex(constructPtrRigidBodyConstraintmex)
  add_constraint_mex(updatePtrRigidBodyConstraintmex)
  # temporarily comment out these lines since there is linking problem
  if (Boost_FOUND)
      add_executable(testConstraintConstructor test/testConstraintConstructor.cpp)
      set_target_properties(testConstraintConstructor PROPERTIES
	COMPILE_FLAGS "-fPIC") 
      target_link_libraries(testConstraintConstructor drakeRBM drakeRigidBodyConstraint drakeRBMurdf drakeURDFinterface)
  endif()
          
endif()

