add_library_with_exports(LIB_NAME drakeXMLUtil SOURCE_FILES xmlUtil.cpp)
target_link_libraries(drakeXMLUtil drakeCommon tinyxml2 spruce)
pods_install_libraries(drakeXMLUtil)
drake_install_headers(xmlUtil.h)
pods_install_pkg_config_file(drake-xml-util
  LIBS -ldrakeXMLUtil -ldrakeCommmon -ltinyxml2 -lspruce
  REQUIRES
  VERSION 0.0.1)

add_library_with_exports(LIB_NAME drakeRBM SOURCE_FILES
  parser_model_instance_id_table.cc
  parser_common.cc
  parser_sdf.cc
  parser_urdf.cc
  RigidBodyTree.cpp
  RigidBody.cpp
  RigidBodyTreeContact.cpp
  rigid_body_actuator.cc
  rigid_body_frame.cc
  rigid_body_loop.cc
  rigid_body_tree_construction.cc
  $<TARGET_OBJECTS:drakeUtil>)
target_link_libraries(drakeRBM
  drakeCollision drakeCommon drakeJoints drakeXMLUtil)
pods_install_libraries(drakeRBM)
drake_install_headers(
  constraint_wrappers.h
  ForceTorqueMeasurement.h
  KinematicPath.h
  KinematicsCache.h
  material_map.h
  pose_map.h
  rigid_body_actuator.h
  RigidBodyFrame.h
  RigidBody.h
  RigidBodyTree.h
  parser_model_instance_id_table.h
  parser_common.h
  parser_sdf.h
  parser_urdf.h
  rigid_body_loop.h
  rigid_body_tree_construction.h)

pods_install_pkg_config_file(drake-rbm
  LIBS -ldrakeRBM -ldrakeCollision -ldrakeJoints -ldrakeXMLUtil
  REQUIRES
  VERSION 0.0.1)

add_library_with_exports(LIB_NAME drakeIKoptions SOURCE_FILES IKoptions.cpp)
target_link_libraries(drakeIKoptions drakeRBM)

# Note that drakeIK is published via a SWIG MATLAB wrapper, but we haven't
# moved it into the "C++ and MATLAB shared code" lower section of this file,
# because the C++ IK will remain in this directory.  Only MATLAB code that will
# be evicted from this directory is moved to the lower section of this file.
set(drakeIK_SRC_FILES constraint_wrappers.cc ik_trajectory_helper.cc
  inverseKin.cpp inverseKinBackend.cpp inverseKinPointwise.cpp
  inverseKinTraj.cpp inverseKinTrajBackend.cpp)
if (gurobi_FOUND)
  set(drakeIK_SRC_FILES ${drakeIK_SRC_FILES} approximateIK.cpp)
endif ()
add_library_with_exports(LIB_NAME drakeIK SOURCE_FILES ${drakeIK_SRC_FILES})
target_link_libraries(drakeIK drakeRBM drakeOptimization drakeRigidBodyConstraint drakeIKoptions)
pods_install_libraries(drakeIK)
pods_install_libraries(drakeIKoptions)
drake_install_headers(constraint_wrappers.h ik_trajectory_helper.h IKoptions.h
  inverseKinBackend.h RigidBodyIK.h)
pods_install_pkg_config_file(drake-ik
  LIBS -ldrakeIK -ldrakeRigidBodyConstraint -ldrakeIKoptions
  REQUIRES drake-rbm
  VERSION 0.0.1)

add_subdirectory(collision)
add_subdirectory(constraint)
add_subdirectory(joints)
add_subdirectory(rigid_body_plant)
add_subdirectory(rigid_body_system1)
add_subdirectory(shapes)
add_subdirectory(test)
