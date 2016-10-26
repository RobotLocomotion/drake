add_library_with_exports(LIB_NAME drakeJoints SOURCE_FILES
  DrakeJoint.cpp
  FixedJoint.cpp
  HelicalJoint.cpp
  PrismaticJoint.cpp
  QuaternionFloatingJoint.cpp
  RevoluteJoint.cpp
  RollPitchYawFloatingJoint.cpp)

target_link_libraries(drakeJoints drakeGeometryUtil)

pods_install_libraries(drakeJoints)
drake_install_headers(
  DrakeJoint.h
  DrakeJointImpl.h
  FixedAxisOneDoFJoint.h
  FixedJoint.h
  HelicalJoint.h
  PrismaticJoint.h
  QuaternionFloatingJoint.h
  RevoluteJoint.h
  RollPitchYawFloatingJoint.h
  floating_base_types.h)
pods_install_pkg_config_file(drake-joints
  LIBS -ldrakeJoints
  REQUIRES
  VERSION 0.0.1)
add_subdirectory(test)
