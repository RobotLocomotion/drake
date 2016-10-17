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

add_subdirectory(test)
