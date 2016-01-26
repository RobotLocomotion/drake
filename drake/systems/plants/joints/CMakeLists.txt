add_library_with_exports(LIB_NAME drakeJoints SOURCE_FILES DrakeJoint.cpp FixedJoint.cpp QuaternionFloatingJoint.cpp RollPitchYawFloatingJoint.cpp PrismaticJoint.cpp RevoluteJoint.cpp HelicalJoint.cpp)

target_link_libraries(drakeJoints drakeGeometryUtil)

pods_install_libraries(drakeJoints)
drake_install_headers(DrakeJoint.h DrakeJointImpl.h FixedAxisOneDoFJoint.h HelicalJoint.h PrismaticJoint.h QuaternionFloatingJoint.h RevoluteJoint.h RollPitchYawFloatingJoint.h FixedJoint.h)
  
add_subdirectory(test)
