include_directories ( .. )

add_library(drakeJoints SHARED DrakeJoint.cpp FixedAxisOneDoFJoint.cpp HelicalJoint.cpp PrismaticJoint.cpp QuaternionFloatingJoint.cpp RevoluteJoint.cpp RollPitchYawFloatingJoint.cpp drakeJointUtil.cpp)

target_link_libraries(drakeJoints drakeGeometryUtil drakeGradientUtil)

# enable_testing()

pods_install_libraries(drakeJoints)
pods_install_headers(DrakeJoint.h FixedAxisOneDoFJoint.h HelicalJoint.h PrismaticJoint.h QuaternionFloatingJoint.h RevoluteJoint.h RollPitchYawFloatingJoint.h drakeJointUtil.h DESTINATION drake/joints)
  
add_subdirectory(test)