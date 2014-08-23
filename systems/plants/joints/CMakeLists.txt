include_directories ( .. )

add_library(drakeJoints SHARED DrakeJoint.cpp FixedAxisOneDoFJoint.cpp HelicalJoint.cpp PrismaticJoint.cpp QuaternionFloatingJoint.cpp RevoluteJoint.cpp RollPitchYawFloatingJoint.cpp)

target_link_libraries(drakeJoints drakeGeometryUtil drakeGradientUtil)

# enable_testing()

pods_install_libraries(drakeJoints)
  
add_subdirectory(test)