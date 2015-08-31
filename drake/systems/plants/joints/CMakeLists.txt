include_directories ( .. )
include_directories ( ${PROJECT_SOURCE_DIR}/util )

add_library(drakeJoints SHARED DrakeJoint.cpp FixedJoint.cpp QuaternionFloatingJoint.cpp RollPitchYawFloatingJoint.cpp PrismaticJoint.cpp RevoluteJoint.cpp HelicalJoint.cpp)

target_link_libraries(drakeJoints drakeGeometryUtil)

pods_install_libraries(drakeJoints)
pods_install_headers(DrakeJoint.h DrakeJointImpl.h FixedAxisOneDoFJoint.h HelicalJoint.h PrismaticJoint.h QuaternionFloatingJoint.h RevoluteJoint.h RollPitchYawFloatingJoint.h FixedJoint.h DESTINATION drake)
  
add_subdirectory(test)
