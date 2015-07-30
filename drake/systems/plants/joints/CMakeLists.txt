include_directories ( .. )
include_directories ( ${PROJECT_SOURCE_DIR}/util )

#add_library(drakeJoints SHARED DrakeJoint.cpp DrakeJointImpl.cpp QuaternionFloatingJoint.cpp FixedAxisOneDoFJoint.cpp HelicalJoint.cpp PrismaticJoint.cpp RevoluteJoint.cpp RollPitchYawFloatingJoint.cpp drakeJointUtil.cpp)
add_library(drakeJoints SHARED DrakeJoint.cpp FixedJoint.cpp QuaternionFloatingJoint.cpp RollPitchYawFloatingJoint.cpp)
#add_library(drakeJoints SHARED DrakeJoint.cpp DrakeJointImpl.cpp)

target_link_libraries(drakeJoints drakeGeometryUtil)

# enable_testing()
pods_install_libraries(drakeJoints)
pods_install_headers(DrakeJoint.h DrakeJointImpl.h FixedAxisOneDoFJoint.h HelicalJoint.h PrismaticJoint.h QuaternionFloatingJoint.h RevoluteJoint.h RollPitchYawFloatingJoint.h FixedJoint.h drakeJointUtil.h DESTINATION drake)
  
add_subdirectory(test)
