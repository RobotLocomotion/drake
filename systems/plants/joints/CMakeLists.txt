include_directories ( .. )

set(drakeJoints_SRC_FILES DrakeJoint.cpp DrakeJointBaseImpl.cpp OneDoFJoint.cpp PrismaticJoint.cpp RevoluteJoint.cpp)

add_library(drakeJoints SHARED ${drakeJoints_SRC_FILES})

# enable_testing()

pods_install_libraries(drakeJoints)
pods_install_headers(DrakeJoint.h DrakeJointBaseImpl.h OneDoFJoint.h PrismaticJoint.h RevoluteJoint.h DESTINATION drake/joints)
  
