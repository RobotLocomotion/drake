include_directories ( .. )

set(drakeJoints_SRC_FILES DrakeJoint.cpp OneDoFJoint.cpp PrismaticJoint.cpp RevoluteJoint.cpp)

add_library(drakeJoints SHARED ${drakeJoints_SRC_FILES})

# enable_testing()

pods_install_libraries(drakeJoints)
pods_install_headers(DrakeJoint.h OneDoFJoint.h PrismaticJoint.h RevoluteJoint.h DESTINATION drake/joints)
  
