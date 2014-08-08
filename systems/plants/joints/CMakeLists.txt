include_directories ( .. )

file(GLOB drakeJoints_SRC_FILES *.cpp)
file(GLOB drakeJoints_HEADER_FILES *.h)

add_library(drakeJoints SHARED ${drakeJoints_SRC_FILES})
target_link_libraries(drakeJoints drakeQuatUtil)

# enable_testing()

pods_install_libraries(drakeJoints)
pods_install_headers(${drakeJoints_HEADER_FILES} DESTINATION drake/joints)
  
