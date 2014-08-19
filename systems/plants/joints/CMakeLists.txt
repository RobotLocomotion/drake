include_directories ( .. )

file(GLOB drakeJoints_SRC_FILES *.cpp)
file(GLOB drakeJoints_HEADER_FILES *.h)
add_library(drakeJoints SHARED ${drakeJoints_SRC_FILES})

target_link_libraries(drakeJoints drakeGeometryUtil drakeGradientUtil)

# enable_testing()

pods_install_libraries(drakeJoints)
  
add_subdirectory(test)