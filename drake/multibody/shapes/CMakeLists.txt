add_library(drakeShapes geometry.cc element.cc visual_element.cc)
target_link_libraries(drakeShapes
    drakeCommon
    Eigen3::Eigen
    spruce
    tinyobjloader)

drake_install_libraries(drakeShapes)
drake_install_headers(drake_shapes.h geometry.h element.h visual_element.h)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
