add_library_with_exports(LIB_NAME drakeShapes SOURCE_FILES geometry.cc element.cc visual_element.cc)
target_link_libraries(drakeShapes spruce)

# if(BUILD_TESTING)
#   add_subdirectory(test)
# endif()

pods_install_libraries(drakeShapes)
drake_install_headers(drake_shapes.h geometry.h element.h visual_element.h)
pods_install_pkg_config_file(drake-shapes
  LIBS -ldrakeShapes
  REQUIRES
  VERSION 0.0.1)
