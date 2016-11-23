add_library_with_exports(LIB_NAME drakeShapes SOURCE_FILES geometry.cc element.cc visual_element.cc)
target_link_libraries(drakeShapes spruce)

# if(BUILD_TESTING)
#   add_subdirectory(test)
# endif()

drake_install_libraries(drakeShapes)
drake_install_headers(drake_shapes.h geometry.h element.h visual_element.h)
drake_install_pkg_config_file(drake-shapes
  TARGET drakeShapes
  LIBS
    -ldrakeShapes
    -lspruce
  REQUIRES eigen3)
