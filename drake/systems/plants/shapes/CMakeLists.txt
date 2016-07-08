
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # TODO(#2372) These are warnings that we can't handle yet.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")
endif()

add_library_with_exports(LIB_NAME drakeShapes SOURCE_FILES Geometry.cpp Element.cpp VisualElement.cpp)
target_link_libraries(drakeShapes spruce)

#enable_testing()
#add_subdirectory(test)

pods_install_libraries(drakeShapes)
drake_install_headers(DrakeShapes.h Geometry.h Element.h VisualElement.h)
pods_install_pkg_config_file(drake-shapes
  LIBS -ldrakeShapes
  REQUIRES
  VERSION 0.0.1)
