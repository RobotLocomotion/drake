add_subdirectory(test)

drake_install_headers(
  Function.h
  functional_form.h
  Gradient.h
  Vector.h)

add_library_with_exports(LIB_NAME drakeCore SOURCE_FILES functional_form.cc)

pods_install_libraries(drakeCore)
pods_install_pkg_config_file(drake-core
  LIBS -ldrakeCore
  REQUIRES
  VERSION 0.0.1)
