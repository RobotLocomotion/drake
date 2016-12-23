add_library(tinyxml2 tinyxml2.cpp tinyxml2.h)
target_compile_definitions(tinyxml2 PUBLIC TINYXML2_EXPORT)

drake_install_headers(tinyxml2.h)
drake_install_libraries(tinyxml2)

drake_install_pkg_config_file(tinyxml2
  TARGET tinyxml2
  URL http://www.grinninglizard.com/tinyxml2/
  VERSION 4.0.1
  LIBS -ltinyxml2
  CFLAGS -DTINYXML2_EXPORT)
