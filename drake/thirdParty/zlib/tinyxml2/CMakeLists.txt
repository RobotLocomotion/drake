add_library(tinyxml2 SHARED tinyxml2.cpp)
set_target_properties(tinyxml2 PROPERTIES COMPILE_DEFINITIONS "TINYXML2_EXPORT")
drake_install_libraries(tinyxml2)
drake_install_headers(tinyxml2.h)
