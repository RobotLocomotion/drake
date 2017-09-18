add_library(tinyxml2 tinyxml2.cpp tinyxml2.h)
target_compile_definitions(tinyxml2 PUBLIC TINYXML2_EXPORT)

drake_install_headers(tinyxml2.h)
drake_install_libraries(tinyxml2)
