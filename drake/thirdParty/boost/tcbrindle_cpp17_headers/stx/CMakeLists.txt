add_library(stx optional.hpp)
set_target_properties(stx PROPERTIES LINKER_LANGUAGE CXX)
drake_install_headers(optional.hpp)
drake_install_libraries(stx)
