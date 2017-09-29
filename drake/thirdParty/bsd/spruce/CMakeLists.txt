add_library(spruce spruce.hh spruce.cc)
target_include_directories(spruce
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

drake_install_headers(spruce.hh)
drake_install_libraries(spruce)
