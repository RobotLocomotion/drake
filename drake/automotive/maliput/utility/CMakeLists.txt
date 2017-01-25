if(BUILD_TESTING)
  add_subdirectory(test)
endif()

drake_install_headers(
  generate_obj.h
  generate_urdf.h
  )

add_executable(yaml_to_obj yaml_to_obj.cc)
target_link_libraries(yaml_to_obj drakeAutomotive yaml-cpp)
