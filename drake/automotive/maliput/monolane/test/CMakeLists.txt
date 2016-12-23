drake_add_cc_test(monolane_lanes_test)
target_link_libraries(monolane_lanes_test drakeAutomotive)

drake_add_cc_test(monolane_builder_test)
target_link_libraries(monolane_builder_test drakeAutomotive)

add_executable(yaml_load yaml_load.cc)
target_link_libraries(yaml_load drakeAutomotive yaml-cpp)
if(PYTHON_EXECUTABLE)
  drake_add_test(
    NAME yaml_load_test
    COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/yaml_load_test.py
    ${CMAKE_CURRENT_BINARY_DIR}/yaml_load)
endif()
