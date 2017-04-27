drake_add_cc_test(generate_urdf_test)
target_link_libraries(generate_urdf_test drakeAutomotive spruce)

if(PYTHON_EXECUTABLE)
  drake_add_test(
    NAME yaml_to_obj_test
    COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/yaml_to_obj_test.py
    $<TARGET_FILE:yaml_to_obj>)
endif()
