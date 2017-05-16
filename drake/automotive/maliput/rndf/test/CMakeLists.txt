if(ignition-rndf0_FOUND)
  drake_add_cc_test(ignition_rndf_test)
  target_include_directories(ignition_rndf_test PRIVATE
    "${IGNITION-RNDF_INCLUDE_DIRS}")
  target_link_libraries(ignition_rndf_test "${IGNITION-RNDF_LIBRARIES}")
endif()
