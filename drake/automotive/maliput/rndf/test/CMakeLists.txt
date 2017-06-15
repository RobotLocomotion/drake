if(ignition-rndf0_FOUND)
    drake_add_cc_test(ignition_rndf_test)
    target_include_directories(ignition_rndf_test PRIVATE
        "${IGNITION-RNDF_INCLUDE_DIRS}"
    )
    target_link_libraries(ignition_rndf_test "${IGNITION-RNDF_LIBRARIES}")

    drake_add_cc_test(junction_test)
    target_include_directories(junction_test PRIVATE
        "${IGNITION-RNDF_INCLUDE_DIRS}"
    )
    target_link_libraries(junction_test
        drakeMaliputRndf
        "${IGNITION-RNDF_LIBRARIES}")

    drake_add_cc_test(spline_helpers_test)
    target_include_directories(spline_helpers_test PRIVATE
        "${IGNITION-RNDF_INCLUDE_DIRS}"
    )
    target_link_libraries(spline_helpers_test
        drakeMaliputRndf
        "${IGNITION-RNDF_LIBRARIES}")

    drake_add_cc_test(road_geometry_test)
    target_include_directories(road_geometry_test PRIVATE
        "${IGNITION-RNDF_INCLUDE_DIRS}"
    )
    target_link_libraries(road_geometry_test
        drakeMaliputRndf
        "${IGNITION-RNDF_LIBRARIES}")

    drake_add_cc_test(segment_test)
    target_include_directories(segment_test PRIVATE
        "${IGNITION-RNDF_INCLUDE_DIRS}"
    )
    target_link_libraries(segment_test
        drakeMaliputRndf
        "${IGNITION-RNDF_LIBRARIES}")
endif()
