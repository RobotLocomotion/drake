if(ignition-rndf0_FOUND)

    add_library_with_exports(LIB_NAME drakeMaliputRndf SOURCE_FILES
        branch_point.cc
        junction.cc
        lane.cc
        road_geometry.cc
        segment.cc
        spline_helpers.cc
        spline_lane.cc
    )

    target_include_directories(drakeMaliputRndf PRIVATE
        "${IGNITION-RNDF_INCLUDE_DIRS}"
    )

    target_link_libraries(drakeMaliputRndf
        drakeCommon
        drakeAutomotive
        "${IGNITION-RNDF_LIBRARIES}"
    )

    drake_install_libraries(drakeMaliputRndf)

    drake_install_headers(
        branch_point.h
        junction.h
        lane.h
        road_geometry.h
        segment.h
        spline_helpers.h
        spline_lane.h
    )

endif()


if(BUILD_TESTING)
  add_subdirectory(test)
endif()
