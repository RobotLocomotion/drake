if(ignition-rndf0_FOUND)

	add_library_with_exports(LIB_NAME drakeMaliputRNDF SOURCE_FILES
		branch_point.cc
		junction.cc
		lane.cc
		road_geometry.cc
		segment.cc
		spline_helpers.cc
		spline_lane.cc
		../api/road_geometry.cc
	)

  	target_include_directories(drakeMaliputRNDF PRIVATE
    	"${IGNITION-RNDF_INCLUDE_DIRS}")

	target_link_libraries(drakeMaliputRNDF
		drakeCommon
		"${IGNITION-RNDF_LIBRARIES}"
	)

	drake_install_libraries(drakeMaliputRNDF)

	drake_install_headers(
		branch_point.h
		junction.h
		lane.h
		road_geometry.h
		segment.h
		spline_lane.h
		spline_helpers.h
	)

endif()


if(BUILD_TESTING)
  add_subdirectory(test)
endif()