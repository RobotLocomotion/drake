#------------------------------------------------------------------------------
# pods_configure_matlab_paths()
#
# provides access to pods and cmake paths inside of matlab
#
# after calling, functions are available in matlab:
# >> pods_get_base_path()
#
# only needs to be called once per project
#------------------------------------------------------------------------------
function(pods_configure_matlab_paths)
    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_lib_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_lib_path()\n"
		"  path = fullfile(pods_get_base_path, 'lib');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_bin_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_bin_path()\n"
		"  path = fullfile(pods_get_base_path, 'bin');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_base_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_base_path()\n"
        "  path = fullfile(fileparts(which(mfilename)),'..');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)
endfunction()
