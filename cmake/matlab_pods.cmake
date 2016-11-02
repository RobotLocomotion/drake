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
    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_r_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_r_path()\n"
		"path = fullfile(pods_get_base_path, 'lib');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_lib_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_lib_path()\n"
		"path = fullfile(pods_get_base_path, 'lib');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_bin_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_bin_path()\n"
		"path = fullfile(pods_get_base_path, 'bin');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_include_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_include_path()\n"
		"path = fullfile(pods_get_base_path, 'include');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_pkgconfig_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_pkgconfig_path()\n"
		"path = fullfile(pods_get_base_path, 'lib', 'pkgconfig');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_base_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_base_path()\n"
                "  path = fullfile(fileparts(which(mfilename)),'..');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_data_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_data_path()\n"
		"path = fullfile(pods_get_base_path(),'data');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_config_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_config_path()\n"
		"path = fullfile(pods_get_base_path(),'config');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)

    set(PODS_PATHFILE ${PROJECT_BINARY_DIR}/matlab/pods_get_models_path.m)
    file(WRITE ${PODS_PATHFILE}
		"function path = pods_get_models_path()\n"
		"path = fullfile(pods_get_base_path(),'models');\n"
    )
    install(FILES ${PODS_PATHFILE} DESTINATION matlab/)
endfunction()
