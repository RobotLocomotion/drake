# Functions to simplify matlab use in a pods setting.
#
# To enable the macros, add the following lines to CMakeLists.txt:
#   set(POD_NAME <pod-name>)
#   include(cmake/pods.cmake)
#   include(cmake/matlab_pods.cmake)
#
# Assumes that <project_path>/build/matlab is on the matlab path
# Provides the following functions, see individual functions for documentation:
#
#   pods_install_matlab_path(...)
#   pods_configure_matlab_paths(...)
#
# ----


# pods_install_matlab_path(<module_name>
#						   [directory])
#
# Create addpath_<module_name> function to access functions in directory
#
# If directory argument is not present it is assumed to be matlab
# directory may also be set to CURRENT_DIRECTORY

function(pods_install_matlab_path)
    list(GET ARGV 0 MODULE_NAME)

    if(ARGC LESS 2)
    	set(SUBDIR_NAME matlab)
    else()
    	list(GET ARGV 1 SUBDIR_NAME)
    	if(SUBDIR_NAME STREQUAL CURRENT_DIRECTORY)
    		set(SUBDIR_NAME "")
    	endif()
    endif()

    set(ADDPATH_FILE ${PROJECT_BINARY_DIR}/matlab/addpath_${MODULE_NAME}.m)

    file(WRITE ${ADDPATH_FILE}
		"function addpath_${MODULE_NAME}(recurse)\n"
		"global ${MODULE_NAME}_path_added\n"
		"if ${MODULE_NAME}_path_added\n"
		"	return;\n"
    	"end\n"
		"${MODULE_NAME}_path_added = 1;\n"
		"path = '${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR_NAME}';\n"
		"if nargin==0\n"
		"    recurse = 1;\n"
		"end\n"
		"if recurse\n"
		"    all_paths = genpath(path);\n"
		"    split_paths = regexp(all_paths, ':','split');\n"
		"    for ii=1:length(split_paths)\n"
		"        ind = strfind(split_paths{ii},'.svn');\n"
		"        if isempty(ind)\n"
		"            addpath(split_paths{ii})\n"
		"        end\n"
		"    end\n"
		"else\n"
		"    rmpath(path)\n"
		"end\n"
    )

	set(RMPATH_FILE ${PROJECT_BINARY_DIR}/matlab/rmpath_${MODULE_NAME}.m)

    file(WRITE ${RMPATH_FILE}
		"function rmpath_${MODULE_NAME}(recurse)\n"
		"global ${MODULE_NAME}_path_added\n"
		"if ~${MODULE_NAME}_path_added\n"
		"	return\n"
		"end\n"
		"${MODULE_NAME}_path_added = 0;\n"
		"path = '${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR_NAME}';\n"
		"if nargin==0\n"
		"    recurse = 1;\n"
		"end\n"
		"if recurse\n"
		"    all_paths = genpath(path);\n"
		"    split_paths = regexp(all_paths, ':','split');\n"
		"    for ii=1:length(split_paths)\n"
		"        ind = strfind(split_paths{ii},'.svn');\n"
		"        if isempty(ind)\n"
		"            rmpath(split_paths{ii})\n"
		"        end\n"
		"    end\n"
		"else\n"
		"    rmpath(path)\n"
		"end\n"
    )


install(FILES ${ADDPATH_FILE} DESTINATION matlab/)
install(FILES ${RMPATH_FILE} DESTINATION matlab/)


endfunction(pods_install_matlab_path)


# pods_configure_matlab_paths()
#
# provides access to pods and cmake paths inside of matlab
#
# after calling, functions are available in matlab:
# >> pods_get_base_path()
#
# only needs to be called once per project

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

endfunction(pods_configure_matlab_paths)

