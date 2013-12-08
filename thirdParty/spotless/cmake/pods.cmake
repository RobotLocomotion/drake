# Macros to simplify compliance with the pods build policies.
#
# To enable the macros, add the following lines to CMakeLists.txt:
#   set(POD_NAME <pod-name>)
#   include(cmake/pods.cmake)
#
# If POD_NAME is not set, then the CMake source directory is used as POD_NAME
#
# Next, any of the following macros can be used.  See the individual macro
# definitions in this file for individual documentation.
#
# C/C++
#   pods_install_headers(...)
#   pods_install_libraries(...)
#   pods_install_executables(...)
#   pods_install_pkg_config_file(...)
#
#   pods_use_pkg_config_packages(...)
#
# Python
#   pods_install_python_packages(...)
#   pods_install_python_script(...)
#
# Java
#   None yet
#
# ----
# File: pods.cmake
# Distributed with pods version: 12.11.14

# pods_install_headers(<header1.h> ... DESTINATION <subdir_name>)
# 
# Install a (list) of header files.
#
# Header files will all be installed to include/<subdir_name>
#
# example:
#   add_library(perception detector.h sensor.h)
#   pods_install_headers(detector.h sensor.h DESTINATION perception)
#
function(pods_install_headers)
    list(GET ARGV -2 checkword)
    if(NOT checkword STREQUAL DESTINATION)
        message(FATAL_ERROR "pods_install_headers missing DESTINATION parameter")
    endif()

    list(GET ARGV -1 dest_dir)
    list(REMOVE_AT ARGV -1)
    list(REMOVE_AT ARGV -1)
    #copy the headers to the INCLUDE_OUTPUT_PATH (${CMAKE_BINARY_DIR}/include)
    foreach(header ${ARGV})
        get_filename_component(_header_name ${header} NAME)
        configure_file(${header} ${INCLUDE_OUTPUT_PATH}/${dest_dir}/${_header_name} COPYONLY)
	endforeach(header)
	#mark them to be installed
	install(FILES ${ARGV} DESTINATION include/${dest_dir})


endfunction(pods_install_headers)

# pods_install_executables(<executable1> ...)
#
# Install a (list) of executables to bin/
function(pods_install_executables)
    install(TARGETS ${ARGV} RUNTIME DESTINATION bin)
endfunction(pods_install_executables)

# pods_install_libraries(<library1> ...)
#
# Install a (list) of libraries to lib/
function(pods_install_libraries)
    install(TARGETS ${ARGV} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
endfunction(pods_install_libraries)


# pods_install_pkg_config_file(<package-name> 
#                              [VERSION <version>]
#                              [DESCRIPTION <description>]
#                              [CFLAGS <cflag> ...]
#                              [LIBS <lflag> ...]
#                              [REQUIRES <required-package-name> ...])
# 
# Create and install a pkg-config .pc file.
#
# example:
#    add_library(mylib mylib.c)
#    pods_install_pkg_config_file(mylib LIBS -lmylib REQUIRES glib-2.0)
function(pods_install_pkg_config_file)
    list(GET ARGV 0 pc_name)
    # TODO error check
    
    set(pc_version 0.0.1)
    set(pc_description ${pc_name})
    set(pc_requires "")
    set(pc_libs "")
    set(pc_cflags "")
    set(pc_fname "${PKG_CONFIG_OUTPUT_PATH}/${pc_name}.pc")
    
    set(modewords LIBS CFLAGS REQUIRES VERSION DESCRIPTION)
    set(curmode "")

    # parse function arguments and populate pkg-config parameters
    list(REMOVE_AT ARGV 0)
    foreach(word ${ARGV})
        list(FIND modewords ${word} mode_index)
        if(${mode_index} GREATER -1)
            set(curmode ${word})
        elseif(curmode STREQUAL LIBS)
            set(pc_libs "${pc_libs} ${word}")
        elseif(curmode STREQUAL CFLAGS)
            set(pc_cflags "${pc_cflags} ${word}")
        elseif(curmode STREQUAL REQUIRES)
            set(pc_requires "${pc_requires} ${word}")
        elseif(curmode STREQUAL VERSION)
            set(pc_version ${word})
            set(curmode "")
        elseif(curmode STREQUAL DESCRIPTION)
            set(pc_description "${word}")
            set(curmode "")
        else(${mode_index} GREATER -1)
            message("WARNING incorrect use of pods_add_pkg_config (${word})")
            break()
        endif(${mode_index} GREATER -1)
    endforeach(word)

    # write the .pc file out
    file(WRITE ${pc_fname}
        "prefix=${CMAKE_INSTALL_PREFIX}\n"
        "exec_prefix=\${prefix}\n"
        "libdir=\${exec_prefix}/lib\n"
        "includedir=\${prefix}/include\n"
        "\n"
        "Name: ${pc_name}\n"
        "Description: ${pc_description}\n"
        "Requires: ${pc_requires}\n"
        "Version: ${pc_version}\n"
        "Libs: -L\${libdir} ${pc_libs}\n"
        "Cflags: -I\${includedir} ${pc_cflags}\n")

    # mark the .pc file for installation to the lib/pkgconfig directory
    install(FILES ${pc_fname} DESTINATION lib/pkgconfig)
    
endfunction(pods_install_pkg_config_file)


# pods_install_python_script(<script_name> <python_module_or_file>)
#
# Create and install a script that invokes the python interpreter with a
# specified python module or script.
#
# A launcher script will be installed to bin/<script_name>. The script simply
# adds <install-prefix>/lib/pythonX.Y/dist-packages
# and  <install-prefix>/lib/pythonX.Y/site-packages 
# to the PYTHONPATH, and then 
# invokes `python -m <python_module>` or `python python_file` 
# depending on whether the function was passed a module name or script file.
#
# example:
#    pods_install_python_script(run-py-module py_pkg.py_module)
#    pods_install_python_script(run-py-script py_script.py)
function(pods_install_python_script script_name python_module_or_file)
    find_package(PythonInterp REQUIRED)

    # which python version?
    execute_process(COMMAND 
        ${PYTHON_EXECUTABLE} -c "import sys; sys.stdout.write(sys.version[:3])"
        OUTPUT_VARIABLE pyversion)

    # where do we install .py files to?
    set(python_install_dir 
        ${CMAKE_INSTALL_PREFIX}/lib/python${pyversion}/dist-packages)
    set(python_old_install_dir #todo: when do we get rid of this? 
        ${CMAKE_INSTALL_PREFIX}/lib/python${pyversion}/site-packages)
        
    if (python_module_or_file MATCHES ".+\\.py") #ends with a .py
        get_filename_component(py_file ${python_module_or_file} ABSOLUTE)     
            
        if (NOT EXISTS ${py_file})
            message(FATAL_ERROR "${python_module_or_file} is not an absolute or relative path to a python script")
        endif()
        
        #get the directory where we'll install the script ${sanitized_POD_NAME}_scripts
        string(REGEX REPLACE "[^a-zA-Z0-9]" "_" __sanitized_pod_name "${POD_NAME}")
        set(pods_scripts_dir "${python_install_dir}/${__sanitized_pod_name}_scripts")
                
        # install the python script file
        install(FILES ${py_file}  DESTINATION "${pods_scripts_dir}")

        get_filename_component(py_script_name ${py_file} NAME)
        # write the bash script file
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${script_name} 
            "#!/bin/sh\n"
            "export PYTHONPATH=${python_install_dir}:${python_old_install_dir}:\${PYTHONPATH}\n"
            "exec ${PYTHON_EXECUTABLE} ${pods_scripts_dir}/${py_script_name} $*\n")    
    else()
        get_filename_component(py_module ${python_module_or_file} NAME) #todo: check whether module exists?
        # write the bash script file
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${script_name} 
            "#!/bin/sh\n"
            "export PYTHONPATH=${python_install_dir}:${python_old_install_dir}:\${PYTHONPATH}\n"
            "exec ${PYTHON_EXECUTABLE} -m ${py_module} $*\n")
    endif()
    # install it...
    install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${script_name} DESTINATION bin)
endfunction()

# _pods_install_python_package(<py_src_dir> <py_module_name>)
#
# Internal helper function
# Install python module in <py_src_dir> to lib/pythonX.Y/dist-packages/<py_module_name>,
# where X.Y refers to the current python version (e.g., 2.6)
#
function(_pods_install_python_package py_src_dir py_module_name)
    find_package(PythonInterp REQUIRED)
    # which python version?
    execute_process(COMMAND 
        ${PYTHON_EXECUTABLE} -c "import sys; sys.stdout.write(sys.version[:3])"
        OUTPUT_VARIABLE pyversion)

    # where do we install .py files to?
    set(python_install_dir 
        ${CMAKE_INSTALL_PREFIX}/lib/python${pyversion}/dist-packages)

    if(EXISTS "${py_src_dir}/__init__.py")
        #install the single module
        file(GLOB_RECURSE module_files   ${py_src_dir}/*)
        foreach(file ${module_files})
            if(NOT file MATCHES ".*\\.svn.*|.*\\.pyc|.*[~#]")
                file(RELATIVE_PATH __tmp_path ${py_src_dir} ${file})
                get_filename_component(__tmp_dir ${__tmp_path} PATH)
                install(FILES ${file}
                    DESTINATION "${python_install_dir}/${py_module_name}/${__tmp_dir}")
            endif()
        endforeach()
    else()
        message(FATAL_ERROR "${py_src_dir} is not a python package!\n")
    endif()
endfunction()


# pods_install_python_packages(<src_dir1> ...)
#
# Install python packages to lib/pythonX.Y/dist-packages, where X.Y refers to
# the current python version (e.g., 2.6)
#
# For each <src_dir> pass in, it will do the following:
# If <src_dir> is a python package (it has a __init__.py file) it will be installed 
# along with any .py files in subdirectories
#
# Otherwise the script searches for and installs any python packages in <src_dir>
function(pods_install_python_packages py_src_dir)
    get_filename_component(py_src_abs_dir ${py_src_dir} ABSOLUTE)
    if(ARGC GREATER 1)
        #install each module seperately 
        foreach(py_module ${ARGV}) 
            pods_install_python_packages(${py_module})
        endforeach()
    elseif(EXISTS "${py_src_abs_dir}/__init__.py")
        #install the single module by name
        get_filename_component(py_module_name ${py_src_abs_dir} NAME) 
        _pods_install_python_package(${py_src_abs_dir} ${py_module_name})            
    else()
        # install any packages within the passed in py_src_dir 
        set(_installed_a_package FALSE)
        file(GLOB sub-dirs RELATIVE ${py_src_abs_dir} ${py_src_abs_dir}/*)
        foreach(sub-dir ${sub-dirs})
            if(EXISTS "${py_src_abs_dir}/${sub-dir}/__init__.py")
                _pods_install_python_package(${py_src_abs_dir}/${sub-dir} ${sub-dir})
                set(_installed_a_package TRUE)
            endif()
        endforeach()
        if (NOT _installed_a_package)
            message(FATAL_ERROR "${py_src_dir} does not contain any python packages!\n")
        endif()
    endif()
endfunction()


# pods_use_pkg_config_packages(<target> <package-name> ...)
#
# Convenience macro to get compiler and linker flags from pkg-config and apply them
# to the specified target.
#
# Invokes `pkg-config --cflags-only-I <package-name> ...` and adds the result to the
# include directories.
#
# Additionally, invokes `pkg-config --libs <package-name> ...` and adds the result to
# the target's link flags (via target_link_libraries)
#
# example:
#   add_executable(myprogram main.c)
#   pods_use_pkg_config_packages(myprogram glib-2.0 opencv)
macro(pods_use_pkg_config_packages target)
    if(${ARGC} LESS 2)
        message(WARNING "Useless invocation of pods_use_pkg_config_packages")
        return()
    endif()
    find_package(PkgConfig REQUIRED)
    execute_process(COMMAND 
        ${PKG_CONFIG_EXECUTABLE} --cflags-only-I ${ARGN}
        OUTPUT_VARIABLE _pods_pkg_include_flags)
    string(STRIP ${_pods_pkg_include_flags} _pods_pkg_include_flags)
    string(REPLACE "-I" "" _pods_pkg_include_flags "${_pods_pkg_include_flags}")
	separate_arguments(_pods_pkg_include_flags)
    #    message("include: ${_pods_pkg_include_flags}")
    execute_process(COMMAND 
        ${PKG_CONFIG_EXECUTABLE} --libs ${ARGN}
        OUTPUT_VARIABLE _pods_pkg_ldflags)
    string(STRIP ${_pods_pkg_ldflags} _pods_pkg_ldflags)
    #    message("ldflags: ${_pods_pkg_ldflags}")
    include_directories(${_pods_pkg_include_flags})
    target_link_libraries(${target} ${_pods_pkg_ldflags})
    
    # make the target depend on libraries that are cmake targets
    if (_pods_pkg_ldflags)
        string(REPLACE " " ";" _split_ldflags ${_pods_pkg_ldflags})
        foreach(__ldflag ${_split_ldflags})
                string(REGEX REPLACE "^-l" "" __depend_target_name ${__ldflag})
                get_target_property(IS_TARGET ${__depend_target_name} LOCATION)
                if (NOT IS_TARGET STREQUAL "IS_TARGET-NOTFOUND")
                    #message("---- ${target} depends on  ${libname}")
                    add_dependencies(${target} ${__depend_target_name})
                endif() 
        endforeach()
    endif()

    unset(_split_ldflags)
    unset(_pods_pkg_include_flags)
    unset(_pods_pkg_ldflags)
endmacro()


# pods_config_search_paths()
#
# Setup include, linker, and pkg-config paths according to the pods core
# policy.  This macro is automatically invoked, there is no need to do so
# manually.
macro(pods_config_search_paths)
    if(NOT DEFINED __pods_setup)
		#set where files should be output locally
	    set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
	    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
	    set(INCLUDE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/include)
	    set(PKG_CONFIG_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/pkgconfig)
		
		#set where files should be installed to
	    set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
	    set(EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
	    set(INCLUDE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)
	    set(PKG_CONFIG_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)


        # add build/lib/pkgconfig to the pkg-config search path
        set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_INSTALL_PATH}:$ENV{PKG_CONFIG_PATH})
        set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_OUTPUT_PATH}:$ENV{PKG_CONFIG_PATH})

        # add build/include to the compiler include path
        include_directories(BEFORE ${INCLUDE_OUTPUT_PATH})
        include_directories(${INCLUDE_INSTALL_PATH})

        # add build/lib to the link path
        link_directories(${LIBRARY_OUTPUT_PATH})
        link_directories(${LIBRARY_INSTALL_PATH})
        

        # abuse RPATH
        if(${CMAKE_INSTALL_RPATH})
            set(CMAKE_INSTALL_RPATH ${LIBRARY_INSTALL_PATH}:${CMAKE_INSTALL_RPATH})
        else(${CMAKE_INSTALL_RPATH})
            set(CMAKE_INSTALL_RPATH ${LIBRARY_INSTALL_PATH})
        endif(${CMAKE_INSTALL_RPATH})

        # for osx, which uses "install name" path rather than rpath
        #set(CMAKE_INSTALL_NAME_DIR ${LIBRARY_OUTPUT_PATH})
        set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})
        
        # hack to force cmake always create install and clean targets 
        install(FILES DESTINATION)
        string(RANDOM LENGTH 32 __rand_target__name__)
        add_custom_target(${__rand_target__name__})
        unset(__rand_target__name__)

        set(__pods_setup true)
    endif(NOT DEFINED __pods_setup)
endmacro(pods_config_search_paths)

macro(enforce_out_of_source)
    if(CMAKE_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
      message(FATAL_ERROR 
      "\n
      Do not run cmake directly in the pod directory. 
      use the supplied Makefile instead!  You now need to
      remove CMakeCache.txt and the CMakeFiles directory.

      Then to build, simply type: 
       $ make
      ")
    endif()
endmacro(enforce_out_of_source)

#set the variable POD_NAME to the directory path, and set the cmake PROJECT_NAME
if(NOT POD_NAME)
    get_filename_component(POD_NAME ${CMAKE_SOURCE_DIR} NAME)
    message(STATUS "POD_NAME is not set... Defaulting to directory name: ${POD_NAME}") 
endif(NOT POD_NAME)
project(${POD_NAME})

#make sure we're running an out-of-source build
enforce_out_of_source()

#call the function to setup paths
pods_config_search_paths()
