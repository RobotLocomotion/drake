# Macros to simplify compliance with the pods build policies.
#
# To enable the macros, add the following line to CMakeLists.txt:
#   include(cmake/pods.cmake)
#
# Next, any of the following macros can be used.  See the individual macro
# definitions in this file for individual documentation.
#
# General
#   pods_find_pkg_config(...)
#   pods_install_pkg_config_file(...)
#
# C/C++
#   pods_install_libraries(...)
#   pods_install_executables(...)
#
#   pods_use_pkg_config_packages(...)
#
# Python
#   pods_install_python_packages(...)
#
# ----
# File: pods.cmake
# Distributed with pods version: 12.11.14


# pods_install_executables(<executable1> ...)
#
# Install a (list) of executables to bin/
function(pods_install_executables)
    install(TARGETS ${ARGV} RUNTIME DESTINATION bin)
endfunction()


# pods_install_libraries(<library1> ...)
#
# Install a (list) of libraries to lib/
function(pods_install_libraries)
    install(TARGETS ${ARGV} RUNTIME DESTINATION lib LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
endfunction()


# pods_install_pkg_config_file(<package-name>
#                              [VERSION <version>]
#                              [DESCRIPTION <description>]
#                              [CFLAGS <cflag> ...]
#                              [LIBS <lflag> ...]
#                              [CLASSPATH <target-jar1> <target-jar2> ...]
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
    set(pc_classpath "")
    set(pc_fname "${PKG_CONFIG_OUTPUT_PATH}/${pc_name}.pc")

    set(modewords LIBS CFLAGS CLASSPATH REQUIRES VERSION DESCRIPTION)
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
	elseif(curmode STREQUAL CLASSPATH)
            if("${pc_classpath}" STREQUAL "")
	       set(pc_classpath "\${prefix}/share/java/${word}.jar")
	    else()
	       set(pc_classpath "${pc_classpath}:\${prefix}/share/java/${word}.jar")
	    endif()
        elseif(curmode STREQUAL REQUIRES)
            set(pc_requires "${pc_requires} ${word}")
        elseif(curmode STREQUAL VERSION)
            set(pc_version ${word})
            set(curmode "")
        elseif(curmode STREQUAL DESCRIPTION)
            set(pc_description "${word}")
            set(curmode "")
        else()
            message("WARNING incorrect use of pods_add_pkg_config (${word})")
            break()
        endif()
    endforeach()

    set(prefix ${CMAKE_INSTALL_PREFIX})

    # write the .pc file out
    file(WRITE ${pc_fname}
        "prefix=${prefix}\n"
        "exec_prefix=\${prefix}\n"
        "libdir=\${exec_prefix}/lib\n"
        "includedir=\${prefix}/include\n"
        "\n"
        "Name: ${pc_name}\n"
        "Description: ${pc_description}\n"
        "Requires: ${pc_requires}\n"
        "Version: ${pc_version}\n"
        "Libs: -L\${libdir} ${pc_libs}\n"
        "Cflags: -I\${includedir} ${pc_cflags}\n"
	"classpath=${pc_classpath}\n"
	)

    # mark the .pc file for installation to the lib/pkgconfig directory
    install(FILES ${pc_fname} DESTINATION lib/pkgconfig)
endfunction()


# pods_install_python_packages(<src_dir1> ...)
#
# Install python packages to lib/pythonX.Y/dist-packages, where X.Y refers to
# the current python version (e.g., 2.6)
#
# The __init__.py file in <src_dir> will be installed along with any .py files
# in subdirectories.
function(pods_install_python_packages py_src_dir)
    get_filename_component(py_src_abs_dir ${py_src_dir} ABSOLUTE)
    get_filename_component(py_module_name ${py_src_abs_dir} NAME)

    # where do we install .py files to?
    set(python_install_dir
        ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages)

    file(GLOB_RECURSE module_files ${py_src_abs_dir}/*)
    foreach(file ${module_files})
      if(NOT file MATCHES ".*\\.pyc|.*[~#]")
        file(RELATIVE_PATH __tmp_path ${py_src_abs_dir} ${file})
        get_filename_component(__tmp_dir ${__tmp_path} PATH)
        install(FILES ${file}
          DESTINATION "${python_install_dir}/${py_module_name}/${__tmp_dir}")
      endif()
    endforeach()
endfunction()


# pods_find_pkg_config(<package-name>)
#
# Invokes `pkg-config --exists <package-name>` and, per the cmake standard,
# sets the variable <package-name>_FOUND if it succeeds
#
# example usage:
#   pods_find_pkg_config(eigen3)
#   if(eigen3_FOUND)
#      ... do something ...
#   endif()
function(pods_find_pkg_config package)
    if(${package}_FOUND)
      return()
    endif()
    find_package(PkgConfig REQUIRED)

    execute_process(COMMAND
      ${PKG_CONFIG_EXECUTABLE} --exists ${package}
      RESULT_VARIABLE found)

    if(found EQUAL 0)
       message(STATUS "Found ${package}")
       set(${package}_FOUND TRUE CACHE BOOL "" FORCE)
    else()
      message(STATUS "Could NOT find ${package} using pods_find_pkg_config. PKG_CONFIG_PATH = $ENV{PKG_CONFIG_PATH}")
      set(${package}_FOUND FALSE CACHE BOOL "" FORCE)
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
function(pods_use_pkg_config_packages target)
  if(${ARGC} LESS 2)
    message(WARNING "Useless invocation of pods_use_pkg_config_packages")
  elseif(${ARGC} GREATER 2)
    foreach(__package ${ARGN})
      pods_use_pkg_config_packages(${target} ${__package})
    endforeach()
  else()
    find_package(PkgConfig REQUIRED)

    string(STRIP ${ARGN} __package)

    if(DEFINED ${__package}_CONFIG_IN_CACHE) # then i've already searched
      set(PODS_PKG_FOUND "${${__package}_CONFIG_IN_CACHE}")
      set(PODS_PKG_LIBRARIES "${${__package}_LIBRARIES}")
      set(PODS_PKG_LIBRARY_DIRS "${${__package}_LIBRARY_DIRS}")
      set(PODS_PKG_LDFLAGS "${${__package}_LDFLAGS}")
      set(PODS_PKG_LDFLAGS_OTHER "${${__package}_LDFLAGS_OTHER}")
      set(PODS_PKG_INCLUDE_DIRS "${${__package}_INCLUDE_DIRS}")
      set(PODS_PKG_CFLAGS "${${__package}_CFLAGS}")
      set(PODS_PKG_CFLAGS_OTHER "${${__package}_CFLAGS_OTHER}")
    else()
      set(PODS_PKG_FOUND "")
      set(PODS_PKG_LIBRARIES "")
      set(PODS_PKG_LIBRARY_DIRS "")
      set(PODS_PKG_LDFLAGS "")
      set(PODS_PKG_LDFLAGS_OTHER "")
      set(PODS_PKG_INCLUDE_DIRS "")
      set(PODS_PKG_CFLAGS "")
      set(PODS_PKG_CFLAGS_OTHER "")

      pkg_check_modules(PODS_PKG ${__package})
      if(NOT PODS_PKG_FOUND)
         message(FATAL_ERROR "ERROR: pods_use_pkg_config_packages FAILED.  could not find packages ${ARGN}.  PKG_CONFIG_PATH = $ENV{PKG_CONFIG_PATH}")
      endif()

      set(${__package}_CONFIG_IN_CACHE "${PODS_PKG_FOUND}" CACHE STRING "")
      set(${__package}_LIBRARIES "${PODS_PKG_LIBRARIES}" CACHE STRING "")
      set(${__package}_LIBRARY_DIRS "${PODS_PKG_LIBRARY_DIRS}" CACHE STRING "")
      set(${__package}_LDFLAGS "${PODS_PKG_LDFLAGS}" CACHE STRING "")
      set(${__package}_LDFLAGS_OTHER "${PODS_PKG_LDFLAGS_OTHER}" CACHE STRING "")
      set(${__package}_INCLUDE_DIRS "${PODS_PKG_INCLUDE_DIRS}" CACHE STRING "")
      set(${__package}_CFLAGS "${PODS_PKG_CFLAGS}" CACHE STRING "")
      set(${__package}_CFLAGS_OTHER "${PODS_PKG_CFLAGS_OTHER}" CACHE STRING "")
    endif()

  	foreach(__inc_dir ${PODS_PKG_INCLUDE_DIRS})
      string(STRIP ${__inc_dir} __inc_dir)
	    if(__inc_dir)
          include_directories(SYSTEM ${__inc_dir})
        endif()
    endforeach()

  	foreach(__ld_dir ${PODS_PKG_LIBRARY_DIRS})
  	  string(STRIP ${__ld_dir} __ld_dir)
  	  if(__ld_dir)
  	    target_link_libraries(${target} "-L${__ld_dir}")
      endif()
  	endforeach()

  	# make the target depend on libraries that are cmake targets
  	foreach(__depend_target_name ${PODS_PKG_LIBRARIES})
      if(TARGET ${__depend_target_name})
        target_link_libraries(${target} ${__depend_target_name})
      else()
        target_link_libraries(${target} ${__depend_target_name})
      endif()
    endforeach()

    if(PODS_PKG_LDFLAGS_OTHER)
  	  target_link_libraries(${target} ${PODS_PKG_LDFLAGS_OTHER})
    endif()

    if(PODS_PKG_CFLAGS_OTHER)
      # TODO: Handle more PODS_PKG_CFLAGS_OTHER flags
      string(FIND "${PODS_PKG_CFLAGS_OTHER}" "-pthread" PTHREAD_POS)

      # handle pthread
      if(PTHREAD_POS GREATER -1)
        # from http://stackoverflow.com/a/29871891
        find_package(Threads REQUIRED)
        if(THREADS_HAVE_PTHREAD_ARG)
          target_compile_options(${target} PUBLIC "-pthread")
        endif()
        if(CMAKE_THREAD_LIBS_INIT)
          target_link_libraries(${target} "${CMAKE_THREAD_LIBS_INIT}")
        endif()
      endif()
    endif()
  endif()
endfunction()


# pods_config_search_paths()
#
# Setup include, linker, and pkg-config paths according to the pods core
# policy.  This macro is automatically invoked, there is no need to do so
# manually.
macro(pods_config_search_paths)
    if(NOT DEFINED __pods_setup)
      # set where files should be output locally
      set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
      set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
      foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${LIBRARY_OUTPUT_PATH})
      endforeach()
      set(PKG_CONFIG_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib/pkgconfig)

      # set where files should be installed to
      set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
      set(EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
      set(INCLUDE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)
      set(PKG_CONFIG_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

      # add build/lib/pkgconfig to the pkg-config search path
      set(ENV{PKG_CONFIG_PATH} "${PKG_CONFIG_OUTPUT_PATH}:${PKG_CONFIG_INSTALL_PATH}:$ENV{PKG_CONFIG_PATH}")

      # add build/lib to the link path
      link_directories(${LIBRARY_OUTPUT_PATH})
      link_directories(${LIBRARY_INSTALL_PATH})

      # abuse RPATH
      if(CMAKE_INSTALL_RPATH)
        set(CMAKE_INSTALL_RPATH ${LIBRARY_INSTALL_PATH}:${CMAKE_INSTALL_RPATH})
      else()
        set(CMAKE_INSTALL_RPATH ${LIBRARY_INSTALL_PATH})
      endif()

      # for OS X, which uses "install name" path rather than rpath
      set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})

      set(__pods_setup TRUE)
    endif()
endmacro()

pods_config_search_paths()
