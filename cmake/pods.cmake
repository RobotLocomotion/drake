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
# General
#   pods_find_pkg_config(...)
#   pods_install_pkg_config_file(...)
#   pods_install_bash_setup(...)
#   get_relative_path(from to)
#
# C/C++
#   pods_install_headers(...)
#   pods_install_libraries(...)
#   pods_install_executables(...)
#
#   pods_use_pkg_config_packages(...)
#   pods_use_pkg_config_includes(...)
#
# Python
#   pods_install_python_packages(...)
#   pods_install_python_script(...)
#
# Java
#   pods_install_jars(...)
#   pods_use_pkg_config_classpath(...)
#
# ----
# File: pods.cmake
# Distributed with pods version: 12.11.14

function(call_cygpath format var)
#  message(STATUS "calling cygpath with ${format} ${${var}}")
#  message("before cygpath: ${${var}}")
#  separate_arguments(${var})
  if (NOT ${var})  # do nothing if var is empty
    return()
  endif()
  # this only works if the incoming path is unix style /
  # otherwise the zap the \ removes all the directory separators
  # so convert to cmake path first
  file(TO_CMAKE_PATH  ${${var}}  ${var})
  string(REGEX REPLACE "([^\\\\]) " "\\1;" ${var} ${${var}})  # separate arguments didn't respect the "Program\ Files"... it resulted in "Program;Files"
  string(REGEX REPLACE "\\\\" "" ${var} "${${var}}")  # now zap the \
  string(STRIP ${${var}} ${var})
  execute_process(COMMAND ${cygpath} ${format} ${${var}} OUTPUT_VARIABLE varout OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REGEX REPLACE "(\r?\n)+" ";" varout ${varout})
#  message("after cygpath ${format}: ${varout}")
  set(${var} ${varout} PARENT_SCOPE)
endfunction()

# On windows, the compilers and the shell commands (potentially) use different syntax for their path strings.
# These macros try to handle that case as cleanly as possible, and do nothing on non-windows
macro(c_compiler_path var)
  if (WIN32 AND cygpath AND ${var})
    call_cygpath(-m ${var})
  endif()
endmacro()

macro(java_compiler_path var)
  if (WIN32 AND cygpath AND ${var})
    call_cygpath(-m ${var})
  endif()
endmacro()

macro(cmake_path var)
  if (WIN32 AND cygpath AND ${var})
    call_cygpath(-m ${var})
  endif()
endmacro()

macro(pkg_config_path var)
  if (WIN32)
    if (cygpath AND ${var})
      call_cygpath(-w ${var})
    else()
      string(REGEX REPLACE "\\/" "\\\\" ${var} ${${var}})
    endif()
  endif()
endmacro()

macro(shell_path var)
  if (WIN32)
    if (cygpath AND ${var})
      # we never want /cygdrive as cmake and other
      # native tools will not know what that is, so
      # use -m to get unix paths with drive:
      call_cygpath(-m ${var})
    else()
      file(TO_CMAKE_PATH ${${var}} ${var})
    endif()
  endif()
endmacro()


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
    install(TARGETS ${ARGV} RUNTIME DESTINATION lib LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
endfunction(pods_install_libraries)

function(pods_install_jars)
  foreach(jarfile ${ARGV})
    install_jar(${jarfile} share/java)
  endforeach()
endfunction(pods_install_jars)

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
            if ("${pc_classpath}" STREQUAL "")
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
        else(${mode_index} GREATER -1)
            message("WARNING incorrect use of pods_add_pkg_config (${word})")
            break()
        endif(${mode_index} GREATER -1)
    endforeach(word)

    set(prefix ${CMAKE_INSTALL_PREFIX})
    shell_path(prefix)

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

endfunction(pods_install_pkg_config_file)

# pods_install_bash_setup(<package-name> <line1> <line2> ...)
#
# Create and install the lines into config/${package}_setup.sh
#
# example:
#    pods_install_bash_setup(mypod "export LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib")
function(pods_install_bash_setup package)
  list(REMOVE_AT ARGV 0)
  set(filename ${PROJECT_BINARY_DIR}/config/${package}_setup.sh)

  # todo: add a \n at the end of every element of ARGV?
  file(WRITE ${filename} ${ARGV} )

  install(FILES ${filename} DESTINATION config)
#  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/config/${package}_setup.sh)

  if (APPLE)
     set(LD_LIBRARY_PATH DYLD_LIBRARY_PATH)
  else()
     set(LD_LIBRARY_PATH LD_LIBRARY_PATH)
  endif()

  set(prefix ${CMAKE_INSTALL_PREFIX})
  shell_path(prefix)

  set(filename ${PROJECT_BINARY_DIR}/config/pods_setup_all.sh)
  file(WRITE ${filename}
    "# THIS FILE IS AUTOMATICALLY GENERATED.  ANY EDITS YOU MAKE WILL LIKELY BE OVERWRITTEN.\n"
    "\n"
    "export PATH=$PATH:${prefix}/bin\n"
    "export ${LD_LIBRARY_PATH}=\$${LD_LIBRARY_PATH}:${prefix}/lib\n"
    "for i in ${prefix}/config/*_setup.sh; do\n"
    "  echo sourcing $i;\n"
    "  source $i;\n"
    "done\n"
  )
  install(FILES ${filename} DESTINATION config)
#  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/config/pods_setup_all.sh)

endfunction()

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


# pods_find_pkg_config(<package-name> <minimum_version>)
#
# Invokes `pkg-config --exists <package-name>` and, per the cmake standard,
# sets the variable <package-name>_FOUND if it succeeds
#
# Takes an optional minimum version number as the second argument.
#
# example usage:
#   pods_find_pkg_config(eigen3)
#   if (eigen3_FOUND)
#      ... do something ...
#   endif()
function(pods_find_pkg_config)
    if (DEFINED ${ARGV0}_FOUND AND ${ARGV0}_FOUND AND ${ARGC} EQUAL 1) # not caching version info yet (but I could)
      return()
    endif()
    if (NOT PKG_CONFIG_EXECUTABLE)
      find_package(PkgConfig REQUIRED)
    endif()

    if(${ARGC} EQUAL 1)
      execute_process(COMMAND
        ${PKG_CONFIG_EXECUTABLE} --exists ${ARGV}
        RESULT_VARIABLE found)
    elseif(${ARGC} EQUAL 2)
      execute_process(COMMAND
        ${PKG_CONFIG_EXECUTABLE} --atleast-version=${ARGV1} ${ARGV0}
        RESULT_VARIABLE found)
    else()
      message(FATAL_ERROR "pods_find_pkg_config take one or two arguments")
    endif()

    if (found EQUAL 0)
       message(STATUS "Found ${ARGV0}")
       set(${ARGV0}_FOUND 1 CACHE BOOL "" FORCE)
    else()
      message(STATUS "Could NOT find ${ARGV0} (version >= ${ARGV1}) using pods_find_pkg_config. PKG_CONFIG_PATH = $ENV{PKG_CONFIG_PATH}")
      set(${ARGV0}_FOUND 0 CACHE BOOL "" FORCE)
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
  elseif (${ARGC} GREATER 2)
    foreach (__package ${ARGN})
      pods_use_pkg_config_packages(${target} ${__package})
    endforeach()
  else()
    if (NOT PKG_CONFIG_EXECUTABLE)
      find_package(PkgConfig REQUIRED)
    endif()

    string(STRIP ${ARGN} __package)

    if (DEFINED ${__package}_CONFIG_IN_CACHE) # then i've already searched
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

    	include(FindPkgConfig)
    	pkg_check_modules(PODS_PKG ${__package})
      if (NOT PODS_PKG_FOUND)
         message(FATAL_ERROR "ERROR: pods_use_pkg_config_packages FAILED.  could not find packages ${ARGN}.  PKG_CONFIG_PATH = $ENV{PKG_CONFIG_PATH}")
      endif()

      set("${__package}_CONFIG_IN_CACHE" "${PODS_PKG_FOUND}" CACHE STRING "")
      set("${__package}_LIBRARIES" "${PODS_PKG_LIBRARIES}" CACHE STRING "")
      set("${__package}_LIBRARY_DIRS" "${PODS_PKG_LIBRARY_DIRS}" CACHE STRING "")
      set("${__package}_LDFLAGS" "${PODS_PKG_LDFLAGS}" CACHE STRING "")
      set("${__package}_LDFLAGS_OTHER" "${PODS_PKG_LDFLAGS_OTHER}" CACHE STRING "")
      set("${__package}_INCLUDE_DIRS" "${PODS_PKG_INCLUDE_DIRS}" CACHE STRING "")
      set("${__package}_CFLAGS" "${PODS_PKG_CFLAGS}" CACHE STRING "")
      set("${__package}_CFLAGS_OTHER" "${PODS_PKG_CFLAGS_OTHER}" CACHE STRING "")
    endif()

#    message(STATUS "using pkg ${__package}")
#    message(STATUS "  FOUND = ${PODS_PKG_FOUND}")
#    message(STATUS "  LIBRARIES = ${PODS_PKG_LIBRARIES}")
#    message(STATUS "  LIBRARY_DIRS = ${PODS_PKG_LIBRARY_DIRS}")
#    message(STATUS "  LDFLAGS = ${PODS_PKG_LDFLAGS}")
#    message(STATUS "  LDFLAGS_OTHER = ${PODS_PKG_LDFLAGS_OTHER}")
#    message(STATUS "  INCLUDE_DIRS = ${PODS_PKG_INCLUDE_DIRS}")
#    message(STATUS "  CFLAGS = ${PODS_PKG_CFLAGS}")
#    message(STATUS "  CFLAGS_OTHER = ${PODS_PKG_CFLAGS_OTHER}")

  	foreach (__inc_dir ${PODS_PKG_INCLUDE_DIRS})
      string(STRIP ${__inc_dir} __inc_dir)
	    if (__inc_dir)
	      c_compiler_path(__inc_dir)
      #	    message("include: ${__inc_dir}")
        include_directories(SYSTEM ${__inc_dir})
      endif()
    endforeach()

  	foreach(__ld_dir ${PODS_PKG_LIBRARY_DIRS})
  	  string(STRIP ${__ld_dir} __ld_dir)
  	  if (__ld_dir)
  	    c_compiler_path(__ld_dir)
  	    if (WIN32)  # only MSVC?
          target_link_libraries(${target} "-LIBPATH:${__ld_dir}")
        else()
  	      target_link_libraries(${target} "-L${__ld_dir}")
        endif()
      endif()
  	endforeach()

  	# make the target depend on libraries that are cmake targets
  	foreach(__depend_target_name ${PODS_PKG_LIBRARIES})
      #         message(STATUS "${target} depends on  ${__depend_target_name}")
      if (TARGET ${__depend_target_name})
        target_link_libraries(${target} ${__depend_target_name})
      else()
        target_link_libraries(${target} ${__depend_target_name})
        # ask cmake to actually find the library (tried this to help when i had only dynamic versions of some libraries, and msvc was only looking for static)
        #	    set(mylib "")
        #	    find_library(mylib ${__depend_target_name} HINTS ${_pods_pkg_ld_dirs})
        #            message(STATUS ${mylib})
        #	    if (NOT mylib)
        #	      message(FATAL_ERROR "Could not find library ${__depend_target_name} specified in pkg-config ${__package} (looked in ${_pods_pkg_ld_dirs} in addition to the usual places)")
        #	    else()
        #              message(STATUS "FOUND ${mylib}")
        #	    endif()
        #	    target_link_libraries(${target} ${mylib})
      endif()
    endforeach()

    if (PODS_PKG_LDFLAGS_OTHER)
  	  target_link_libraries(${target} ${PODS_PKG_LDFLAGS_OTHER})
    endif()

    if (PODS_PKG_CFLAGS_OTHER)
      # TODO: Handle more PODS_PKG_CFLAGS_OTHER flags
      string(FIND "${PODS_PKG_CFLAGS_OTHER}" "-pthread" PTHREAD_POS)

      # handle pthread
      if (PTHREAD_POS GREATER -1)
        # from http://stackoverflow.com/a/29871891
        find_package(Threads REQUIRED)
        if (THREADS_HAVE_PTHREAD_ARG)
          target_compile_options(${target} PUBLIC "-pthread")
        endif()
        if (CMAKE_THREAD_LIBS_INIT)
          target_link_libraries(${target} "${CMAKE_THREAD_LIBS_INIT}")
        endif()
      endif()
    endif(PODS_PKG_CFLAGS_OTHER)
  endif()
endfunction()

# pods_use_pkg_config_includes(<package-name> ...)
#
# Invokes `pkg-config --cflags-only-I <package-name> ...` and adds the result to the
# include directories.
#
macro(pods_use_pkg_config_includes)
  if(${ARGC} LESS 1)
    message(WARNING "Useless invocation of pods_use_pkg_config_includes")
  else()
    if (NOT PKG_CONFIG_EXECUTABLE)
      find_package(PkgConfig REQUIRED)
    endif()

    execute_process(COMMAND
      ${PKG_CONFIG_EXECUTABLE} --cflags-only-I ${ARGN}
      OUTPUT_VARIABLE _pods_pkg_include_flags)
    string(STRIP ${_pods_pkg_include_flags} _pods_pkg_include_flags)
    string(REPLACE "-I" "" _pods_pkg_include_flags "${_pods_pkg_include_flags}")

    c_compiler_path(_pods_pkg_include_flags)
    include_directories(SYSTEM ${_pods_pkg_include_flags})
  endif()
endmacro()


# pods_use_pkg_config_classpath(<package-name> ...)
#
# Convenience macro to get classpath flags from pkg-config and add them to CMAKE_JAVA_INCLUDE_PATH
#
# Invokes `pkg-config --variable=classpath <package-name> ...`, adds the result to the
# include path, and then calls pods_use_pkg_config_classpath on the required packages (to recursively add the path)
#
# also sets the variable <package-name>
# example:
#   pods_use_pkg_config_classpath(lcm-java)

function(pods_use_pkg_config_classpath)
    if(${ARGC} LESS 1)
        message(WARNING "Useless invocation of pods_use_pkg_config_packages")
        return()
    endif()
    if (NOT PKG_CONFIG_EXECUTABLE)
      find_package(PkgConfig REQUIRED)
    endif()

    foreach(arg ${ARGV})
      string(STRIP ${arg} _arg)
      execute_process(COMMAND
        ${PKG_CONFIG_EXECUTABLE} --variable=classpath ${arg}
        OUTPUT_VARIABLE _pods_pkg_classpath_flags)
      string(STRIP ${_pods_pkg_classpath_flags} _pods_pkg_classpath_flags)
      if (NOT WIN32)
        string(REPLACE ":" " " _pods_pkg_classpath_flags ${_pods_pkg_classpath_flags})
      endif()
      java_compiler_path(_pods_pkg_classpath_flags)
      if (NOT WIN32)
        string(REPLACE " " ":" _pods_pkg_classpath_flags ${_pods_pkg_classpath_flags})
      endif()

      set( CMAKE_JAVA_INCLUDE_PATH ${CMAKE_JAVA_INCLUDE_PATH}:${_pods_pkg_classpath_flags})
      string(REPLACE "::" ":" CMAKE_JAVA_INCLUDE_PATH ${CMAKE_JAVA_INCLUDE_PATH})
      string(REGEX REPLACE "^:" "" CMAKE_JAVA_INCLUDE_PATH ${CMAKE_JAVA_INCLUDE_PATH})

      execute_process(COMMAND
          ${PKG_CONFIG_EXECUTABLE} --print-requires ${arg}
          OUTPUT_VARIABLE _pods_pkg_classpath_requires OUTPUT_STRIP_TRAILING_WHITESPACE)

      if (NOT "${_pods_pkg_classpath_requires}" STREQUAL "")
          string(STRIP ${_pods_pkg_classpath_requires} _pods_pkg_classpath_requires)
          pods_use_pkg_config_classpath(${_pods_pkg_classpath_requires})
      endif()

      set( ${_arg}_CLASSPATH "${_pods_pkg_classpath_flags}" PARENT_SCOPE )
    endforeach()

    set( CMAKE_JAVA_INCLUDE_PATH ${CMAKE_JAVA_INCLUDE_PATH} PARENT_SCOPE )
endfunction()


# pods_config_search_paths()
#
# Setup include, linker, and pkg-config paths according to the pods core
# policy.  This macro is automatically invoked, there is no need to do so
# manually.
macro(pods_config_search_paths)
    if(NOT DEFINED __pods_setup)
      #set where files should be output locally
      set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
      set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
      foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
        string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
        set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${LIBRARY_OUTPUT_PATH} )
      endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )
      set(PKG_CONFIG_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib/pkgconfig)

      #set where files should be installed to
      set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
      set(EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
      set(INCLUDE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)
      set(PKG_CONFIG_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

      pkg_config_path(PKG_CONFIG_OUTPUT_PATH)
      pkg_config_path(PKG_CONFIG_INSTALL_PATH)

      # add build/lib/pkgconfig to the pkg-config search path
      if (WIN32)
      	set(_path "${PKG_CONFIG_OUTPUT_PATH};${PKG_CONFIG_INSTALL_PATH};$ENV{PKG_CONFIG_PATH}")
      	string(REGEX REPLACE ";+$" "" _path "${_path}")
      	set(ENV{PKG_CONFIG_PATH} "${_path}")
      else()
      	set(ENV{PKG_CONFIG_PATH} "${PKG_CONFIG_OUTPUT_PATH}:${PKG_CONFIG_INSTALL_PATH}:$ENV{PKG_CONFIG_PATH}")
      endif()

      shell_path(PKG_CONFIG_OUTPUT_PATH)
      shell_path(PKG_CONFIG_INSTALL_PATH)

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

      set(__pods_setup true)
    endif(NOT DEFINED __pods_setup)
endmacro(pods_config_search_paths)

macro(enforce_out_of_source)
    if(PROJECT_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
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
    get_filename_component(POD_NAME ${PROJECT_SOURCE_DIR} NAME)
    message(STATUS "POD_NAME is not set... Defaulting to directory name: ${POD_NAME}")
endif(NOT POD_NAME)
project(${POD_NAME})
set(POD_NAME "${POD_NAME}" CACHE STRING "${POD_NAME}" )

# PODs out-of-source build logic
if (CMAKE_INSTALL_PREFIX STREQUAL "/usr/local" OR
    CMAKE_INSTALL_PREFIX STREQUAL "C:/Program Files/Project" OR
    CMAKE_INSTALL_PREFIX STREQUAL "C:/Program Files (x86)/Project")
  find_file(_build_dir build PATHS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/.. ${PROJECT_SOURCE_DIR}/../.. ${PROJECT_SOURCE_DIR}/../../.. ${PROJECT_SOURCE_DIR}/../../../..)
  if (_build_dir)
    set(CMAKE_INSTALL_PREFIX "${_build_dir}")
  else()
    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/build)
    set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/build)
  endif()
endif()
message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

if ( WIN32 ) # convert to windows paths
   find_program(cygpath cygpath)
endif()
cmake_path(CMAKE_INSTALL_PREFIX)

#make sure we're running an out-of-source build
enforce_out_of_source()

#call the function to setup paths
pods_config_search_paths()
