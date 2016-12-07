# Macros to simplify compliance with the pods build policies.
#
# To enable the macros, add the following line to CMakeLists.txt:
#   include(cmake/pods.cmake)
#
# Next, any of the following macros can be used.  See the individual macro
# definitions in this file for individual documentation.
#
# Python
#   pods_install_python_packages(...)
#
# ----
# File: pods.cmake
# Distributed with pods version: 12.11.14

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
      "${DRAKE_LIBRARY_DIR}/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages")

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

# pods_config_search_paths()
#
# Setup include, linker, and pkg-config paths according to the pods core
# policy.  This macro is automatically invoked, there is no need to do so
# manually.
macro(pods_config_search_paths)
    if(NOT DEFINED __pods_setup)
      # set where files should be installed to
      set(LIBRARY_INSTALL_PATH "${DRAKE_LIBRARY_DIR}")

      # add install/lib/pkgconfig to the pkg-config search path so we find
      # .pc files installed by externals (use both the correct library dir, and
      # plain 'lib', as some externals may not use the correct library dir)
      set(ENV{PKG_CONFIG_PATH} "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/pkgconfig:${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

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
