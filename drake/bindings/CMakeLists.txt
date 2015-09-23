set(python_install_dir lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages)

add_subdirectory(swig)
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python/pydrake)
