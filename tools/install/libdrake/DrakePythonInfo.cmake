find_package(PythonInterp 2.7 EXACT MODULE REQUIRED)

# TODO(eric.cousineau): Update when #8014 is fully resolved.
set(_drake_sitepackages
    "${drake_DIR}/../../python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages")
get_filename_component(_drake_sitepackages "${_drake_sitepackages}" ABSOLUTE)

# TODO(eric.cousineau): Have a test inherit CMake's environment variables?
set(ENV{PYTHONPATH} "${_drake_sitepackages}:$ENV{PYTHONPATH}")
