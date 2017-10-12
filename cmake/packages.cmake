#------------------------------------------------------------------------------
# Find external packages.
#------------------------------------------------------------------------------
macro(drake_find_packages)
  find_package(Matlab MODULE REQUIRED
    COMPONENTS MAIN_PROGRAM MEX_COMPILER MX_LIBRARY)
  find_package(Python 2.7 MODULE REQUIRED)

  find_package(Doxygen MODULE REQUIRED)
  find_package(Threads MODULE REQUIRED)

  find_package(Eigen3 CONFIG REQUIRED)
  set_property(TARGET Eigen3::Eigen APPEND PROPERTY
    INTERFACE_COMPILE_DEFINITIONS EIGEN_MPL2_ONLY)  # Per #4065.

  find_package(fmt CONFIG REQUIRED)
  find_package(gflags CONFIG REQUIRED)
  find_package(NLopt CONFIG REQUIRED)
  find_package(Protobuf MODULE REQUIRED)

  find_package(pybind11 CONFIG REQUIRED COMPONENTS NumPy)
  if(NUMPY_VERSION VERSION_LESS 1.7)
    message(FATAL_ERROR "NumPy version 1.7 or above NOT found")
  endif()

  find_package(tinyobjloader CONFIG REQUIRED)
endmacro()
