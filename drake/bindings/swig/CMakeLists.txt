list(APPEND CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/share/cmake)
drake_find_package(SWIG MODULE REQUIRED)
set(DRAKE_SWIG_FILES ${CMAKE_INSTALL_PREFIX}/share/swig)
include(DrakeUseSWIG)

# Ignore SWIG errors about unknown base types
set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-w315,-401")
# Ignore SWIG errors about not being able to wrap operator <<
set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-w503")
# The autodiff.i in swig has for-loop variable warnings.
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} APPEND PROPERTY
  COMPILE_OPTIONS ${CXX_FLAGS_NO_SIGN_COMPARE})
# SWIG output code makes expansive use of variable shadowing.
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} APPEND PROPERTY
  COMPILE_OPTIONS ${CXX_FLAGS_NO_ERROR_SHADOW})

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Suppresses warnings due to the existence of deprecated methods. These
  # deprecated methods should still be made available through the SWIG-generated
  # APIs until they are actually removed from the original C++ APIs.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

  # Swig sometimes emits code which appears to have uninitialized
  # variables.  Err on the side of swig being correct.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized")
endif()

drake_find_package(snopt_c CONFIG)

if(PYTHON_EXECUTABLE)
  include(SwigPython)
  set(python_install_subdir lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages ${CMAKE_CURRENT_SOURCE_DIR}/../python)

  add_swig_python_module(autodiffPython autodiffutils.i
    CPLUSPLUS
    SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python ${DRAKE_SWIG_FILES}/common
    DESTINATION ${python_install_subdir})

  add_swig_python_module(rbtPython rbtree.i
    CPLUSPLUS
    LINK_LIBRARIES drakeRBM
    SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python ${DRAKE_SWIG_FILES}/common
    DESTINATION ${python_install_subdir})

  add_swig_python_module(ikPython ik.i
    CPLUSPLUS
    LINK_LIBRARIES drakeIK
    SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python ${DRAKE_SWIG_FILES}/common
    DESTINATION ${python_install_subdir})
endif()

if(Matlab_FOUND)
  if(SWIG_HAS_MATLAB_SUPPORT)
    # TODO: we can remove this check when mainline SWIG gets MATLAB support someday, or when we figure out how to build swig-matlab on Windows
    set(matlab_install_subdir ${CMAKE_BINARY_DIR}/bindings/matlab)
    file(MAKE_DIRECTORY ${matlab_install_subdir}) # so that swig -MM can put dependency lists there at configure-time
    message("Found Swig with Matlab support")
    include(SwigMatlab)
    add_swig_matlab_module(autodiffMatlab autodiffutils_matlab.i
      CPLUSPLUS
      INCLUDE_DIRS
      SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab ${DRAKE_SWIG_FILES}/common
      DESTINATION ${matlab_install_subdir})

    add_swig_matlab_module(rbtMatlab rbtree_matlab.i
      CPLUSPLUS
      LINK_LIBRARIES drakeRBM
      SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab ${DRAKE_SWIG_FILES}/common
      DESTINATION ${matlab_install_subdir})

    add_swig_matlab_module(ikMatlab ik_matlab.i
      CPLUSPLUS
      LINK_LIBRARIES drakeIK
      SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab ${DRAKE_SWIG_FILES}/common
      DESTINATION ${matlab_install_subdir})
  else()
    message("Could NOT find Swig with Matlab support; not generating Matlab bindings")
  endif()
endif()
