set(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/share/cmake ${CMAKE_MODULE_PATH})
if (EXISTS "${CMAKE_INSTALL_PREFIX}/bin/swig")
  set(SWIG_EXECUTABLE ${CMAKE_INSTALL_PREFIX}/bin/swig)
endif()
find_package(SWIG REQUIRED)
set(DRAKE_SWIG_FILES ${CMAKE_INSTALL_PREFIX}/share/swig)
include(DrakeUseSWIG)
include(SwigPython)

# Ignore SWIG errors about unknown base types
set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-w315,-401")
# Ignore SWIG errors about not being able to wrap operator <<
set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-w503")

add_swig_python_module(autodiffPython autodiffutils.i
                       CPLUSPLUS
                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python ${DRAKE_SWIG_FILES}/common
                       DESTINATION ${python_install_subdir})

add_swig_python_module(rbtPython rbtree.i
                       CPLUSPLUS
                       INCLUDE_DIRS 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../../util
                         ${CMAKE_CURRENT_SOURCE_DIR}/../../systems
                         ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants
                         ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/joints 
                       LINK_LIBRARIES drakeRBM 
                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python ${DRAKE_SWIG_FILES}/common
                       DESTINATION ${python_install_subdir})

if (snopt_c_FOUND)
  add_swig_python_module(ikPython ik.i
                         CPLUSPLUS
                         INCLUDE_DIRS 
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../util
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../systems
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/constraint 
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/joints 
                         LINK_LIBRARIES drakeIK 
                         SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python ${DRAKE_SWIG_FILES}/common
                         DESTINATION ${python_install_subdir})
endif()


if (MATLAB_FOUND)
  # TODO: we can remove this check when mainline swig gets matlab support someday, or when we figure out how to build swig-matlab on windows
  if (SWIG_HAS_MATLAB_SUPPORT)
    message("Found Swig with Matlab support")
    include(SwigMatlab)

    add_swig_matlab_module(autodiffMatlab autodiffutils_matlab.i
                           CPLUSPLUS
                           INCLUDE_DIRS 
                           SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab ${DRAKE_SWIG_FILES}/common
                           DESTINATION ${matlab_install_subdir})

  	add_swig_matlab_module(rbtMatlab rbtree_matlab.i
  	                       CPLUSPLUS
  	                       INCLUDE_DIRS 
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../util
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../systems
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/joints 
                         LINK_LIBRARIES drakeRBM 
                         SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab ${DRAKE_SWIG_FILES}/common
                         DESTINATION ${matlab_install_subdir})

    if (snopt_c_FOUND)
      add_swig_matlab_module(ikMatlab ik_matlab.i
                             CPLUSPLUS
                             INCLUDE_DIRS 
                               ${CMAKE_CURRENT_SOURCE_DIR}/../../util
                               ${CMAKE_CURRENT_SOURCE_DIR}/../../systems
                               ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/constraint 
                               ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/joints 
                             LINK_LIBRARIES drakeIK 
                             SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab ${DRAKE_SWIG_FILES}/common
                             DESTINATION ${matlab_install_subdir})
    endif()
  else()
    message("Could NOT find Swig with Matlab support; not generating Matlab bindings")
  endif()
endif()
