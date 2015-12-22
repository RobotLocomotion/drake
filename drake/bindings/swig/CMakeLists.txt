set(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/share/cmake ${CMAKE_MODULE_PATH})
# set(SWIG_EXECUTABLE ${CMAKE_INSTALL_PREFIX}/bin/swig)
set(DRAKE_SWIG_FILES ${CMAKE_INSTALL_PREFIX}/share/swig)
include(DrakeUseSWIG)
include(SwigPython)

add_swig_python_module(lcmPython lcm.i
                       CPLUSPLUS
                       LINK_LIBRARIES lcm
                       DESTINATION ${python_install_subdir})

# add_swig_python_module(systemsPython systems.i
#                        CPLUSPLUS
#                        INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../examples/Pendulum 
#                          ${CMAKE_CURRENT_SOURCE_DIR}/../../systems 
#                          ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/frames 
#                          ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants 
#                          ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/joints 
#                          ${CMAKE_CURRENT_SOURCE_DIR}/../../util
#                        LINK_LIBRARIES drakeSystem
#                        SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python
#                        DESTINATION ${python_install_subdir})

# add_swig_python_module(botvisPython botvis.i
#                        CPLUSPLUS
#                        INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants
#                        LINK_LIBRARIES drakeSystem
#                        SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python
#                        DESTINATION ${python_install_subdir})

add_swig_python_module(autodiffPython autodiff_utils.i
                       CPLUSPLUS
                       INCLUDE_DIRS 
                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python ${DRAKE_SWIG_FILES}/common
                       DESTINATION ${python_install_subdir})

add_swig_python_module(rbmPython rbm.i
                       CPLUSPLUS
                       INCLUDE_DIRS 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../../util
                         ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/joints 
                       LINK_LIBRARIES drakeRBM 
                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python ${DRAKE_SWIG_FILES}/common
                       DESTINATION ${python_install_subdir})

if (snopt_c_FOUND)
  add_swig_python_module(ikPython ik.i
                         CPLUSPLUS
                         INCLUDE_DIRS 
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants 
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/constraint 
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../util
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/joints 
                         LINK_LIBRARIES drakeIK 
                         SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python
                         DESTINATION ${python_install_subdir})
endif()

# add_swig_python_module(pendulumPython pendulum.i
#                        CPLUSPLUS
#                        INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../examples/Pendulum 
# 	                       ${CMAKE_CURRENT_SOURCE_DIR}/../../systems 
# 	                       ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/frames 
# 	                       ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants 
# 	                       ${CMAKE_CURRENT_SOURCE_DIR}/../../systems/plants/joints 
# 	                       ${CMAKE_CURRENT_SOURCE_DIR}/../../util
#                        LINK_LIBRARIES drakeSystem drakeGeometryUtil drakeRBM
#                        SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python
#                        DESTINATION ${python_install_subdir})


# if (MATLAB_FOUND)
#   include(SwigMatlab)
# 	add_swig_matlab_module(rbmMatlab rbm.i
# 	                       CPLUSPLUS
# 	                       INCLUDE_DIRS 
# 	                         ${CMAKE_CURRENT_SOURCE_DIR}/..//../systems/plants 
# 	                       LINK_LIBRARIES drakeRBM 
# 	                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab ${DRAKE_SWIG_FILES}/common
# 	                       DESTINATION matlab)
# endif()
