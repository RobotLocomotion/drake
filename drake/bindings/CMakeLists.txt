
set(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/share/cmake ${CMAKE_MODULE_PATH})
set(SWIG_EXECUTABLE ${CMAKE_INSTALL_PREFIX}/bin/swig)
set(DRAKE_SWIG_FILES ${CMAKE_INSTALL_PREFIX}/share/swig)
include(DrakeUseSWIG)
include(SwigPython)
include(SwigMatlab)

find_package(PythonInterp REQUIRED)
set(python_install_dir lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages)

add_swig_python_module(lcmPython lcm.i
                       CPLUSPLUS
                       LINK_LIBRARIES lcm
                       DESTINATION ${python_install_dir} ${CMAKE_CURRENT_SOURCE_DIR})

add_swig_python_module(systemsPython systems.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../examples/Pendulum 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/frames 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants/joints 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../util
                       LINK_LIBRARIES drakeSystem
                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python
                       DESTINATION ${python_install_dir} ${CMAKE_CURRENT_SOURCE_DIR})

add_swig_python_module(botvisPython botvis.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants
                       LINK_LIBRARIES drakeSystem
                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python
                       DESTINATION ${python_install_dir} ${CMAKE_CURRENT_SOURCE_DIR})

add_swig_python_module(rbmPython rbm.i
                       CPLUSPLUS
                       INCLUDE_DIRS 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
                       LINK_LIBRARIES drakeRBM 
                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python
                       DESTINATION ${python_install_dir} ${CMAKE_CURRENT_SOURCE_DIR})

add_swig_python_module(ikPython ik.i
                       CPLUSPLUS
                       INCLUDE_DIRS 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants/constraint 
                       LINK_LIBRARIES drakeRBM drakeRigidBodyConstraint drakeIK 
                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python
                       DESTINATION ${python_install_dir} ${CMAKE_CURRENT_SOURCE_DIR})

add_swig_python_module(pendulum pendulum.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../examples/Pendulum 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems/frames 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants/joints 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../util
                       LINK_LIBRARIES drakeSystem drakeGeometryUtil drakeRBM
                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/python
                       DESTINATION ${python_install_dir} ${CMAKE_CURRENT_SOURCE_DIR})

# if (MATLAB_FOUND)
# 	add_swig_matlab_module(MODULE lcm
# 	                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/lcm.i
# 	                       CPLUSPLUS
# 	                       LINK_LIBRARIES lcm
# 	                       DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/drake_matlab)


# 	add_swig_matlab_module(MODULE systems
# 	                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/systems.i
# 	                       CPLUSPLUS
# 	                       INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../examples/Pendulum 
# 	                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems 
# 	                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/frames 
# 	                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
# 	                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants/joints 
# 	                         ${CMAKE_CURRENT_SOURCE_DIR}/../util
# 	                       LINK_LIBRARIES drakeSystem
# 	                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab
# 	                       DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/drake_matlab)

# 	add_swig_matlab_module(MODULE botvis
# 	                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/botvis.i
# 	                       CPLUSPLUS
# 	                       INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants
# 	                       LINK_LIBRARIES drakeSystem
# 	                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab
# 	                       DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/drake_matlab)

# 	add_swig_matlab_module(MODULE rbm
# 	                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/rbm.i
# 	                       CPLUSPLUS
# 	                       INCLUDE_DIRS 
# 	                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
# 	                       LINK_LIBRARIES drakeRBM 
# 	                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab
# 	                       DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/drake_matlab)

# 	add_swig_matlab_module(MODULE ik
# 	                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/ik.i
# 	                       CPLUSPLUS
# 	                       INCLUDE_DIRS 
# 	                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
# 	                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants/constraint 
# 	                       LINK_LIBRARIES drakeRBM drakeRigidBodyConstraint drakeIK 
# 	                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab
# 	                       DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/drake_matlab)

# 	add_swig_matlab_module(MODULE pendulum
# 	                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/pendulum.i
# 	                       CPLUSPLUS
# 	                       INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../examples/Pendulum 
# 		                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems 
# 		                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems/frames 
# 		                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
# 		                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants/joints 
# 		                       ${CMAKE_CURRENT_SOURCE_DIR}/../util
# 	                       LINK_LIBRARIES drakeSystem drakeGeometryUtil drakeRBM
# 	                       SWIG_INCLUDE_DIRS ${DRAKE_SWIG_FILES}/matlab
# 	                       DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/drake_matlab)

# endif()
