set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../swigmake/cmake ${CMAKE_MODULE_PATH})
include(SwigPython)

find_package(PythonInterp REQUIRED)
set(python_install_dir lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages)

add_swig_python_module(TARGET systems_wrapper
                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/systems.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../examples/Pendulum 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/frames 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants/joints 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../util
                       LINK_LIBRARIES drakeSystem
                       SWIG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../swigmake/swig
                       DESTINATION ${python_install_dir}/drake ${CMAKE_CURRENT_SOURCE_DIR}/drake)

add_swig_python_module(TARGET rbm_wrapper
                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/rbm.i
                       CPLUSPLUS
                       INCLUDE_DIRS 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
                       LINK_LIBRARIES drakeRBM 
                       SWIG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../swigmake/swig
                       DESTINATION ${python_install_dir}/drake ${CMAKE_CURRENT_SOURCE_DIR}/drake)

add_swig_python_module(TARGET ik_wrapper
                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/ik.i
                       CPLUSPLUS
                       INCLUDE_DIRS 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
                         ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants/constraint 
                       LINK_LIBRARIES drakeRBM drakeRigidBodyConstraint drakeIK 
                       SWIG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../swigmake/swig
                       DESTINATION ${python_install_dir}/drake ${CMAKE_CURRENT_SOURCE_DIR}/drake)

add_swig_python_module(TARGET pendulum_wrapper
                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/pendulum.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../examples/Pendulum 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems/frames 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../systems/plants/joints 
	                       ${CMAKE_CURRENT_SOURCE_DIR}/../util
                       LINK_LIBRARIES drakeSystem drakeGeometryUtil drakeRBM
                       SWIG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../swigmake/swig
                       DESTINATION ${python_install_dir}/drake ${CMAKE_CURRENT_SOURCE_DIR}/drake)

