set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../swigmake/cmake ${CMAKE_MODULE_PATH})
include(SwigPython)

find_package(PythonInterp REQUIRED)
set(python_install_dir lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages)

add_swig_python_module(TARGET drake_wrapper
                       SWIG_I_FILE ${CMAKE_CURRENT_SOURCE_DIR}/drake.i
                       CPLUSPLUS
                       INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../examples/Pendulum ${CMAKE_CURRENT_SOURCE_DIR}/../systems ${CMAKE_CURRENT_SOURCE_DIR}/../systems/frames 
                       LINK_LIBRARIES drakeSystem
                       SWIG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../swigmake/swig
                       DESTINATION ${python_install_dir}/drake ${CMAKE_CURRENT_SOURCE_DIR}/drake)