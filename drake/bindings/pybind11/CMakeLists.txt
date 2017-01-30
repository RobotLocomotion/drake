set(PYDRAKE_INSTALL_DIR lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/pydrake)

string(REPLACE "-Wl,--no-undefined" "" CMAKE_MODULE_LINKER_FLAGS ${CMAKE_MODULE_LINKER_FLAGS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-shadow")
pybind11_add_module(_rbtree rbtree.cc)
target_link_libraries(_rbtree PRIVATE drakeRBM drakeMultibodyParsers Eigen3::Eigen)
install(TARGETS _rbtree DESTINATION ${PYDRAKE_INSTALL_DIR})

pybind11_add_module(_autodiffutils autodiffutils.cc)
target_link_libraries(_autodiffutils PRIVATE drakeRBM drakeMultibodyParsers Eigen3::Eigen)
install(TARGETS _autodiffutils DESTINATION ${PYDRAKE_INSTALL_DIR})

pybind11_add_module(_ik ik.cc)
target_link_libraries(_ik PRIVATE drakeIK Eigen3::Eigen)
install(TARGETS _ik DESTINATION ${PYDRAKE_INSTALL_DIR}/solvers)
