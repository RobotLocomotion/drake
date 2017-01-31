set(PYDRAKE_INSTALL_DIR lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/pydrake)

if(APPLE)
  string(REPLACE "-Wl,-undefined -Wl,error" "" CMAKE_MODULE_LINKER_FLAGS ${CMAKE_MODULE_LINKER_FLAGS})
else()
  string(REPLACE "-Wl,--no-undefined" "" CMAKE_MODULE_LINKER_FLAGS ${CMAKE_MODULE_LINKER_FLAGS})
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-shadow")
pybind11_add_module(_pydrake_rbtree pydrake_rbtree.cc)
target_link_libraries(_pydrake_rbtree PRIVATE drakeRBM drakeMultibodyParsers Eigen3::Eigen)
install(TARGETS _pydrake_rbtree DESTINATION ${PYDRAKE_INSTALL_DIR})

pybind11_add_module(_pydrake_autodiffutils pydrake_autodiffutils.cc)
target_link_libraries(_pydrake_autodiffutils PRIVATE drakeRBM drakeMultibodyParsers Eigen3::Eigen)
install(TARGETS _pydrake_autodiffutils DESTINATION ${PYDRAKE_INSTALL_DIR})

pybind11_add_module(_pydrake_ik pydrake_ik.cc)
target_link_libraries(_pydrake_ik PRIVATE drakeIK Eigen3::Eigen)
install(TARGETS _pydrake_ik DESTINATION ${PYDRAKE_INSTALL_DIR}/solvers)
