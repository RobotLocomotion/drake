
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-shadow")
pybind11_add_module(_rbtree rbtree.cc)
target_link_libraries(_rbtree PRIVATE drakeRBM drakeMultibodyParsers Eigen3::Eigen)
install(TARGETS _rbtree DESTINATION lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/pydrake)
