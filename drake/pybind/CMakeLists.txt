find_package(pybind11 REQUIRED)

pybind11_add_module(rbtree rbtree.cc)
target_link_libraries(rbtree PRIVATE drakeMultibodyParsers drakeRBM)
install(TARGETS rbtree DESTINATION lib/python/${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/pydrake)
