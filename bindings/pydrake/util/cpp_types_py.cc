#include <pybind11/eval.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

namespace py = pybind11;

PYBIND11_MODULE(_cpp_types_py, m) {
  using drake::pydrake::internal::RegisterType;
  py::exec("import numpy as np");
  // Make mappings for C++ RTTI to Python types.
  // Unfortunately, this is hard to obtain from `pybind11`.
  RegisterType<bool>("bool");
  RegisterType<std::string>("str");
  RegisterType<double>("float");
  RegisterType<float>("np.float32");
  RegisterType<int>("int");
  RegisterType<uint32_t>("np.uint32");
  RegisterType<int64_t>("np.int64");
  // For supporting generic Python types.
  RegisterType<py::object>("object");
}
