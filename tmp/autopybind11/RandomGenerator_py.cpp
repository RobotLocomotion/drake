#include "drake/common/random.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake;

namespace py = pybind11;
void apb11_pydrake_RandomGenerator_py_register(py::module &m) {
  py::class_<RandomGenerator> PyRandomGenerator(m, "RandomGenerator");

  PyRandomGenerator.def(py::init<RandomGenerator const &>(), py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<RandomGenerator::result_type>(), py::arg("value"))
      .def_static("max", static_cast<RandomGenerator::result_type (*)()>(
                             &RandomGenerator::max))
      .def_static("min", static_cast<RandomGenerator::result_type (*)()>(
                             &RandomGenerator::min))
      .def_readonly_static("default_seed", &RandomGenerator::default_seed)
      .def("__call__",
           static_cast<RandomGenerator::result_type (RandomGenerator::*)()>(
               &RandomGenerator::operator()));
}
