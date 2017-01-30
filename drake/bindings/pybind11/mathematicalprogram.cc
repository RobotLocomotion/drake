#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>


#include "drake/solvers/mathematical_program.h"

namespace py = pybind11;


PYBIND11_MAKE_OPAQUE(drake::solvers::VectorXDecisionVariable);

PYBIND11_PLUGIN(_mathematicalprogram) {
  py::module m("_mathematicalprogram", "Drake MathematicalProgram Bindings");

  py::class_<drake::solvers::MathematicalProgram>(m, "MathematicalProgram")
    .def(py::init<>())
    .def("_NewContinuousVariables", (drake::solvers::VectorXDecisionVariable (drake::solvers::MathematicalProgram::*)(
            size_t,
            const std::string&)) &drake::solvers::MathematicalProgram::NewContinuousVariables,
         py::arg("rows"),
         py::arg("name") = "x");

  py::class_<drake::symbolic::Variable>(m, "Variable")
    .def(py::init<const std::string&>());

  py::class_<drake::solvers::VectorXDecisionVariable>(m, "VectorXDecisionVariable")
    .def("__len__", [](const drake::solvers::VectorXDecisionVariable& v) {
      return v.size();
    })
    .def("__getitem__", [](const drake::solvers::VectorXDecisionVariable& v, size_t i) {
      return v(i);
    }, py::return_value_policy::reference);


  return m.ptr();
}
