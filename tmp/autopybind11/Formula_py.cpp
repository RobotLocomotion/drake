#include "drake/common/symbolic.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

namespace py = pybind11;
void apb11_pydrake_Formula_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::symbolic;

  py::class_<Formula> PyFormula(m, "Formula");

  PyFormula.def(py::init<Formula const &>(), py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<bool>(), py::arg("value"))
      .def(py::init<::std::shared_ptr<const FormulaCell>>(), py::arg("ptr"))
      .def(py::init<Variable const &>(), py::arg("var"))
      .def("EqualTo",
           static_cast<bool (Formula::*)(Formula const &) const>(
               &Formula::EqualTo),
           py::arg("f"))
      .def("Evaluate",
           static_cast<bool (Formula::*)(Environment const &,
                                         ::drake::RandomGenerator *) const>(
               &Formula::Evaluate),
           py::arg("env") = (Environment)Environment{},
           py::arg("random_generator") = (::drake::RandomGenerator *)nullptr)
      .def("Evaluate",
           static_cast<bool (Formula::*)(::drake::RandomGenerator *) const>(
               &Formula::Evaluate),
           py::arg("random_generator"))
      .def_static("False", static_cast<Formula (*)()>(&Formula::False))
      .def("GetFreeVariables", static_cast<Variables (Formula::*)() const>(
                                   &Formula::GetFreeVariables))
      .def(
          "Less",
          static_cast<bool (Formula::*)(Formula const &) const>(&Formula::Less),
          py::arg("f"))
      .def(
          "Substitute",
          static_cast<Formula (Formula::*)(Variable const &, Expression const &)
                          const>(&Formula::Substitute),
          py::arg("var"), py::arg("e"))
      .def("Substitute",
           static_cast<Formula (Formula::*)(Substitution const &) const>(
               &Formula::Substitute),
           py::arg("s"))
      .def_static("True", static_cast<Formula (*)()>(&Formula::True))
      .def("get_kind",
           static_cast<FormulaKind (Formula::*)() const>(&Formula::get_kind))
      .def("to_string",
           static_cast<::std::string (Formula::*)() const>(&Formula::to_string))

      .def("__bool__",
           static_cast<bool (Formula::*)() const>(&Formula::operator bool))
      .def(
          "__str__", +[](Formula const &f) {
            std::ostringstream oss;
            oss << f;
            std::string s(oss.str());

            return s;
          });
}
