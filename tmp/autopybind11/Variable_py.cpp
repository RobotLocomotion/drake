#include "drake/common/symbolic.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

using namespace drake::symbolic;

namespace py = pybind11;

void apb11_pydrake_Variable_py_register(py::module &m) {
  py::class_<Variable> PyVariable(m, "Variable");

  py::enum_<Variable::Type>(PyVariable, "Type", py::arithmetic())
      .value("BINARY", Variable::Type::BINARY, "")
      .value("BOOLEAN", Variable::Type::BOOLEAN, "")
      .value("CONTINUOUS", Variable::Type::CONTINUOUS, "")
      .value("INTEGER", Variable::Type::INTEGER, "")
      .value("RANDOM_EXPONENTIAL", Variable::Type::RANDOM_EXPONENTIAL, "")
      .value("RANDOM_GAUSSIAN", Variable::Type::RANDOM_GAUSSIAN, "")
      .value("RANDOM_UNIFORM", Variable::Type::RANDOM_UNIFORM, "");
  PyVariable.def(py::init<Variable const &>(), py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<::std::nullptr_t>(), py::arg("arg0"))
      .def(py::init<::std::string, Variable::Type>(), py::arg("name"),
           py::arg("type") = Variable::Type(Variable::Type::CONTINUOUS))
      .def("equal_to",
           static_cast<bool (Variable::*)(Variable const &) const>(
               &Variable::equal_to),
           py::arg("v"))
      .def("get_id",
           static_cast<Variable::Id (Variable::*)() const>(&Variable::get_id))
      .def("get_name", static_cast<::std::string (Variable::*)() const>(
                           &Variable::get_name))
      .def("is_dummy",
           static_cast<bool (Variable::*)() const>(&Variable::is_dummy))
      .def("less",
           static_cast<bool (Variable::*)(Variable const &) const>(
               &Variable::less),
           py::arg("v"))
      .def("to_string", static_cast<::std::string (Variable::*)() const>(
                            &Variable::to_string))

      .def(
          "__str__", +[](Variable const &var) {
            std::ostringstream oss;
            oss << var;
            std::string s(oss.str());

            return s;
          });
}
