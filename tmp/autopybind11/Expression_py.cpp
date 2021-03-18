#include "drake/common/symbolic.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

namespace py = pybind11;
void apb11_pydrake_Expression_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::symbolic;

  py::class_<Expression> PyExpression(m, "Expression");

  PyExpression.def(py::init<Expression const &>(), py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<double>(), py::arg("d"))
      .def(py::init<Variable const &>(), py::arg("var"))
      .def("Differentiate",
           static_cast<Expression (Expression::*)(Variable const &) const>(
               &Expression::Differentiate),
           py::arg("x"))
      .def_static("E", static_cast<Expression (*)()>(&Expression::E))
      .def("EqualTo",
           static_cast<bool (Expression::*)(Expression const &) const>(
               &Expression::EqualTo),
           py::arg("e"))
      .def("Evaluate",
           static_cast<double (Expression::*)(
               Environment const &, ::drake::RandomGenerator *) const>(
               &Expression::Evaluate),
           py::arg("env") = (Environment)Environment{},
           py::arg("random_generator") = (::drake::RandomGenerator *)nullptr)
      .def(
          "Evaluate",
          static_cast<double (Expression::*)(::drake::RandomGenerator *) const>(
              &Expression::Evaluate),
          py::arg("random_generator"))
      .def("EvaluatePartial",
           static_cast<Expression (Expression::*)(Environment const &) const>(
               &Expression::EvaluatePartial),
           py::arg("env"))
      .def("Expand",
           static_cast<Expression (Expression::*)() const>(&Expression::Expand))
      .def("GetVariables", static_cast<Variables (Expression::*)() const>(
                               &Expression::GetVariables))
      .def("Jacobian",
           [](Expression &self,
              ::Eigen::Ref<const Eigen::Matrix<Variable, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &vars) {
             return self.Jacobian(vars);
           })
      .def("Less",
           static_cast<bool (Expression::*)(Expression const &) const>(
               &Expression::Less),
           py::arg("e"))
      .def_static("NaN", static_cast<Expression (*)()>(&Expression::NaN))
      .def_static("One", static_cast<Expression (*)()>(&Expression::One))
      .def_static("Pi", static_cast<Expression (*)()>(&Expression::Pi))
      .def("Substitute",
           static_cast<Expression (Expression::*)(Variable const &,
                                                  Expression const &) const>(
               &Expression::Substitute),
           py::arg("var"), py::arg("e"))
      .def("Substitute",
           static_cast<Expression (Expression::*)(Substitution const &) const>(
               &Expression::Substitute),
           py::arg("s"))
      .def_static("Zero", static_cast<Expression (*)()>(&Expression::Zero))
      .def("get_kind", static_cast<ExpressionKind (Expression::*)() const>(
                           &Expression::get_kind))
      .def("is_expanded",
           static_cast<bool (Expression::*)() const>(&Expression::is_expanded))
      .def("is_polynomial", static_cast<bool (Expression::*)() const>(
                                &Expression::is_polynomial))
      .def("to_string", static_cast<::std::string (Expression::*)() const>(
                            &Expression::to_string))

      .def(
          "__mul__",
          +[](Expression lhs, Expression const &rhs) { return lhs * rhs; })
      .def(
          "__imul__",
          +[](Expression &lhs, Expression const &rhs) { return lhs *= rhs; })
      .def(
          "__add__",
          +[](Expression lhs, Expression const &rhs) { return lhs + rhs; })
      .def(
          "__add__", +[](Expression const &e) { return e; })
      .def("__add__",
           static_cast<Expression &(Expression::*)()>(&Expression::operator++))
      .def(
          "__add__",
          static_cast<Expression (Expression::*)(int)>(&Expression::operator++),
          py::arg("arg0"))
      .def(
          "__iadd__",
          +[](Expression &lhs, Expression const &rhs) { return lhs += rhs; })
      .def(
          "__sub__",
          +[](Expression lhs, Expression const &rhs) { return lhs - rhs; })
      .def(
          "__sub__", +[](Expression const &e) { return e; })
      .def("__sub__",
           static_cast<Expression &(Expression::*)()>(&Expression::operator--))
      .def(
          "__sub__",
          static_cast<Expression (Expression::*)(int)>(&Expression::operator--),
          py::arg("arg0"))
      .def(
          "__isub__",
          +[](Expression &lhs, Expression const &rhs) { return lhs -= rhs; })
      .def(
          "__truediv__",
          +[](Expression lhs, Expression const &rhs) { return lhs / rhs; })
      .def(
          "__itruediv__",
          +[](Expression &lhs, Expression const &rhs) { return lhs /= rhs; })
      .def(
          "__str__", +[](Expression const &e) {
            std::ostringstream oss;
            oss << e;
            std::string s(oss.str());

            return s;
          });
}
