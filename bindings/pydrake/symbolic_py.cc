#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(_symbolic_py, m) {
  using drake::symbolic::Variable;
  using drake::symbolic::Expression;
  using drake::symbolic::Formula;
  using drake::symbolic::pow;

  m.doc() = "Symbolic variables, expressions, and formulae";

  py::class_<Variable>(m, "Variable")
    .def(py::init<const std::string&>())
    .def("get_id", &Variable::get_id)
    .def("__repr__", &Variable::to_string)
    .def("__add__", [](const Variable& self, const Variable& other) {
      return Expression{self + other};
    }, py::is_operator())
    .def("__add__", [](const Variable& self, double other) {
      return Expression{self + other};
    }, py::is_operator())
    .def("__add__", [](const Variable& self, const Expression& other) {
      return Expression{self + other};
    }, py::is_operator())
    .def("__radd__", [](const Variable& self, double other) {
      return Expression{other + self};
    }, py::is_operator())
    .def("__sub__", [](const Variable& self, const Variable& other) {
      return Expression{self - other};
    }, py::is_operator())
    .def("__sub__", [](const Variable& self, double other) {
      return Expression{self - other};
    }, py::is_operator())
    .def("__sub__", [](const Variable& self, const Expression& other) {
      return Expression{self - other};
    }, py::is_operator())
    .def("__rsub__", [](const Variable& self, double other) {
      return Expression{other - self};
    }, py::is_operator())
    .def("__mul__", [](const Variable& self, const Variable& other) {
      return Expression{self * other};
    }, py::is_operator())
    .def("__mul__", [](const Variable& self, double other) {
      return Expression{self * other};
    }, py::is_operator())
    .def("__mul__", [](const Variable& self, const Expression& other) {
      return Expression{self * other};
    }, py::is_operator())
    .def("__rmul__", [](const Variable& self, double other) {
      return Expression{other * self};
    }, py::is_operator())
    .def("__truediv__", [](const Variable& self, const Variable& other) {
      return Expression{self / other};
    }, py::is_operator())
    .def("__truediv__", [](const Variable& self, double other) {
      return Expression{self / other};
    }, py::is_operator())
    .def("__truediv__", [](const Variable& self, const Expression& other) {
      return Expression{self / other};
    }, py::is_operator())
    .def("__rtruediv__", [](const Variable& self, double other) {
      return Expression{other / self};
    }, py::is_operator())
    .def("__pow__", [](const Variable& self, int other) {
      return Expression{pow(self, other)};
    }, py::is_operator())
    .def("__pow__", [](const Variable& self, double other) {
      return Expression{pow(self, other)};
    }, py::is_operator())
    .def("__pow__", [](const Variable& self, const Variable& other) {
      return Expression{pow(self, other)};
    }, py::is_operator())
    .def("__pow__", [](const Variable& self, const Expression& other) {
      return Expression{pow(self, other)};
    }, py::is_operator())
    .def("__neg__", [](const Variable& self) {
      return Expression{-self};
    }, py::is_operator())
    .def("__lt__", [](const Variable& self, const Variable& other) {
      return Formula{Expression(self) < Expression(other)};
    }, py::is_operator())
    .def("__lt__", [](const Variable& self, double other) {
      return Formula{Expression(self) < Expression(other)};
    }, py::is_operator())
    .def("__lt__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) < other};
    }, py::is_operator())
    .def("__le__", [](const Variable& self, const Variable& other) {
      return Formula{Expression(self) <= Expression(other)};
    }, py::is_operator())
    .def("__le__", [](const Variable& self, double other) {
      return Formula{Expression(self) <= Expression(other)};
    }, py::is_operator())
    .def("__le__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) <= other};
    }, py::is_operator())
    .def("__gt__", [](const Variable& self, const Variable& other) {
      return Formula{Expression(self) > Expression(other)};
    }, py::is_operator())
    .def("__gt__", [](const Variable& self, double other) {
      return Formula{Expression(self) > Expression(other)};
    }, py::is_operator())
    .def("__gt__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) > other};
    }, py::is_operator())
    .def("__ge__", [](const Variable& self, const Variable& other) {
      return Formula{Expression(self) >= Expression(other)};
    }, py::is_operator())
    .def("__ge__", [](const Variable& self, double other) {
      return Formula{Expression(self) >= Expression(other)};
    }, py::is_operator())
    .def("__ge__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) >= other};
    }, py::is_operator())
    .def("__eq__", [](const Variable& self, const Variable& other) {
      return Formula{Expression(self) == Expression(other)};
    }, py::is_operator())
    .def("__eq__", [](const Variable& self, double other) {
      return Formula{Expression(self) == Expression(other)};
    }, py::is_operator())
    .def("__eq__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) == other};
    }, py::is_operator());


  py::class_<Expression>(m, "Expression")
    .def(py::init<>())
    .def(py::init<const Variable&>())
    .def("__repr__", &Expression::to_string)
    .def("Expand", &Expression::Expand)
    .def(py::self   + py::self)
    .def(py::self   + Variable())
    .def(py::self   + double())
    .def(double()   + py::self)
    .def(py::self   - py::self)
    .def(py::self   - Variable())
    .def(py::self   - double())
    .def(double()   - py::self)
    .def(py::self   * py::self)
    .def(py::self   * Variable())
    .def(py::self   * double())
    .def(double()   * py::self)
    .def(py::self   / py::self)
    .def(py::self   / Variable())
    .def(py::self   / double())
    .def(double()   / py::self)
    .def("__pow__", [](const Expression& self, int other) {
      return pow(self, other);
    }, py::is_operator())
    .def("__pow__", [](const Expression& self, double other) {
      return pow(self, other);
    }, py::is_operator())
    .def("__pow__", [](const Expression& self, const Variable& other) {
      return pow(self, other);
    }, py::is_operator())
    .def("__pow__", [](const Expression& self, const Expression& other) {
      return pow(self, other);
    }, py::is_operator())
    .def("__neg__", [](const Expression& self) {
      return -self;
    }, py::is_operator())
    .def("__lt__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) < Expression(other)};
    }, py::is_operator())
    .def("__lt__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) < Expression(other)};
    }, py::is_operator())
    .def("__lt__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) < Expression(other)};
    }, py::is_operator())
    .def("__le__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) <= Expression(other)};
    }, py::is_operator())
    .def("__le__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) <= Expression(other)};
    }, py::is_operator())
    .def("__le__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) <= Expression(other)};
    }, py::is_operator())
    .def("__gt__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) > Expression(other)};
    }, py::is_operator())
    .def("__gt__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) > Expression(other)};
    }, py::is_operator())
    .def("__gt__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) > Expression(other)};
    }, py::is_operator())
    .def("__ge__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) >= Expression(other)};
    }, py::is_operator())
    .def("__ge__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) >= Expression(other)};
    }, py::is_operator())
    .def("__ge__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) >= Expression(other)};
    }, py::is_operator())
    .def("__eq__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) == Expression(other)};
    }, py::is_operator())
    .def("__eq__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) == Expression(other)};
    }, py::is_operator())
    .def("__eq__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) == Expression(other)};
    }, py::is_operator());

  py::class_<Formula>(m, "Formula")
    .def("__repr__", &Formula::to_string);

  // Cannot overload logical operators: http://stackoverflow.com/a/471561
  // Defining custom function for clarity.
  // Could use bitwise operators:
  // https://docs.python.org/2/library/operator.html#operator.__and__
  // However, this may reduce clarity and introduces constraints on order of
  // operations.
  m
    // Hide AND and OR to permit us to make it accept 1 or more arguments in
    // Python (and not have to handle type safety within C++).
    .def("__logical_and", [](const Formula& a,
                             const Formula& b) {
      return a && b;
    })
    .def("__logical_or", [](const Formula& a,
                            const Formula& b) {
      return a || b;
    })
    .def("logical_not", [](const Formula& a) {
          return !a;
    });
}

}  // namespace pydrake
}  // namespace drake
