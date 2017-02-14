#include "drake/bindings/pybind11/pydrake_symbolic_types.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>


namespace py = pybind11;

PYBIND11_PLUGIN(symbolic) {
  using drake::symbolic::Variable;
  using drake::symbolic::Expression;
  using drake::symbolic::Formula;

  py::module m("symbolic", "Symbolic variables, expressions, and formulae");

  py::class_<Variable>(m, "Variable")
    .def(py::init<const std::string&>())
    .def("__repr__", &Variable::to_string)
    .def("__add__", [](const Variable& self,
                       const Variable& other) {
      return Expression{self + other};
    })
    .def("__add__", [](const Variable& self, double other) {
      return Expression{self + other};
    })
    .def("__add__", [](const Variable& self, const Expression& other) {
      return Expression{self + other};
    })
    .def("__radd__", [](const Variable& self, double other) {
      return Expression{other + self};
    })
    .def("__sub__", [](const Variable& self,
                       const Variable& other) {
      return Expression{self - other};
    })
    .def("__sub__", [](const Variable& self, double other) {
      return Expression{self - other};
    })
    .def("__sub__", [](const Variable& self, const Expression& other) {
      return Expression{self - other};
    })
    .def("__rsub__", [](const Variable& self, double other) {
      return Expression{other - self};
    })
    .def("__mul__", [](const Variable& self,
                       const Variable& other) {
      return Expression{self * other};
    })
    .def("__mul__", [](const Variable& self, double other) {
      return Expression{self * other};
    })
    .def("__mul__", [](const Variable& self, const Expression& other) {
      return Expression{self * other};
    })
    .def("__rmul__", [](const Variable& self, double other) {
      return Expression{other * self};
    })
    .def("__truediv__", [](const Variable& self,
                           const Variable& other) {
      return Expression{self / other};
    })
    .def("__truediv__", [](const Variable& self,
                           double other) {
      return Expression{self / other};
    })
    .def("__truediv__", [](const Variable& self, const Expression& other) {
      return Expression{self / other};
    })
    .def("__rtruediv__", [](const Variable& self,
                            double other) {
      return Expression{other / self};
    })
    .def("__lt__", [](const Variable& self,
                      const Variable& other) {
      return Formula{Expression(self) < Expression(other)};
    })
    .def("__lt__", [](const Variable& self,
                      double other) {
      return Formula{Expression(self) < Expression(other)};
    })
    .def("__lt__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) < other};
    })
    .def("__le__", [](const Variable& self,
                      const Variable& other) {
      return Formula{Expression(self) <= Expression(other)};
    })
    .def("__le__", [](const Variable& self,
                      double other) {
      return Formula{Expression(self) <= Expression(other)};
    })
    .def("__le__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) <= other};
    })
    .def("__gt__", [](const Variable& self,
                      const Variable& other) {
      return Formula{Expression(self) > Expression(other)};
    })
    .def("__gt__", [](const Variable& self,
                      double other) {
      return Formula{Expression(self) > Expression(other)};
    })
    .def("__gt__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) > other};
    })
    .def("__ge__", [](const Variable& self,
                      const Variable& other) {
      return Formula{Expression(self) >= Expression(other)};
    })
    .def("__ge__", [](const Variable& self,
                      double other) {
      return Formula{Expression(self) >= Expression(other)};
    })
    .def("__ge__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) >= other};
    })
    .def("__eq__", [](const Variable& self,
                      const Variable& other) {
      return Formula{Expression(self) == Expression(other)};
    })
    .def("__eq__", [](const Variable& self,
                      double other) {
      return Formula{Expression(self) == Expression(other)};
    })
    .def("__eq__", [](const Variable& self, const Expression& other) {
      return Formula{Expression(self) == other};
    });


  py::class_<Expression>(m, "Expression")
    .def(py::init<>())
    .def(py::init<const Variable&>())
    .def("__repr__", &Expression::to_string)
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
    .def("__lt__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) < Expression(other)};
    })
    .def("__lt__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) < Expression(other)};
    })
    .def("__lt__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) < Expression(other)};
    })
    .def("__le__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) <= Expression(other)};
    })
    .def("__le__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) <= Expression(other)};
    })
    .def("__le__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) <= Expression(other)};
    })
    .def("__gt__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) > Expression(other)};
    })
    .def("__gt__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) > Expression(other)};
    })
    .def("__gt__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) > Expression(other)};
    })
    .def("__ge__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) >= Expression(other)};
    })
    .def("__ge__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) >= Expression(other)};
    })
    .def("__ge__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) >= Expression(other)};
    })
    .def("__eq__", [](const Expression& self,
                      const Variable& other) {
      return Formula{Expression(self) == Expression(other)};
    })
    .def("__eq__", [](const Expression& self,
                      const Expression& other) {
      return Formula{Expression(self) == Expression(other)};
    })
    .def("__eq__", [](const Expression& self,
                      double other) {
      return Formula{Expression(self) == Expression(other)};
    });

  py::class_<Formula>(m, "Formula")
    .def("__repr__", &Formula::to_string);

  return m.ptr();
}
