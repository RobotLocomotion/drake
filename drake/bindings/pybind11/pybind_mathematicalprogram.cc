#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>


#include "drake/solvers/mathematical_program.h"

namespace py = pybind11;

using drake::Variable;
using drake::symbolic::Expression;
using drake::symbolic::Formula;

PYBIND11_MAKE_OPAQUE(drake::solvers::VectorXDecisionVariable);

PYBIND11_PLUGIN(_pybind_mathematicalprogram) {
  py::module m("_pybind_mathematicalprogram",
               "Drake MathematicalProgram Bindings");

  py::class_<drake::solvers::MathematicalProgram>(m, "MathematicalProgram")
    .def(py::init<>())
    .def("_NewContinuousVariables", (drake::solvers::VectorXDecisionVariable
          (drake::solvers::MathematicalProgram::*)(
          size_t,
          const std::string&))
         &drake::solvers::MathematicalProgram::NewContinuousVariables,
         py::arg("rows"),
         py::arg("name") = "x")
    .def("_NewBinaryVariables", (drake::solvers::VectorXDecisionVariable
         (drake::solvers::MathematicalProgram::*)(
         size_t,
         const std::string&))
         &drake::solvers::MathematicalProgram::NewBinaryVariables,
         py::arg("rows"),
         py::arg("name") = "x")
    .def("AddLinearConstraint",
         (drake::solvers::Binding<drake::solvers::LinearConstraint>
          (drake::solvers::MathematicalProgram::*)(
          const Expression&,
          double,
          double))
          &drake::solvers::MathematicalProgram::AddLinearConstraint)
    .def("AddLinearConstraint",
         (drake::solvers::Binding<drake::solvers::LinearConstraint>
          (drake::solvers::MathematicalProgram::*)(
          const Formula&))
          &drake::solvers::MathematicalProgram::AddLinearConstraint)
    .def("AddLinearCost",
         (drake::solvers::Binding<drake::solvers::LinearConstraint>
          (drake::solvers::MathematicalProgram::*)(
          const Expression&))
          &drake::solvers::MathematicalProgram::AddLinearCost)
    .def("Solve", &drake::solvers::MathematicalProgram::Solve)
    .def("_GetSolution", [](const drake::solvers::MathematicalProgram& prog,
                            const Variable& var) {
      return prog.GetSolution(var);
    })
    .def("_GetSolution",
         [](const drake::solvers::MathematicalProgram& prog,
            const drake::solvers::VectorXDecisionVariable& var) {
      return prog.GetSolution(var);
    });

  py::enum_<drake::solvers::SolutionResult>(m, "SolutionResult")
    .value("kSolutionFound", drake::solvers::SolutionResult::kSolutionFound)
    .value("kInvalidInput", drake::solvers::SolutionResult::kInvalidInput)
    .value("kInfeasibleConstraints",
           drake::solvers::SolutionResult::kInfeasibleConstraints)
    .value("kUnknownError", drake::solvers::SolutionResult::kUnknownError);

  py::class_<Variable>(m, "Variable")
    .def(py::init<const std::string&>())
    .def("__add__", [](const Variable& self,
                       const Variable& other) {
      return Expression{self + other};
    })
    .def("__add__", [](const Variable& self, double other) {
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
    .def("__le__", [](const Variable& self,
                      const Variable& other) {
      return Formula{Expression(self) <= Expression(other)};
    })
    .def("__le__", [](const Variable& self,
                      double other) {
      return Formula{Expression(self) <= Expression(other)};
    })
    .def("__gt__", [](const Variable& self,
                      const Variable& other) {
      return Formula{Expression(self) > Expression(other)};
    })
    .def("__gt__", [](const Variable& self,
                      double other) {
      return Formula{Expression(self) > Expression(other)};
    })
    .def("__ge__", [](const Variable& self,
                      const Variable& other) {
      return Formula{Expression(self) >= Expression(other)};
    })
    .def("__ge__", [](const Variable& self,
                      double other) {
      return Formula{Expression(self) >= Expression(other)};
    })
    .def("__eq__", [](const Variable& self,
                      const Variable& other) {
      return Formula{Expression(self) == Expression(other)};
    })
    .def("__eq__", [](const Variable& self,
                      double other) {
      return Formula{Expression(self) == Expression(other)};
    });


  py::class_<Expression>(m, "Expression")
    .def(py::init<>())
    .def(py::init<const Variable&>())
    .def(py::self   + py::self)
    .def(py::self   + Variable())
    .def(Variable() + py::self)
    .def(py::self   + double())
    .def(double()   + py::self)
    .def(py::self   - py::self)
    .def(py::self   - Variable())
    .def(Variable() - py::self)
    .def(py::self   - double())
    .def(double()   - py::self)
    .def(py::self   * py::self)
    .def(py::self   * Variable())
    .def(Variable() * py::self)
    .def(py::self   * double())
    .def(double()   * py::self)
    .def(py::self   / py::self)
    .def(py::self   / Variable())
    .def(Variable() / py::self)
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

  py::class_<Formula>(m, "Formula");

  py::class_<drake::solvers::VectorXDecisionVariable>(
    m, "_VectorXDecisionVariable")
    .def(py::init<size_t>())
    .def("__len__", [](const drake::solvers::VectorXDecisionVariable& v) {
      return v.size();
    })
    .def("__getitem__", [](const drake::solvers::VectorXDecisionVariable& v,
                           size_t i) {
      return v(i);
    }, py::return_value_policy::reference)
    .def("__setitem__", [](drake::solvers::VectorXDecisionVariable& v,
                           size_t i, const Variable &var) {
      v(i) = var;
    })
    ;

  py::class_<drake::solvers::LinearConstraint,
             std::shared_ptr<drake::solvers::LinearConstraint> >(
    m, "LinearConstraint");

  py::class_<drake::solvers::Binding<drake::solvers::LinearConstraint> >(
    m, "Binding_LinearConstraint");

  return m.ptr();
}
