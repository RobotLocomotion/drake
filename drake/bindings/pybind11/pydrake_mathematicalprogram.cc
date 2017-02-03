#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>


#include "drake/solvers/mathematical_program.h"

namespace py = pybind11;


PYBIND11_MAKE_OPAQUE(drake::solvers::VectorXDecisionVariable);

PYBIND11_PLUGIN(_pydrake_mathematicalprogram) {
  using drake::Variable;
  using drake::symbolic::Expression;
  using drake::symbolic::Formula;

  using drake::solvers::Binding;
  using drake::solvers::MathematicalProgram;
  using drake::solvers::Constraint;
  using drake::solvers::LinearConstraint;
  using drake::solvers::QuadraticConstraint;
  using drake::solvers::QuadraticConstraint;
  using drake::solvers::VectorXDecisionVariable;
  using drake::solvers::SolutionResult;

  py::module m("_pydrake_mathematicalprogram",
               "Drake MathematicalProgram Bindings");

  py::class_<MathematicalProgram>(m, "MathematicalProgram")
    .def(py::init<>())
    .def("_NewContinuousVariables", (VectorXDecisionVariable
          (MathematicalProgram::*)(
          size_t,
          const std::string&))
         &MathematicalProgram::NewContinuousVariables,
         py::arg("rows"),
         py::arg("name") = "x")
    .def("_NewBinaryVariables", (VectorXDecisionVariable
         (MathematicalProgram::*)(
         size_t,
         const std::string&))
         &MathematicalProgram::NewBinaryVariables,
         py::arg("rows"),
         py::arg("name") = "x")
    .def("AddLinearConstraint",
         (Binding<LinearConstraint>
          (MathematicalProgram::*)(
          const Expression&,
          double,
          double))
          &MathematicalProgram::AddLinearConstraint)
    .def("AddLinearConstraint",
         (Binding<LinearConstraint>
          (MathematicalProgram::*)(
          const Formula&))
          &MathematicalProgram::AddLinearConstraint)
    .def("AddLinearCost",
         (Binding<LinearConstraint>
          (MathematicalProgram::*)(
          const Expression&))
          &MathematicalProgram::AddLinearCost)
    .def("_AddQuadraticCost", (std::shared_ptr<QuadraticConstraint>
         (MathematicalProgram::*)(
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const VectorXDecisionVariable>&))
         &MathematicalProgram::AddQuadraticCost)
    .def("Solve", &MathematicalProgram::Solve)
    .def("linear_constraints", &MathematicalProgram::linear_constraints)
    .def("linear_equality_constraints", &MathematicalProgram::linear_equality_constraints)
    .def("linear_costs", &MathematicalProgram::linear_costs)
    .def("quadratic_costs", &MathematicalProgram::quadratic_costs)
    .def("FindDecisionVariableIndex", &MathematicalProgram::FindDecisionVariableIndex)
    .def("num_vars", &MathematicalProgram::num_vars)
    .def("_GetSolution", [](const MathematicalProgram& prog,
                            const Variable& var) {
      return prog.GetSolution(var);
    })
    .def("_GetSolution",
         [](const MathematicalProgram& prog,
            const VectorXDecisionVariable& var) {
      return prog.GetSolution(var);
    })
    .def("EvalBindingAtSolution", 
         (Eigen::VectorXd(MathematicalProgram::*)(
          const Binding<LinearConstraint>&) const) &MathematicalProgram::EvalBindingAtSolution)
    .def("EvalBindingAtSolution", 
         (Eigen::VectorXd(MathematicalProgram::*)(
          const Binding<QuadraticConstraint>&) const) &MathematicalProgram::EvalBindingAtSolution);

  py::enum_<SolutionResult>(m, "SolutionResult")
    .value("kSolutionFound", SolutionResult::kSolutionFound)
    .value("kInvalidInput", SolutionResult::kInvalidInput)
    .value("kInfeasibleConstraints",
           SolutionResult::kInfeasibleConstraints)
    .value("kUnknownError", SolutionResult::kUnknownError);

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
    .def("__repr__", &Expression::to_string)
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

  py::class_<Formula>(m, "Formula")
    .def("__repr__", &Formula::to_string);

  py::class_<VectorXDecisionVariable>(
    m, "_VectorXDecisionVariable")
    .def(py::init<size_t>())
    .def("__len__", [](const VectorXDecisionVariable& v) {
      return v.size();
    })
    .def("__getitem__", [](const VectorXDecisionVariable& v,
                           size_t i) {
      return v(i);
    }, py::return_value_policy::reference)
    .def("__setitem__", [](VectorXDecisionVariable& v,
                           size_t i, const Variable &var) {
      v(i) = var;
    });

  // Assign the wrapped Constraint class to the name 'constraint'
  // so we can use it in this file to indicate that the other constraint
  // types inherit from it.
  py::class_<Constraint> constraint(m, "Constraint");
  constraint.def("lower_bound", &Constraint::lower_bound)
            .def("upper_bound", &Constraint::upper_bound);

  py::class_<LinearConstraint, std::shared_ptr<LinearConstraint> >(
    m, "LinearConstraint", constraint)
    .def("A", &LinearConstraint::A);

  py::class_<QuadraticConstraint, std::shared_ptr<QuadraticConstraint> >(
    m, "QuadraticConstraint", constraint)
    .def("Q", &QuadraticConstraint::Q)
    .def("b", &QuadraticConstraint::b);

  py::class_<Binding<LinearConstraint> >(
    m, "Binding_LinearConstraint")
    .def("constraint", &Binding<LinearConstraint>::constraint)
    .def("_variables", &Binding<LinearConstraint>::variables);

  py::class_<Binding<QuadraticConstraint> >(
    m, "Binding_QuadraticConstraint")
    .def("constraint", &Binding<QuadraticConstraint>::constraint)
    .def("_variables", &Binding<QuadraticConstraint>::variables);

  return m.ptr();
}
