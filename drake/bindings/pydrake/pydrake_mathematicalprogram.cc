#include <cstddef>
#include <memory>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/pydrake_symbolic_types.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_type_converter.h"

namespace py = pybind11;
using std::string;

using drake::solvers::Binding;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::Constraint;
using drake::solvers::Cost;
using drake::solvers::EvaluatorBase;
using drake::solvers::LinearConstraint;
using drake::solvers::LinearCost;
using drake::solvers::LinearEqualityConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramSolverInterface;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::QuadraticCost;
using drake::solvers::SolutionResult;
using drake::solvers::SolverId;
using drake::solvers::SolverType;
using drake::solvers::SolverTypeConverter;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Variable;

namespace {
// Unwrap an optional<T> for more idiomatic use in Python.  A nullopt in C++
// becomes None in Python, and non-nullopt in C++ becomes T directly in Python.
template <typename T>
std::unique_ptr<T> deref_optional(const drake::optional<T>& value) {
  return value ? std::make_unique<T>(*value) : nullptr;
}

/// Helper to adapt SolverType to SolverId.
template <typename Value>
void SetSolverOptionBySolverType(
    MathematicalProgram* self,
    SolverType solver_type, const std::string& key, const Value& value) {
  self->SetSolverOption(SolverTypeConverter::TypeToId(solver_type), key, value);
}

/*
 * Register a Binding template, and add the corresponding overloads to the
 * pybind11 MathematicalProgram class.
 * @param scope The scope this will be added to (e.g., the module).
 * @param pprog_cls Pointer to Python MathematicalProgram class. Overloads will
 * be added to the binding
 * @param name Name of the Cost / Constraint class.
 */
template <typename C>
auto RegisterBinding(py::handle* pscope,
                     py::class_<MathematicalProgram>* pprog_cls,
                     const string& name) {
  auto& scope = *pscope;
  auto& prog_cls = *pprog_cls;
  typedef Binding<C> B;
  string pyname = "Binding_" + name;
  auto binding_cls = py::class_<B>(scope, pyname.c_str())
    .def("constraint", &B::constraint)
    .def("variables", &B::variables);
  // Register overloads for MathematicalProgram class
  prog_cls
    .def("EvalBindingAtSolution",
         static_cast<Eigen::VectorXd(MathematicalProgram::*)(const B&) const>(
             &MathematicalProgram::EvalBindingAtSolution));
  return binding_cls;
}

}  // namespace

PYBIND11_MODULE(_pydrake_mathematicalprogram, m) {
  m.doc() = "Drake MathematicalProgram Bindings";

  py::object variable =
    py::module::import("pydrake.symbolic").attr("Variable");
  py::object expression =
    py::module::import("pydrake.symbolic").attr("Expression");
  py::object formula =
    py::module::import("pydrake.symbolic").attr("Formula");

  py::class_<MathematicalProgramSolverInterface>(
    m, "MathematicalProgramSolverInterface")
    .def("available", &MathematicalProgramSolverInterface::available)
    .def("solver_id", &MathematicalProgramSolverInterface::solver_id)
    .def("Solve", &MathematicalProgramSolverInterface::Solve)
    .def("solver_type", [](const MathematicalProgramSolverInterface& self) {
        return deref_optional(SolverTypeConverter::IdToType(self.solver_id()));
    })
    .def("SolverName", [](const MathematicalProgramSolverInterface& self) {
        return self.solver_id().name();
    });

  py::class_<SolverId>(m, "SolverId")
    .def("name", &SolverId::name);

  py::enum_<SolverType>(m, "SolverType")
    .value("kDReal", SolverType::kDReal)
    .value("kEqualityConstrainedQP", SolverType::kEqualityConstrainedQP)
    .value("kGurobi", SolverType::kGurobi)
    .value("kIpopt", SolverType::kIpopt)
    .value("kLinearSystem", SolverType::kLinearSystem)
    .value("kMobyLCP", SolverType::kMobyLCP)
    .value("kMosek", SolverType::kMosek)
    .value("kNlopt", SolverType::kNlopt)
    .value("kSnopt", SolverType::kSnopt);

  py::class_<MathematicalProgram> prog_cls(m, "MathematicalProgram");
  prog_cls
    .def(py::init<>())
    .def("NewContinuousVariables",
         static_cast<VectorXDecisionVariable
         (MathematicalProgram::*)(
             int,
             const std::string&)
         >(&MathematicalProgram::NewContinuousVariables),
         py::arg("rows"),
         py::arg("name") = "x")
    .def("NewContinuousVariables",
         static_cast<MatrixXDecisionVariable
         (MathematicalProgram::*)(
             int,
             int,
             const std::string&)
         >(&MathematicalProgram::NewContinuousVariables),
         py::arg("rows"),
         py::arg("cols"),
         py::arg("name") = "x")
    .def("NewBinaryVariables",
         static_cast<VectorXDecisionVariable
         (MathematicalProgram::*)(
             int,
             const std::string&)
         >(&MathematicalProgram::NewBinaryVariables),
         py::arg("rows"),
         py::arg("name") = "b")
    .def("NewBinaryVariables",
         static_cast<MatrixXDecisionVariable
         (MathematicalProgram::*)(
             int,
             int,
             const std::string&)
         >(&MathematicalProgram::NewBinaryVariables),
         py::arg("rows"),
         py::arg("cols"),
         py::arg("name") = "b")
    .def("AddLinearConstraint",
         static_cast<Binding<LinearConstraint>
         (MathematicalProgram::*)(
             const Expression&,
             double,
             double)
         >(&MathematicalProgram::AddLinearConstraint))
    .def("AddLinearConstraint",
         static_cast<Binding<LinearConstraint>
         (MathematicalProgram::*)(
             const Formula&)
         >(&MathematicalProgram::AddLinearConstraint))
    .def("AddLinearCost",
         static_cast<Binding<LinearCost>
         (MathematicalProgram::*)(
          const Expression&)
         >(&MathematicalProgram::AddLinearCost))
    .def("AddQuadraticCost",
         static_cast<Binding<QuadraticCost>
         (MathematicalProgram::*)(
             const Eigen::Ref<const Eigen::MatrixXd>&,
             const Eigen::Ref<const Eigen::VectorXd>&,
             const Eigen::Ref<const VectorXDecisionVariable>&)
         >(&MathematicalProgram::AddQuadraticCost))
    .def("AddQuadraticCost",
         static_cast<Binding<QuadraticCost>
         (MathematicalProgram::*)(
             const Expression&)
         >(&MathematicalProgram::AddQuadraticCost))
    .def("Solve", &MathematicalProgram::Solve)
    .def("GetSolverId", [](const MathematicalProgram& prog) {
        return deref_optional(prog.GetSolverId());
    })
    .def("linear_constraints", &MathematicalProgram::linear_constraints)
    .def("linear_equality_constraints",
         &MathematicalProgram::linear_equality_constraints)
    .def("bounding_box_constraints",
         &MathematicalProgram::bounding_box_constraints)
    .def("linear_costs", &MathematicalProgram::linear_costs)
    .def("quadratic_costs", &MathematicalProgram::quadratic_costs)
    .def("FindDecisionVariableIndex",
         &MathematicalProgram::FindDecisionVariableIndex)
    .def("num_vars", &MathematicalProgram::num_vars)
    .def("GetSolution", [](const MathematicalProgram& prog,
                            const Variable& var) {
      return prog.GetSolution(var);
    })
    .def("GetSolution",
         [](const MathematicalProgram& prog,
            const VectorXDecisionVariable& var) {
      return prog.GetSolution(var);
    })
    .def("GetSolution",
         [](const MathematicalProgram& prog,
            const MatrixXDecisionVariable& var) {
      return prog.GetSolution(var);
    })
    .def("SetSolverOption", &SetSolverOptionBySolverType<double>)
    .def("SetSolverOption", &SetSolverOptionBySolverType<int>)
    .def("SetSolverOption", &SetSolverOptionBySolverType<string>);

  py::enum_<SolutionResult>(m, "SolutionResult")
    .value("kSolutionFound", SolutionResult::kSolutionFound)
    .value("kInvalidInput", SolutionResult::kInvalidInput)
    .value("kInfeasibleConstraints",
           SolutionResult::kInfeasibleConstraints)
    .value("kUnbounded", SolutionResult::kUnbounded)
    .value("kUnknownError", SolutionResult::kUnknownError)
    .value("kInfeasible_Or_Unbounded",
           SolutionResult::kInfeasible_Or_Unbounded);

  // TODO(eric.cousineau): Expose Eval() in a Python-friendly fashion.
  py::class_<EvaluatorBase, std::shared_ptr<EvaluatorBase>>(m, "EvaluatorBase")
    .def("num_outputs", &EvaluatorBase::num_outputs)
    .def("num_vars", &EvaluatorBase::num_vars);

  py::class_<Constraint, EvaluatorBase, std::shared_ptr<Constraint>>(
    m, "Constraint")
    .def("num_constraints", &Constraint::num_constraints)
    .def("lower_bound", &Constraint::lower_bound)
    .def("upper_bound", &Constraint::upper_bound);

  py::class_<LinearConstraint, Constraint, std::shared_ptr<LinearConstraint>>(
    m, "LinearConstraint")
    .def("A", &LinearConstraint::A);

  py::class_<LinearEqualityConstraint, LinearConstraint,
             std::shared_ptr<LinearEqualityConstraint>>(
    m, "LinearEqualityConstraint");

  py::class_<BoundingBoxConstraint, LinearConstraint,
             std::shared_ptr<BoundingBoxConstraint>>(
    m, "BoundingBoxConstraint");

  RegisterBinding<LinearConstraint>(&m, &prog_cls, "LinearConstraint");
  RegisterBinding<LinearEqualityConstraint>(&m, &prog_cls,
                                            "LinearEqualityConstraint");
  RegisterBinding<BoundingBoxConstraint>(&m, &prog_cls,
                                         "BoundingBoxConstraint");

  // Mirror procedure for costs
  py::class_<Cost, std::shared_ptr<Cost>> cost(
    m, "Cost");

  py::class_<LinearCost, Cost, std::shared_ptr<LinearCost>>(
    m, "LinearCost")
    .def("a", &LinearCost::a)
    .def("b", &LinearCost::b);

  py::class_<QuadraticCost, Cost,
             std::shared_ptr<QuadraticCost>>(m, "QuadraticCost")
    .def("Q", &QuadraticCost::Q)
    .def("b", &QuadraticCost::b)
    .def("c", &QuadraticCost::c);

  RegisterBinding<LinearCost>(&m, &prog_cls, "LinearCost");
  RegisterBinding<QuadraticCost>(&m, &prog_cls, "QuadraticCost");
}
