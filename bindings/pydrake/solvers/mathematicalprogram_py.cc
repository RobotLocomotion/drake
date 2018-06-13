#include <cstddef>
#include <memory>

#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_type_converter.h"

using Eigen::Dynamic;
using std::string;
using std::vector;

namespace drake {
namespace pydrake {

using solvers::Binding;
using solvers::BoundingBoxConstraint;
using solvers::Constraint;
using solvers::Cost;
using solvers::EvaluatorBase;
using solvers::LinearConstraint;
using solvers::LorentzConeConstraint;
using solvers::LinearCost;
using solvers::LinearComplementarityConstraint;
using solvers::LinearEqualityConstraint;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramSolverInterface;
using solvers::MatrixXDecisionVariable;
using solvers::MatrixXIndeterminate;
using solvers::PositiveSemidefiniteConstraint;
using solvers::QuadraticCost;
using solvers::SolutionResult;
using solvers::SolverId;
using solvers::SolverType;
using solvers::SolverTypeConverter;
using solvers::VariableRefList;
using solvers::VectorXDecisionVariable;
using solvers::VectorXIndeterminate;
using solvers::VisualizationCallback;
using symbolic::Expression;
using symbolic::Formula;
using symbolic::Monomial;
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;

namespace {

/// Helper to adapt SolverType to SolverId.
template <typename Value>
void SetSolverOptionBySolverType(MathematicalProgram* self,
                                 SolverType solver_type, const std::string& key,
                                 const Value& value) {
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
                         .def("evaluator", &B::evaluator)
                         .def("constraint", &B::evaluator)
                         .def("variables", &B::variables);
  // Deprecate `constraint`.
  py::module deprecation = py::module::import("pydrake.util.deprecation");
  py::object deprecated = deprecation.attr("deprecated");
  binding_cls.attr("constraint") =
      deprecated("`constraint` is deprecated; please use `evaluator` instead.")(
          binding_cls.attr("constraint"));
  // Register overloads for MathematicalProgram class
  prog_cls.def(
      "EvalBindingAtSolution",
      static_cast<Eigen::VectorXd (MathematicalProgram::*)(const B&) const>(
          &MathematicalProgram::EvalBindingAtSolution));
  return binding_cls;
}

class PyFunctionCost : public Cost {
 public:
  using DoubleFunc = std::function<double(const Eigen::VectorXd&)>;
  using AutoDiffFunc = std::function<AutoDiffXd(const VectorX<AutoDiffXd>&)>;

  PyFunctionCost(int num_vars, const py::function& func,
                 const std::string& description)
      : Cost(num_vars, description),
        double_func_(py::cast<DoubleFunc>(func)),
        autodiff_func_(py::cast<AutoDiffFunc>(func)) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    y[0] = double_func_(x);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    y[0] = autodiff_func_(x);
  }

 private:
  const DoubleFunc double_func_;
  const AutoDiffFunc autodiff_func_;
};

class PyFunctionConstraint : public Constraint {
 public:
  using DoubleFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
  using AutoDiffFunc =
      std::function<VectorX<AutoDiffXd>(const VectorX<AutoDiffXd>&)>;

  PyFunctionConstraint(int num_vars, const py::function& func,
                       const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                       const std::string& description)
      : Constraint(lb.size(), num_vars, lb, ub, description),
        double_func_(py::cast<DoubleFunc>(func)),
        autodiff_func_(py::cast<AutoDiffFunc>(func)) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    y = double_func_(x);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    y = autodiff_func_(x);
  }

 private:
  const DoubleFunc double_func_;
  const AutoDiffFunc autodiff_func_;
};

}  // namespace

PYBIND11_MODULE(_mathematicalprogram_py, m) {
  m.doc() = "Drake MathematicalProgram Bindings";

  py::module::import("pydrake.autodiffutils");
  py::object variable = py::module::import("pydrake.symbolic").attr("Variable");
  py::object variables =
      py::module::import("pydrake.symbolic").attr("Variables");
  py::object expression =
      py::module::import("pydrake.symbolic").attr("Expression");
  py::object formula = py::module::import("pydrake.symbolic").attr("Formula");

  py::class_<MathematicalProgramSolverInterface>(
      m, "MathematicalProgramSolverInterface")
      .def("available", &MathematicalProgramSolverInterface::available)
      .def("solver_id", &MathematicalProgramSolverInterface::solver_id)
      .def("Solve", &MathematicalProgramSolverInterface::Solve)
      .def("solver_type",
           [](const MathematicalProgramSolverInterface& self) {
             return SolverTypeConverter::IdToType(self.solver_id());
           })
      .def("SolverName", [](const MathematicalProgramSolverInterface& self) {
        return self.solver_id().name();
      });

  py::class_<SolverId>(m, "SolverId").def("name", &SolverId::name);

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
  prog_cls.def(py::init<>())
      .def("NewContinuousVariables",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<VectorXDecisionVariable (MathematicalProgram::*)(
               int, const std::string&)>(
               &MathematicalProgram::NewContinuousVariables),
           py::arg("rows"), py::arg("name") = "x")
      .def("NewContinuousVariables",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<MatrixXDecisionVariable (MathematicalProgram::*)(
               int, int, const std::string&)>(
               &MathematicalProgram::NewContinuousVariables),
           py::arg("rows"), py::arg("cols"), py::arg("name") = "x")
      .def("NewBinaryVariables",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<VectorXDecisionVariable (MathematicalProgram::*)(
               int, const std::string&)>(
               &MathematicalProgram::NewBinaryVariables),
           py::arg("rows"), py::arg("name") = "b")
      .def("NewBinaryVariables",
           py::overload_cast<int, int, const string&>(
               &MathematicalProgram::NewBinaryVariables<Dynamic, Dynamic>),
           py::arg("rows"), py::arg("cols"), py::arg("name") = "b")
      .def("NewSymmetricContinuousVariables",
           // `py::overload_cast` and `overload_cast_explict` struggle with
           // overloads that compete with templated methods.
           [](MathematicalProgram* self, int rows, const string& name) {
             return self->NewSymmetricContinuousVariables(rows, name);
           },
           py::arg("rows"), py::arg("name") = "Symmetric")
      .def("NewFreePolynomial", &MathematicalProgram::NewFreePolynomial,
           py::arg("indeterminates"), py::arg("deg"),
           py::arg("coeff_name") = "a")
      .def("NewSosPolynomial",
           static_cast<
               std::pair<Polynomial, Binding<PositiveSemidefiniteConstraint>> (
                   MathematicalProgram::*)(
                   const Eigen::Ref<const VectorX<Monomial>>&)>(
               &MathematicalProgram::NewSosPolynomial))
      .def("NewSosPolynomial",
           static_cast<
               std::pair<Polynomial, Binding<PositiveSemidefiniteConstraint>> (
                   MathematicalProgram::*)(const Variables&, int)>(
               &MathematicalProgram::NewSosPolynomial))
      .def("NewIndeterminates",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<VectorXIndeterminate (MathematicalProgram::*)(
               int, const std::string&)>(
               &MathematicalProgram::NewIndeterminates),
           py::arg("rows"), py::arg("name") = "x")
      .def("NewIndeterminates",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<MatrixXIndeterminate (MathematicalProgram::*)(
               int, int, const std::string&)>(
               &MathematicalProgram::NewIndeterminates),
           py::arg("rows"), py::arg("cols"), py::arg("name") = "X")
      .def("AddBoundingBoxConstraint",
           static_cast<Binding<BoundingBoxConstraint> (MathematicalProgram::*)(
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&)>(
               &MathematicalProgram::AddBoundingBoxConstraint))
      .def("AddBoundingBoxConstraint",
           static_cast<Binding<BoundingBoxConstraint> (MathematicalProgram::*)(
               double, double, const symbolic::Variable&)>(
               &MathematicalProgram::AddBoundingBoxConstraint))
      .def("AddBoundingBoxConstraint",
           [](MathematicalProgram* self, double lb, double ub,
              const Eigen::Ref<const MatrixX<symbolic::Variable>>& vars) {
             return self->AddBoundingBoxConstraint(lb, ub, vars);
           })
      .def("AddConstraint",
           [](MathematicalProgram* self, py::function func,
              const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
              const Eigen::Ref<const VectorXDecisionVariable>& vars,
              std::string& description) {
             return self->AddConstraint(
                 std::make_shared<PyFunctionConstraint>(vars.size(), func, lb,
                                                        ub, description),
                 vars);
           },
           py::arg("func"), py::arg("vars"), py::arg("lb"), py::arg("ub"),
           py::arg("description") = "")
      .def("AddConstraint",
           static_cast<Binding<Constraint> (MathematicalProgram::*)(
               const Expression&, double, double)>(
               &MathematicalProgram::AddConstraint))
      .def("AddConstraint",
           static_cast<Binding<Constraint> (MathematicalProgram::*)(
               const Formula&)>(&MathematicalProgram::AddConstraint))
      .def("AddLinearConstraint",
           static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
               const Expression&, double, double)>(
               &MathematicalProgram::AddLinearConstraint))
      .def("AddLinearConstraint",
           static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
               const Formula&)>(&MathematicalProgram::AddLinearConstraint))
      .def("AddLorentzConeConstraint",
           static_cast<Binding<LorentzConeConstraint> (MathematicalProgram::*)(
               const Eigen::Ref<const VectorX<drake::symbolic::Expression>>&)>(
               &MathematicalProgram::AddLorentzConeConstraint))
      .def("AddPositiveSemidefiniteConstraint",
           [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixXDecisionVariable>& vars) {
             return self->AddPositiveSemidefiniteConstraint(vars);
           })
      .def("AddLinearComplementarityConstraint",
           static_cast<Binding<LinearComplementarityConstraint> (
               MathematicalProgram::*)(
               const Eigen::Ref<const Eigen::MatrixXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&)>(
               &MathematicalProgram::AddLinearComplementarityConstraint))
      .def("AddPositiveSemidefiniteConstraint",
           [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixX<Expression>>& e) {
             return self->AddPositiveSemidefiniteConstraint(e);
           })
      .def("AddCost",
           [](MathematicalProgram* self, py::function func,
              const Eigen::Ref<const VectorXDecisionVariable>& vars,
              std::string& description) {
             return self->AddCost(std::make_shared<PyFunctionCost>(
                                      vars.size(), func, description),
                                  vars);
           },
           py::arg("func"), py::arg("vars"), py::arg("description") = "")
      .def("AddCost",
          static_cast<Binding<Cost> (MathematicalProgram::*)(
          const Expression&)>(&MathematicalProgram::AddCost))
      .def("AddLinearCost",
           static_cast<Binding<LinearCost> (MathematicalProgram::*)(
               const Expression&)>(&MathematicalProgram::AddLinearCost))
      .def("AddQuadraticCost",
           static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(
               const Eigen::Ref<const Eigen::MatrixXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&)>(
               &MathematicalProgram::AddQuadraticCost))
      .def("AddQuadraticCost",
           static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(
               const Expression&)>(&MathematicalProgram::AddQuadraticCost))
      .def("AddQuadraticErrorCost",
           overload_cast_explicit<
               Binding<QuadraticCost>, const Eigen::Ref<const Eigen::MatrixXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&>(
               &MathematicalProgram::AddQuadraticErrorCost),
           py::arg("Q"), py::arg("x_desired"), py::arg("vars"))
      .def("AddSosConstraint",
           static_cast<std::pair<Binding<PositiveSemidefiniteConstraint>,
                                 Binding<LinearEqualityConstraint>> (
               MathematicalProgram::*)(
               const Polynomial&, const Eigen::Ref<const VectorX<Monomial>>&)>(
               &MathematicalProgram::AddSosConstraint))
      .def("AddSosConstraint",
           static_cast<std::pair<Binding<PositiveSemidefiniteConstraint>,
                                 Binding<LinearEqualityConstraint>> (
               MathematicalProgram::*)(const Polynomial&)>(
               &MathematicalProgram::AddSosConstraint))
      .def("AddSosConstraint",
           static_cast<std::pair<Binding<PositiveSemidefiniteConstraint>,
                                 Binding<LinearEqualityConstraint>> (
               MathematicalProgram::*)(
               const Expression&, const Eigen::Ref<const VectorX<Monomial>>&)>(
               &MathematicalProgram::AddSosConstraint))
      .def("AddSosConstraint",
           static_cast<std::pair<Binding<PositiveSemidefiniteConstraint>,
                                 Binding<LinearEqualityConstraint>> (
               MathematicalProgram::*)(const Expression&)>(
               &MathematicalProgram::AddSosConstraint))
      .def("AddVisualizationCallback",
          static_cast<Binding<VisualizationCallback> (MathematicalProgram::*)(
              const VisualizationCallback::CallbackFunction&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddVisualizationCallback))
      .def("Solve", &MathematicalProgram::Solve)
      .def("GetSolverId", &MathematicalProgram::GetSolverId)
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
      .def("GetSolution",
           [](const MathematicalProgram& prog, const Variable& var) {
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
      .def("SubstituteSolution",
          [](const MathematicalProgram& prog,
            const symbolic::Expression& e) {
          return prog.SubstituteSolution(e);
          })
      .def("SubstituteSolution",
          [](const MathematicalProgram& prog,
            const symbolic::Polynomial& p) {
          return prog.SubstituteSolution(p);
          })
      .def("GetInitialGuess",
          [](MathematicalProgram& prog,
             const symbolic::Variable& decision_variable) {
            return prog.GetInitialGuess(decision_variable);
          })
      .def("GetInitialGuess",
          [](MathematicalProgram& prog,
             const VectorXDecisionVariable& decision_variables) {
            return prog.GetInitialGuess(decision_variables);
          })
      .def("GetInitialGuess",
          [](MathematicalProgram& prog,
             const MatrixXDecisionVariable& decision_variables) {
            return prog.GetInitialGuess(decision_variables);
          })
      .def("SetInitialGuess",
          [](MathematicalProgram& prog,
             const symbolic::Variable& decision_variable,
             double variable_guess_value) {
            prog.SetInitialGuess(decision_variable, variable_guess_value);
          })
      .def("SetInitialGuess",
          [](MathematicalProgram& prog,
             const MatrixXDecisionVariable& decision_variable_mat,
             const Eigen::MatrixXd& x0) {
            prog.SetInitialGuess(decision_variable_mat, x0);
          })
      .def("SetInitialGuessForAllVariables",
          [](MathematicalProgram& prog,
             const Eigen::VectorXd& x0) {
            prog.SetInitialGuessForAllVariables(x0);
          })
      .def("SetSolverOption", &SetSolverOptionBySolverType<double>)
      .def("SetSolverOption", &SetSolverOptionBySolverType<int>)
      .def("SetSolverOption", &SetSolverOptionBySolverType<string>);

  py::enum_<SolutionResult>(m, "SolutionResult")
      .value("kSolutionFound", SolutionResult::kSolutionFound)
      .value("kInvalidInput", SolutionResult::kInvalidInput)
      .value("kInfeasibleConstraints", SolutionResult::kInfeasibleConstraints)
      .value("kUnbounded", SolutionResult::kUnbounded)
      .value("kUnknownError", SolutionResult::kUnknownError)
      .value("kInfeasible_Or_Unbounded",
             SolutionResult::kInfeasible_Or_Unbounded)
      .value("kIterationLimit", SolutionResult::kIterationLimit)
      .value("kDualInfeasible", SolutionResult::kDualInfeasible);

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
      .def("A", &LinearConstraint::A)
      .def("UpdateCoefficients",
          [](LinearConstraint& self, const Eigen::MatrixXd& new_A,
             const Eigen::VectorXd& new_lb, const Eigen::VectorXd& new_ub) {
            self.UpdateCoefficients(new_A, new_lb, new_ub);
          }, py::arg("new_A"), py::arg("new_lb"), py::arg("new_ub"))
      .def("UpdateLowerBound",
          [](LinearConstraint& self, const Eigen::VectorXd& new_lb) {
            self.UpdateLowerBound(new_lb);
          }, py::arg("new_lb"))
      .def("UpdateUpperBound",
          [](LinearConstraint& self, const Eigen::VectorXd& new_ub) {
            self.UpdateUpperBound(new_ub);
          }, py::arg("new_ub"))
      .def("set_bounds",
          [](LinearConstraint& self, const Eigen::VectorXd& new_lb,
             const Eigen::VectorXd& new_ub) {
            self.set_bounds(new_lb, new_ub);
          }, py::arg("new_lb"), py::arg("new_ub"));

  py::class_<LorentzConeConstraint, Constraint,
             std::shared_ptr<LorentzConeConstraint>>(
    m, "LorentzConeConstraint")
    .def("A", &LorentzConeConstraint::A);

  py::class_<LinearEqualityConstraint, LinearConstraint,
             std::shared_ptr<LinearEqualityConstraint>>(
      m, "LinearEqualityConstraint");

  py::class_<BoundingBoxConstraint, LinearConstraint,
             std::shared_ptr<BoundingBoxConstraint>>(m,
                                                     "BoundingBoxConstraint");

  py::class_<PositiveSemidefiniteConstraint, Constraint,
             std::shared_ptr<PositiveSemidefiniteConstraint>>(
      m, "PositiveSemidefiniteConstraint");

  py::class_<LinearComplementarityConstraint, Constraint,
             std::shared_ptr<LinearComplementarityConstraint>>(
      m, "LinearComplementarityConstraint");

  RegisterBinding<Constraint>(&m, &prog_cls, "Constraint");
  RegisterBinding<LinearConstraint>(&m, &prog_cls, "LinearConstraint");
  RegisterBinding<LorentzConeConstraint>(&m, &prog_cls,
                                         "LorentzConeConstraint");
  RegisterBinding<LinearEqualityConstraint>(&m, &prog_cls,
                                            "LinearEqualityConstraint");
  RegisterBinding<BoundingBoxConstraint>(&m, &prog_cls,
                                         "BoundingBoxConstraint");
  RegisterBinding<PositiveSemidefiniteConstraint>(
      &m, &prog_cls, "PositiveSemidefiniteConstraint");
  RegisterBinding<LinearComplementarityConstraint>(
      &m, &prog_cls, "LinearComplementarityConstraint");

  // Mirror procedure for costs
  py::class_<Cost, std::shared_ptr<Cost>> cost(m, "Cost");

  py::class_<LinearCost, Cost, std::shared_ptr<LinearCost>>(m, "LinearCost")
      .def("a", &LinearCost::a)
      .def("b", &LinearCost::b);

  py::class_<QuadraticCost, Cost, std::shared_ptr<QuadraticCost>>(
      m, "QuadraticCost")
      .def("Q", &QuadraticCost::Q)
      .def("b", &QuadraticCost::b)
      .def("c", &QuadraticCost::c);

  RegisterBinding<Cost>(&m, &prog_cls, "Cost");
  RegisterBinding<LinearCost>(&m, &prog_cls, "LinearCost");
  RegisterBinding<QuadraticCost>(&m, &prog_cls, "QuadraticCost");

  py::class_<VisualizationCallback, EvaluatorBase,
             std::shared_ptr<VisualizationCallback>>(m,
                                                     "VisualizationCallback");

  RegisterBinding<VisualizationCallback>(&m, &prog_cls,
                                         "VisualizationCallback");
}

}  // namespace pydrake
}  // namespace drake
