#include <cstddef>
#include <memory>

#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_optional_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
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
using solvers::ExponentialConeConstraint;
using solvers::LinearComplementarityConstraint;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::LorentzConeConstraint;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::MatrixXDecisionVariable;
using solvers::MatrixXIndeterminate;
using solvers::PositiveSemidefiniteConstraint;
using solvers::QuadraticCost;
using solvers::SolutionResult;
using solvers::SolverId;
using solvers::SolverInterface;
using solvers::SolverOptions;
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
auto RegisterBinding(py::handle* scope, const string& name) {
  constexpr auto& cls_doc = pydrake_doc.drake.solvers.Binding;
  typedef Binding<C> B;
  string pyname = "Binding_" + name;
  py::class_<B> binding_cls(*scope, pyname.c_str());
  binding_cls  // BR
      .def("evaluator", &B::evaluator, cls_doc.evaluator.doc)
      .def("variables", &B::variables, cls_doc.variables.doc);
  if (!std::is_same<C, EvaluatorBase>::value) {
    // This is required for implicit argument conversion. See below for
    // `EvaluatorBase`'s generic constructor for attempting downcasting.
    // TODO(eric.cousineau): See if there is a more elegant mechanism for this.
    py::implicitly_convertible<B, Binding<EvaluatorBase>>();
  }
  // Add deprecated `constraint`.
  binding_cls.def(
      "constraint", &B::evaluator, cls_doc.constraint.doc_deprecated);
  DeprecateAttribute(binding_cls, "constraint",
      "`constraint` is deprecated; please use `evaluator` instead.");
  return binding_cls;
}

class PyFunctionCost : public Cost {
 public:
  using DoubleFunc = std::function<double(const Eigen::VectorXd&)>;
  using AutoDiffFunc = std::function<AutoDiffXd(const VectorX<AutoDiffXd>&)>;

  PyFunctionCost(
      int num_vars, const py::function& func, const std::string& description)
      : Cost(num_vars, description),
        double_func_(py::cast<DoubleFunc>(func)),
        autodiff_func_(py::cast<AutoDiffFunc>(func)) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
      Eigen::VectorXd* y) const override {
    y->resize(1);
    (*y)[0] = double_func_(x);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
      AutoDiffVecXd* y) const override {
    y->resize(1);
    (*y)[0] = autodiff_func_(x);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
      VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "PyFunctionCost does not support symbolic evaluation.");
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
      Eigen::VectorXd* y) const override {
    *y = double_func_(x);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
      AutoDiffVecXd* y) const override {
    *y = autodiff_func_(x);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
      VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "PyFunctionConstraint does not support symbolic evaluation.");
  }

 private:
  const DoubleFunc double_func_;
  const AutoDiffFunc autodiff_func_;
};

constexpr const char* const kSolveDeprecationString =
    "MathematicalProgram methods that assume the solution is stored inside "
    "the program are deprecated; for details and porting advice, see "
    "https://github.com/RobotLocomotion/drake/issues/9633.  This method "
    "will be removed on 2019-06-01.";

}  // namespace

PYBIND11_MODULE(mathematicalprogram, m) {
  m.doc() = "Drake MathematicalProgram Bindings";
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::module::import("pydrake.autodiffutils");
  py::object variable = py::module::import("pydrake.symbolic").attr("Variable");
  py::object variables =
      py::module::import("pydrake.symbolic").attr("Variables");
  py::object expression =
      py::module::import("pydrake.symbolic").attr("Expression");
  py::object formula = py::module::import("pydrake.symbolic").attr("Formula");

  py::class_<SolverInterface>(m, "SolverInterface", doc.SolverInterface.doc)
      .def("available", &SolverInterface::available,
          doc.SolverInterface.available.doc)
      .def("solver_id", &SolverInterface::solver_id,
          doc.SolverInterface.solver_id.doc)
      .def("AreProgramAttributesSatisfied",
          &SolverInterface::AreProgramAttributesSatisfied, py::arg("prog"),
          doc.SolverInterface.AreProgramAttributesSatisfied.doc)
      .def("Solve",
          pydrake::overload_cast_explicit<void, const MathematicalProgram&,
              const optional<Eigen::VectorXd>&, const optional<SolverOptions>&,
              MathematicalProgramResult*>(&SolverInterface::Solve),
          py::arg("prog"), py::arg("initial_guess"), py::arg("solver_options"),
          py::arg("result"), doc.SolverInterface.Solve.doc_4args)
      .def("Solve",
          // This method really lives on SolverBase, but we manually write it
          // out here to avoid all of the overloading / inheritance hassles.
          [](const SolverInterface& self, const MathematicalProgram& prog,
              const optional<Eigen::VectorXd>& initial_guess,
              const optional<SolverOptions>& solver_options) {
            MathematicalProgramResult result;
            self.Solve(prog, initial_guess, solver_options, &result);
            return result;
          },
          py::arg("prog"), py::arg("initial_guess"), py::arg("solver_options"),
          doc.SolverBase.Solve.doc)
      .def("Solve",
          [](const SolverInterface& self, MathematicalProgram& prog) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return self.Solve(prog);
#pragma GCC diagnostic pop
          },
          py::arg("prog"), kSolveDeprecationString)
      // TODO(m-chaturvedi) Add Pybind11 documentation.
      .def("solver_type",
          [](const SolverInterface& self) {
            return SolverTypeConverter::IdToType(self.solver_id());
          })
      .def("SolverName",
          [](const SolverInterface& self) { return self.solver_id().name(); });

  py::class_<SolverId>(m, "SolverId", doc.SolverId.doc)
      .def("name", &SolverId::name, doc.SolverId.name.doc);

  py::enum_<SolverType>(m, "SolverType", doc.SolverType.doc)
      .value("kDReal", SolverType::kDReal, doc.SolverType.kDReal.doc)
      .value("kEqualityConstrainedQP", SolverType::kEqualityConstrainedQP,
          doc.SolverType.kEqualityConstrainedQP.doc)
      .value("kGurobi", SolverType::kGurobi, doc.SolverType.kGurobi.doc)
      .value("kIpopt", SolverType::kIpopt, doc.SolverType.kIpopt.doc)
      .value("kLinearSystem", SolverType::kLinearSystem,
          doc.SolverType.kLinearSystem.doc)
      .value("kMobyLCP", SolverType::kMobyLCP, doc.SolverType.kMobyLCP.doc)
      .value("kMosek", SolverType::kMosek, doc.SolverType.kMosek.doc)
      .value("kNlopt", SolverType::kNlopt, doc.SolverType.kNlopt.doc)
      .value("kOsqp", SolverType::kOsqp, doc.SolverType.kOsqp.doc)
      .value("kSnopt", SolverType::kSnopt, doc.SolverType.kSnopt.doc);

  // TODO(jwnimmer-tri) Bind the accessors for SolverOptions.
  py::class_<SolverOptions>(m, "SolverOptions", doc.SolverOptions.doc)
      .def(py::init<>(), doc.SolverOptions.ctor.doc);

  py::class_<MathematicalProgramResult>(
      m, "MathematicalProgramResult", doc.MathematicalProgramResult.doc)
      .def(py::init<>(), doc.MathematicalProgramResult.ctor.doc)
      .def("is_success", &MathematicalProgramResult::is_success,
          doc.MathematicalProgramResult.is_success.doc)
      .def("get_x_val", &MathematicalProgramResult::get_x_val,
          doc.MathematicalProgramResult.get_x_val.doc)
      .def("get_solution_result",
          &MathematicalProgramResult::get_solution_result,
          doc.MathematicalProgramResult.get_solution_result.doc)
      .def("get_optimal_cost", &MathematicalProgramResult::get_optimal_cost,
          doc.MathematicalProgramResult.get_optimal_cost.doc)
      .def("get_solver_id", &MathematicalProgramResult::get_solver_id,
          doc.MathematicalProgramResult.get_solver_id.doc)
      .def("get_solver_details",
          [](const MathematicalProgramResult& self) {
            const auto& abstract = self.get_abstract_solver_details();
            // TODO(#9398): Figure out why `py_reference` is necessary.
            py::object value_ref = py::cast(&abstract, py_reference);
            return value_ref.attr("get_value")();
          },
          py_reference_internal,
          doc.MathematicalProgramResult.get_solver_details.doc)
      .def("GetSolution",
          [](const MathematicalProgramResult& self) {
            return self.GetSolution();
          },
          doc.MathematicalProgramResult.GetSolution.doc_0args)
      .def("GetSolution",
          [](const MathematicalProgramResult& self, const Variable& var) {
            return self.GetSolution(var);
          },
          doc.MathematicalProgramResult.GetSolution.doc_1args_var)
      .def("GetSolution",
          [](const MathematicalProgramResult& self,
              const VectorXDecisionVariable& var) {
            return self.GetSolution(var);
          },
          doc.MathematicalProgramResult.GetSolution
              .doc_1args_constEigenMatrixBase)
      .def("GetSolution",
          [](const MathematicalProgramResult& self,
              const MatrixXDecisionVariable& var) {
            return self.GetSolution(var);
          },
          doc.MathematicalProgramResult.GetSolution
              .doc_1args_constEigenMatrixBase)
      .def("GetSolution",
          [](const MathematicalProgramResult& self,
              const symbolic::Expression& e) { return self.GetSolution(e); },
          doc.MathematicalProgramResult.GetSolution.doc_1args_e)
      .def("GetSolution",
          [](const MathematicalProgramResult& self,
              const MatrixX<symbolic::Expression>& mat) {
            return self.GetSolution(mat);
          })
      .def("GetSuboptimalSolution",
          [](const MathematicalProgramResult& self,
              const symbolic::Variable& var, int solution_number) {
            return self.GetSuboptimalSolution(var, solution_number);
          },
          doc.MathematicalProgramResult.GetSuboptimalSolution
              .doc_2args_var_solution_number)
      .def("GetSuboptimalSolution",
          [](const MathematicalProgramResult& self,
              const VectorXDecisionVariable& var, int solution_number) {
            return self.GetSuboptimalSolution(var, solution_number);
          },
          doc.MathematicalProgramResult.GetSuboptimalSolution
              .doc_2args_constEigenMatrixBase_int)
      .def("GetSuboptimalSolution",
          [](const MathematicalProgramResult& self,
              const MatrixXDecisionVariable& var, int solution_number) {
            return self.GetSuboptimalSolution(var, solution_number);
          },
          doc.MathematicalProgramResult.GetSuboptimalSolution
              .doc_2args_constEigenMatrixBase_int)
      .def("num_suboptimal_solution()",
          [](const MathematicalProgramResult& self) {
            return self.num_suboptimal_solution();
          },
          doc.MathematicalProgramResult.num_suboptimal_solution.doc)
      .def("get_suboptimal_objective",
          [](const MathematicalProgramResult& self, int solution_number) {
            return self.get_suboptimal_objective(solution_number);
          },
          doc.MathematicalProgramResult.get_suboptimal_objective.doc)
      .def("EvalBinding",
          [](const MathematicalProgramResult& self,
              const Binding<EvaluatorBase>& binding) {
            return self.EvalBinding(binding);
          },
          doc.MathematicalProgramResult.EvalBinding.doc);

  py::class_<MathematicalProgram> prog_cls(
      m, "MathematicalProgram", doc.MathematicalProgram.doc);
  prog_cls.def(py::init<>(), doc.MathematicalProgram.ctor.doc)
      .def("NewContinuousVariables",
          static_cast<VectorXDecisionVariable (MathematicalProgram::*)(
              int, const std::string&)>(
              &MathematicalProgram::NewContinuousVariables),
          py::arg("rows"), py::arg("name") = "x",
          doc.MathematicalProgram.NewContinuousVariables.doc_2args)
      .def("NewContinuousVariables",
          static_cast<MatrixXDecisionVariable (MathematicalProgram::*)(
              int, int, const std::string&)>(
              &MathematicalProgram::NewContinuousVariables),
          py::arg("rows"), py::arg("cols"), py::arg("name") = "x",
          doc.MathematicalProgram.NewContinuousVariables.doc_3args)
      .def("NewBinaryVariables",
          static_cast<VectorXDecisionVariable (MathematicalProgram::*)(int,
              const std::string&)>(&MathematicalProgram::NewBinaryVariables),
          py::arg("rows"), py::arg("name") = "b",
          doc.MathematicalProgram.NewBinaryVariables.doc_2args)
      .def("NewBinaryVariables",
          py::overload_cast<int, int, const string&>(
              &MathematicalProgram::NewBinaryVariables<Dynamic, Dynamic>),
          py::arg("rows"), py::arg("cols"), py::arg("name") = "b",
          doc.MathematicalProgram.NewBinaryVariables.doc_3args)
      .def("NewSymmetricContinuousVariables",
          // `py::overload_cast` and `overload_cast_explict` struggle with
          // overloads that compete with templated methods.
          [](MathematicalProgram* self, int rows, const string& name) {
            return self->NewSymmetricContinuousVariables(rows, name);
          },
          py::arg("rows"), py::arg("name") = "Symmetric",
          doc.MathematicalProgram.NewSymmetricContinuousVariables.doc_2args)
      .def("NewFreePolynomial", &MathematicalProgram::NewFreePolynomial,
          py::arg("indeterminates"), py::arg("deg"),
          py::arg("coeff_name") = "a",
          doc.MathematicalProgram.NewFreePolynomial.doc)
      .def("NewSosPolynomial",
          static_cast<std::pair<Polynomial, MatrixXDecisionVariable> (
              MathematicalProgram::*)(
              const Eigen::Ref<const VectorX<Monomial>>&)>(
              &MathematicalProgram::NewSosPolynomial),
          py::arg("monomial_basis"),
          doc.MathematicalProgram.NewSosPolynomial.doc_1args)
      .def("NewSosPolynomial",
          static_cast<std::pair<Polynomial, MatrixXDecisionVariable> (
              MathematicalProgram::*)(const Variables&, int)>(
              &MathematicalProgram::NewSosPolynomial),
          py::arg("indeterminates"), py::arg("degree"),
          doc.MathematicalProgram.NewSosPolynomial.doc_2args)
      .def("NewIndeterminates",
          static_cast<VectorXIndeterminate (MathematicalProgram::*)(int,
              const std::string&)>(&MathematicalProgram::NewIndeterminates),
          py::arg("rows"), py::arg("name") = "x",
          doc.MathematicalProgram.NewIndeterminates.doc_2args)
      .def("NewIndeterminates",
          static_cast<MatrixXIndeterminate (MathematicalProgram::*)(int, int,
              const std::string&)>(&MathematicalProgram::NewIndeterminates),
          py::arg("rows"), py::arg("cols"), py::arg("name") = "X",
          doc.MathematicalProgram.NewIndeterminates.doc_3args)
      .def("AddBoundingBoxConstraint",
          static_cast<Binding<BoundingBoxConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddBoundingBoxConstraint),
          doc.MathematicalProgram.AddBoundingBoxConstraint.doc_3args_lb_ub_vars)
      .def("AddBoundingBoxConstraint",
          static_cast<Binding<BoundingBoxConstraint> (MathematicalProgram::*)(
              double, double, const symbolic::Variable&)>(
              &MathematicalProgram::AddBoundingBoxConstraint),
          doc.MathematicalProgram.AddBoundingBoxConstraint.doc_3args_lb_ub_var)
      .def("AddBoundingBoxConstraint",
          [](MathematicalProgram* self, double lb, double ub,
              const Eigen::Ref<const MatrixX<symbolic::Variable>>& vars) {
            return self->AddBoundingBoxConstraint(lb, ub, vars);
          },
          doc.MathematicalProgram.AddBoundingBoxConstraint
              .doc_3args_double_double_constEigenMatrixBase)
      .def("AddConstraint",
          [](MathematicalProgram* self, py::function func,
              const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
              const Eigen::Ref<const VectorXDecisionVariable>& vars,
              std::string& description) {
            return self->AddConstraint(
                std::make_shared<PyFunctionConstraint>(
                    vars.size(), func, lb, ub, description),
                vars);
          },
          py::arg("func"), py::arg("lb"), py::arg("ub"), py::arg("vars"),
          py::arg("description") = "",
          "Adds a constraint using a Python function.")
      .def("AddConstraint",
          static_cast<Binding<Constraint> (MathematicalProgram::*)(
              const Expression&, double, double)>(
              &MathematicalProgram::AddConstraint),
          doc.MathematicalProgram.AddConstraint.doc_3args_e_lb_ub)
      .def("AddConstraint",
          static_cast<Binding<Constraint> (MathematicalProgram::*)(
              const Formula&)>(&MathematicalProgram::AddConstraint),
          doc.MathematicalProgram.AddConstraint.doc_1args_f)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearConstraint),
          py::arg("A"), py::arg("lb"), py::arg("ub"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearConstraint.doc_4args_A_lb_ub_vars)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Expression&, double, double)>(
              &MathematicalProgram::AddLinearConstraint),
          py::arg("e"), py::arg("lb"), py::arg("ub"),
          doc.MathematicalProgram.AddLinearConstraint.doc_3args_e_lb_ub)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const VectorX<symbolic::Expression>>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&)>(
              &MathematicalProgram::AddLinearConstraint),
          py::arg("v"), py::arg("lb"), py::arg("ub"),
          doc.MathematicalProgram.AddLinearConstraint.doc_3args_v_lb_ub)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Formula&)>(&MathematicalProgram::AddLinearConstraint),
          py::arg("f"), doc.MathematicalProgram.AddLinearConstraint.doc_1args_f)
      .def("AddLinearConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const VectorX<Formula>>& formulas) {
            return self->AddLinearConstraint(formulas.array());
          },
          py::arg("formulas"),
          doc.MathematicalProgram.AddLinearConstraint
              .doc_1args_constEigenArrayBase)
      .def("AddLinearEqualityConstraint",
          static_cast<Binding<LinearEqualityConstraint> (
              MathematicalProgram::*)(const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearEqualityConstraint),
          doc.MathematicalProgram.AddLinearEqualityConstraint.doc_3args)
      .def("AddLinearEqualityConstraint",
          static_cast<Binding<LinearEqualityConstraint> (
              MathematicalProgram::*)(const Expression&, double)>(
              &MathematicalProgram::AddLinearEqualityConstraint),
          doc.MathematicalProgram.AddLinearEqualityConstraint.doc_2args)
      .def("AddLinearEqualityConstraint",
          static_cast<Binding<LinearEqualityConstraint> (
              MathematicalProgram::*)(const Formula&)>(
              &MathematicalProgram::AddLinearEqualityConstraint),
          doc.MathematicalProgram.AddLinearEqualityConstraint.doc_1args)
      .def("AddLorentzConeConstraint",
          static_cast<Binding<LorentzConeConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const VectorX<drake::symbolic::Expression>>&)>(
              &MathematicalProgram::AddLorentzConeConstraint),
          doc.MathematicalProgram.AddLorentzConeConstraint.doc)
      .def("AddPositiveSemidefiniteConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixXDecisionVariable>& vars) {
            return self->AddPositiveSemidefiniteConstraint(vars);
          },
          doc.MathematicalProgram.AddPositiveSemidefiniteConstraint
              .doc_1args_symmetric_matrix_var)
      .def("AddLinearComplementarityConstraint",
          static_cast<Binding<LinearComplementarityConstraint> (
              MathematicalProgram::*)(const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearComplementarityConstraint),
          doc.MathematicalProgram.AddLinearComplementarityConstraint.doc)
      .def("AddPositiveSemidefiniteConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixX<Expression>>& e) {
            return self->AddPositiveSemidefiniteConstraint(e);
          },
          doc.MathematicalProgram.AddPositiveSemidefiniteConstraint
              .doc_1args_constEigenMatrixBase)
      .def("AddExponentialConeConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const Vector3<symbolic::Expression>>& z) {
            return self->AddExponentialConeConstraint(z);
          },
          doc.MathematicalProgram.AddExponentialConeConstraint.doc_1args)
      .def("AddCost",
          [](MathematicalProgram* self, py::function func,
              const Eigen::Ref<const VectorXDecisionVariable>& vars,
              std::string& description) {
            return self->AddCost(std::make_shared<PyFunctionCost>(
                                     vars.size(), func, description),
                vars);
          },
          py::arg("func"), py::arg("vars"), py::arg("description") = "",
          // N.B. There is no corresponding C++ method, so the docstring here
          // is a literal, not a reference to documentation_pybind.h
          "Adds a cost function")
      .def("AddCost",
          static_cast<Binding<Cost> (MathematicalProgram::*)(
              const Expression&)>(&MathematicalProgram::AddCost),
          // N.B. There is no corresponding C++ method, so the docstring here
          // is a literal, not a reference to documentation_pybind.h
          "Adds a cost expression")
      .def("AddLinearCost",
          static_cast<Binding<LinearCost> (MathematicalProgram::*)(
              const Expression&)>(&MathematicalProgram::AddLinearCost),
          doc.MathematicalProgram.AddLinearCost.doc_1args)
      .def("AddQuadraticCost",
          static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddQuadraticCost),
          doc.MathematicalProgram.AddQuadraticCost.doc_3args)
      .def("AddQuadraticCost",
          static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(
              const Expression&)>(&MathematicalProgram::AddQuadraticCost),
          doc.MathematicalProgram.AddQuadraticCost.doc_1args)
      .def("AddQuadraticErrorCost",
          overload_cast_explicit<Binding<QuadraticCost>,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&>(
              &MathematicalProgram::AddQuadraticErrorCost),
          py::arg("Q"), py::arg("x_desired"), py::arg("vars"),
          doc.MathematicalProgram.AddQuadraticErrorCost.doc)
      .def("AddL2NormCost",
          overload_cast_explicit<Binding<QuadraticCost>,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&>(
              &MathematicalProgram::AddL2NormCost),
          py::arg("A"), py::arg("b"), py::arg("vars"),
          doc.MathematicalProgram.AddL2NormCost.doc)
      .def("AddMaximizeLogDeterminantSymmetricMatrixCost",
          static_cast<void (MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Expression>>& X)>(
              &MathematicalProgram::
                  AddMaximizeLogDeterminantSymmetricMatrixCost),
          py::arg("X"),
          doc.MathematicalProgram.AddMaximizeLogDeterminantSymmetricMatrixCost
              .doc)
      .def("AddMaximizeGeometricMeanCost",
          overload_cast_explicit<void, const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorX<symbolic::Variable>>&>(
              &MathematicalProgram::AddMaximizeGeometricMeanCost),
          py::arg("A"), py::arg("b"), py::arg("x"),
          doc.MathematicalProgram.AddMaximizeGeometricMeanCost.doc_3args)
      .def("AddMaximizeGeometricMeanCost",
          overload_cast_explicit<void,
              const Eigen::Ref<const VectorX<symbolic::Variable>>&, double>(
              &MathematicalProgram::AddMaximizeGeometricMeanCost),
          py::arg("x"), py::arg("c"),
          doc.MathematicalProgram.AddMaximizeGeometricMeanCost.doc_2args)
      .def("AddSosConstraint",
          static_cast<MatrixXDecisionVariable (MathematicalProgram::*)(
              const Polynomial&, const Eigen::Ref<const VectorX<Monomial>>&)>(
              &MathematicalProgram::AddSosConstraint),
          doc.MathematicalProgram.AddSosConstraint.doc_2args_p_monomial_basis)
      .def("AddSosConstraint",
          static_cast<
              std::pair<MatrixXDecisionVariable, VectorX<symbolic::Monomial>> (
                  MathematicalProgram::*)(const Polynomial&)>(
              &MathematicalProgram::AddSosConstraint),
          doc.MathematicalProgram.AddSosConstraint.doc_1args_p)
      .def("AddSosConstraint",
          static_cast<MatrixXDecisionVariable (MathematicalProgram::*)(
              const Expression&, const Eigen::Ref<const VectorX<Monomial>>&)>(
              &MathematicalProgram::AddSosConstraint),
          doc.MathematicalProgram.AddSosConstraint.doc_2args_e_monomial_basis)
      .def("AddSosConstraint",
          static_cast<
              std::pair<MatrixXDecisionVariable, VectorX<symbolic::Monomial>> (
                  MathematicalProgram::*)(const Expression&)>(
              &MathematicalProgram::AddSosConstraint),
          doc.MathematicalProgram.AddSosConstraint.doc_1args_e)
      .def("AddVisualizationCallback",
          static_cast<Binding<VisualizationCallback> (MathematicalProgram::*)(
              const VisualizationCallback::CallbackFunction&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddVisualizationCallback),
          doc.MathematicalProgram.AddVisualizationCallback.doc)
      .def("Solve",
          [](MathematicalProgram* self) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return self->Solve();
#pragma GCC diagnostic pop
          },
          kSolveDeprecationString)
      .def("GetSolverId",
          [](MathematicalProgram* self) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return self->GetSolverId();
#pragma GCC diagnostic pop
          },
          kSolveDeprecationString)
      .def("linear_constraints", &MathematicalProgram::linear_constraints,
          doc.MathematicalProgram.linear_constraints.doc)
      .def("linear_equality_constraints",
          &MathematicalProgram::linear_equality_constraints,
          doc.MathematicalProgram.linear_equality_constraints.doc)
      .def("bounding_box_constraints",
          &MathematicalProgram::bounding_box_constraints,
          doc.MathematicalProgram.bounding_box_constraints.doc)
      .def("linear_costs", &MathematicalProgram::linear_costs,
          doc.MathematicalProgram.linear_costs.doc)
      .def("quadratic_costs", &MathematicalProgram::quadratic_costs,
          doc.MathematicalProgram.quadratic_costs.doc)
      .def("GetAllCosts", &MathematicalProgram::GetAllCosts,
          doc.MathematicalProgram.GetAllCosts.doc)
      .def("GetLinearConstraints",
          &MathematicalProgram::GetAllLinearConstraints,
          doc.MathematicalProgram.GetAllLinearConstraints.doc)
      .def("GetAllConstraints", &MathematicalProgram::GetAllConstraints,
          doc.MathematicalProgram.GetAllConstraints.doc)
      .def("FindDecisionVariableIndex",
          &MathematicalProgram::FindDecisionVariableIndex, py::arg("var"),
          doc.MathematicalProgram.FindDecisionVariableIndex.doc)
      .def("FindDecisionVariableIndices",
          &MathematicalProgram::FindDecisionVariableIndices, py::arg("vars"),
          doc.MathematicalProgram.FindDecisionVariableIndices.doc)
      .def("num_vars", &MathematicalProgram::num_vars,
          doc.MathematicalProgram.num_vars.doc)
      .def("decision_variables", &MathematicalProgram::decision_variables,
          doc.MathematicalProgram.decision_variables.doc)
      .def("GetSolution",
          [](const MathematicalProgram& prog, const Variable& var) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.GetSolution(var);
#pragma GCC diagnostic pop
          },
          kSolveDeprecationString)
      .def("GetSolution",
          [](const MathematicalProgram& prog,
              const VectorXDecisionVariable& var) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.GetSolution(var);
#pragma GCC diagnostic pop
          },
          kSolveDeprecationString)
      .def("GetSolution",
          [](const MathematicalProgram& prog,
              const MatrixXDecisionVariable& var) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.GetSolution(var);
#pragma GCC diagnostic pop
          },
          kSolveDeprecationString)
      .def("SubstituteSolution",
          [](const MathematicalProgram& prog, const symbolic::Expression& e) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.SubstituteSolution(e);
#pragma GCC diagnostic pop
          },
          kSolveDeprecationString)
      .def("SubstituteSolution",
          [](const MathematicalProgram& prog, const symbolic::Polynomial& p) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.SubstituteSolution(p);
#pragma GCC diagnostic pop
          },
          kSolveDeprecationString)
      .def("EvalBinding",
          [](const MathematicalProgram& prog,
              const Binding<EvaluatorBase>& binding,
              const VectorX<double>& prog_var_vals) {
            return prog.EvalBinding(binding, prog_var_vals);
          },
          py::arg("binding"), py::arg("prog_var_vals"),
          doc.MathematicalProgram.EvalBinding.doc)
      .def("EvalBinding",
          [](const MathematicalProgram& prog,
              const Binding<EvaluatorBase>& binding,
              const VectorX<AutoDiffXd>& prog_var_vals) {
            return prog.EvalBinding(binding, prog_var_vals);
          },
          py::arg("binding"), py::arg("prog_var_vals"),
          doc.MathematicalProgram.EvalBinding.doc)
      .def("EvalBindings",
          [](const MathematicalProgram& prog,
              const std::vector<Binding<EvaluatorBase>>& binding,
              const VectorX<double>& prog_var_vals) {
            return prog.EvalBindings(binding, prog_var_vals);
          },
          py::arg("bindings"), py::arg("prog_var_vals"),
          doc.MathematicalProgram.EvalBindings.doc)
      .def("EvalBindings",
          [](const MathematicalProgram& prog,
              const std::vector<Binding<EvaluatorBase>>& binding,
              const VectorX<AutoDiffXd>& prog_var_vals) {
            return prog.EvalBindings(binding, prog_var_vals);
          },
          py::arg("bindings"), py::arg("prog_var_vals"),
          doc.MathematicalProgram.EvalBindings.doc)
      .def("EvalBindingAtSolution",
          [](const MathematicalProgram& prog,
              const Binding<EvaluatorBase>& binding) {
            WarnDeprecated(kSolveDeprecationString);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            return prog.EvalBindingAtSolution(binding);
#pragma GCC diagnostic pop
          },
          py::arg("binding"), kSolveDeprecationString)
      .def("GetInitialGuess",
          [](MathematicalProgram& prog,
              const symbolic::Variable& decision_variable) {
            return prog.GetInitialGuess(decision_variable);
          },
          doc.MathematicalProgram.GetInitialGuess.doc_1args_decision_variable)
      .def("GetInitialGuess",
          [](MathematicalProgram& prog,
              const VectorXDecisionVariable& decision_variables) {
            return prog.GetInitialGuess(decision_variables);
          },
          doc.MathematicalProgram.GetInitialGuess
              .doc_1args_constEigenMatrixBase)
      .def("GetInitialGuess",
          [](MathematicalProgram& prog,
              const MatrixXDecisionVariable& decision_variables) {
            return prog.GetInitialGuess(decision_variables);
          },
          doc.MathematicalProgram.GetInitialGuess
              .doc_1args_constEigenMatrixBase)
      .def("SetInitialGuess",
          [](MathematicalProgram& prog,
              const symbolic::Variable& decision_variable,
              double variable_guess_value) {
            prog.SetInitialGuess(decision_variable, variable_guess_value);
          },
          doc.MathematicalProgram.SetInitialGuess
              .doc_2args_decision_variable_variable_guess_value)
      .def("SetInitialGuess",
          [](MathematicalProgram& prog,
              const MatrixXDecisionVariable& decision_variable_mat,
              const Eigen::MatrixXd& x0) {
            prog.SetInitialGuess(decision_variable_mat, x0);
          },
          doc.MathematicalProgram.SetInitialGuess
              .doc_2args_constEigenMatrixBase_constEigenMatrixBase)
      .def("SetInitialGuessForAllVariables",
          [](MathematicalProgram& prog, const Eigen::VectorXd& x0) {
            prog.SetInitialGuessForAllVariables(x0);
          },
          doc.MathematicalProgram.SetInitialGuessForAllVariables.doc)
      .def("SetDecisionVariableValueInVector",
          [](const MathematicalProgram& prog,
              const symbolic::Variable& decision_variable,
              double decision_variable_new_value,
              Eigen::Ref<Eigen::VectorXd> values) {
            prog.SetDecisionVariableValueInVector(
                decision_variable, decision_variable_new_value, &values);
          },
          py::arg("decision_variable"), py::arg("decision_variable_new_value"),
          py::arg("values"),
          doc.MathematicalProgram.SetDecisionVariableValueInVector
              .doc_3args_decision_variable_decision_variable_new_value_values)
      .def("SetDecisionVariableValueInVector",
          [](const MathematicalProgram& prog,
              const Eigen::Ref<const MatrixXDecisionVariable>&
                  decision_variables,
              const Eigen::Ref<const Eigen::MatrixXd>&
                  decision_variables_new_values,
              Eigen::Ref<Eigen::VectorXd> values) {
            prog.SetDecisionVariableValueInVector(
                decision_variables, decision_variables_new_values, &values);
          },
          py::arg("decision_variables"),
          py::arg("decision_variables_new_values"), py::arg("values"),
          doc.MathematicalProgram.SetDecisionVariableValueInVector
              .doc_3args_decision_variables_decision_variables_new_values_values)
      .def("SetSolverOption", &SetSolverOptionBySolverType<double>,
          doc.MathematicalProgram.SetSolverOption.doc)
      .def("SetSolverOption", &SetSolverOptionBySolverType<int>,
          doc.MathematicalProgram.SetSolverOption.doc)
      .def("SetSolverOption", &SetSolverOptionBySolverType<string>,
          doc.MathematicalProgram.SetSolverOption.doc)
      // TODO(m-chaturvedi) Add Pybind11 documentation.
      .def("GetSolverOptions",
          [](MathematicalProgram& prog, SolverType solver_type) {
            py::dict out;
            py::object update = out.attr("update");
            const SolverId id = SolverTypeConverter::TypeToId(solver_type);
            update(prog.GetSolverOptionsDouble(id));
            update(prog.GetSolverOptionsInt(id));
            update(prog.GetSolverOptionsStr(id));
            return out;
          });

  py::enum_<SolutionResult>(m, "SolutionResult", doc.SolutionResult.doc)
      .value("kSolutionFound", SolutionResult::kSolutionFound,
          doc.SolutionResult.kSolutionFound.doc)
      .value("kInvalidInput", SolutionResult::kInvalidInput,
          doc.SolutionResult.kInvalidInput.doc)
      .value("kInfeasibleConstraints", SolutionResult::kInfeasibleConstraints,
          doc.SolutionResult.kInfeasibleConstraints.doc)
      .value("kUnbounded", SolutionResult::kUnbounded,
          doc.SolutionResult.kUnbounded.doc)
      .value("kUnknownError", SolutionResult::kUnknownError,
          doc.SolutionResult.kUnknownError.doc)
      .value("kInfeasible_Or_Unbounded",
          SolutionResult::kInfeasible_Or_Unbounded,
          doc.SolutionResult.kInfeasible_Or_Unbounded.doc)
      .value("kIterationLimit", SolutionResult::kIterationLimit,
          doc.SolutionResult.kIterationLimit.doc)
      .value("kDualInfeasible", SolutionResult::kDualInfeasible,
          doc.SolutionResult.kDualInfeasible.doc);

  {
    using Class = EvaluatorBase;
    constexpr auto& cls_doc = doc.EvaluatorBase;
    py::class_<Class, std::shared_ptr<EvaluatorBase>> cls(m, "EvaluatorBase");
    cls  // BR
        .def("num_outputs", &Class::num_outputs, cls_doc.num_outputs.doc)
        .def("num_vars", &Class::num_vars, cls_doc.num_vars.doc)
        .def("get_description", &Class::get_description,
            cls_doc.get_description.doc)
        .def("set_description", &Class::set_description,
            cls_doc.set_description.doc);
    auto bind_eval = [&cls, &cls_doc](auto dummy_x, auto dummy_y) {
      using T_x = decltype(dummy_x);
      using T_y = decltype(dummy_y);
      cls.def("Eval",
          [](const Class& self, const Eigen::Ref<const VectorX<T_x>>& x) {
            VectorX<T_y> y(self.num_outputs());
            self.Eval(x, &y);
            return y;
          },
          py::arg("x"), cls_doc.Eval.doc);
    };
    bind_eval(double{}, double{});
    bind_eval(AutoDiffXd{}, AutoDiffXd{});
    bind_eval(symbolic::Variable{}, symbolic::Expression{});
  }

  RegisterBinding<EvaluatorBase>(&m, "EvaluatorBase")
      .def(py::init([](py::object binding) {
        // Define a type-erased downcast to mirror the implicit
        // "downcast-ability" of Binding<> types.
        return std::make_unique<Binding<EvaluatorBase>>(
            binding.attr("evaluator")().cast<std::shared_ptr<EvaluatorBase>>(),
            binding.attr("variables")().cast<VectorXDecisionVariable>());
      }));

  py::class_<Constraint, EvaluatorBase, std::shared_ptr<Constraint>>(
      m, "Constraint", doc.Constraint.doc)
      .def("num_constraints", &Constraint::num_constraints,
          doc.Constraint.num_constraints.doc)
      .def("lower_bound", &Constraint::lower_bound,
          doc.Constraint.lower_bound.doc)
      .def("upper_bound", &Constraint::upper_bound,
          doc.Constraint.upper_bound.doc)
      .def("CheckSatisfied",
          [](Constraint& self, const Eigen::Ref<const Eigen::VectorXd>& x,
              double tol) { return self.CheckSatisfied(x, tol); },
          py::arg("x"), py::arg("tol") = 1E-6,
          doc.Constraint.CheckSatisfied.doc)
      .def("CheckSatisfied",
          [](Constraint& self, const Eigen::Ref<const AutoDiffVecXd>& x,
              double tol) { return self.CheckSatisfied(x, tol); },
          py::arg("x"), py::arg("tol") = 1E-6,
          doc.Constraint.CheckSatisfied.doc)
      .def("CheckSatisfied",
          [](Constraint& self,
              const Eigen::Ref<const VectorX<symbolic::Variable>>& x) {
            return self.CheckSatisfied(x);
          },
          py::arg("x"), doc.Constraint.CheckSatisfied.doc);

  py::class_<LinearConstraint, Constraint, std::shared_ptr<LinearConstraint>>(
      m, "LinearConstraint", doc.LinearConstraint.doc)
      .def("A", &LinearConstraint::A, doc.LinearConstraint.A.doc)
      .def("UpdateCoefficients",
          [](LinearConstraint& self, const Eigen::MatrixXd& new_A,
              const Eigen::VectorXd& new_lb, const Eigen::VectorXd& new_ub) {
            self.UpdateCoefficients(new_A, new_lb, new_ub);
          },
          py::arg("new_A"), py::arg("new_lb"), py::arg("new_ub"),
          doc.LinearConstraint.UpdateCoefficients.doc)
      .def("UpdateLowerBound",
          [](LinearConstraint& self, const Eigen::VectorXd& new_lb) {
            self.UpdateLowerBound(new_lb);
          },
          py::arg("new_lb"), doc.Constraint.UpdateLowerBound.doc)
      .def("UpdateUpperBound",
          [](LinearConstraint& self, const Eigen::VectorXd& new_ub) {
            self.UpdateUpperBound(new_ub);
          },
          py::arg("new_ub"), doc.Constraint.UpdateUpperBound.doc)
      .def("set_bounds",
          [](LinearConstraint& self, const Eigen::VectorXd& new_lb,
              const Eigen::VectorXd& new_ub) {
            self.set_bounds(new_lb, new_ub);
          },
          py::arg("new_lb"), py::arg("new_ub"), doc.Constraint.set_bounds.doc);

  py::class_<LorentzConeConstraint, Constraint,
      std::shared_ptr<LorentzConeConstraint>>(
      m, "LorentzConeConstraint", doc.LorentzConeConstraint.doc)
      .def("A", &LorentzConeConstraint::A, doc.LorentzConeConstraint.A.doc);
  py::class_<LinearEqualityConstraint, LinearConstraint,
      std::shared_ptr<LinearEqualityConstraint>>(
      m, "LinearEqualityConstraint", doc.LinearEqualityConstraint.doc)
      .def("UpdateCoefficients",
          [](LinearEqualityConstraint& self,  // BR
              const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq) {
            self.UpdateCoefficients(Aeq, beq);
          },
          py::arg("Aeq"), py::arg("beq"),
          doc.LinearEqualityConstraint.UpdateCoefficients.doc);

  py::class_<BoundingBoxConstraint, LinearConstraint,
      std::shared_ptr<BoundingBoxConstraint>>(
      m, "BoundingBoxConstraint", doc.BoundingBoxConstraint.doc);

  py::class_<PositiveSemidefiniteConstraint, Constraint,
      std::shared_ptr<PositiveSemidefiniteConstraint>>(m,
      "PositiveSemidefiniteConstraint", doc.PositiveSemidefiniteConstraint.doc);

  py::class_<LinearComplementarityConstraint, Constraint,
      std::shared_ptr<LinearComplementarityConstraint>>(m,
      "LinearComplementarityConstraint",
      doc.LinearComplementarityConstraint.doc);

  py::class_<ExponentialConeConstraint, Constraint,
      std::shared_ptr<ExponentialConeConstraint>>(
      m, "ExponentialConeConstraint", doc.ExponentialConeConstraint.doc);

  RegisterBinding<Constraint>(&m, "Constraint");
  RegisterBinding<LinearConstraint>(&m, "LinearConstraint");
  RegisterBinding<LorentzConeConstraint>(&m, "LorentzConeConstraint");
  RegisterBinding<LinearEqualityConstraint>(&m, "LinearEqualityConstraint");
  RegisterBinding<BoundingBoxConstraint>(&m, "BoundingBoxConstraint");
  RegisterBinding<PositiveSemidefiniteConstraint>(
      &m, "PositiveSemidefiniteConstraint");
  RegisterBinding<LinearComplementarityConstraint>(
      &m, "LinearComplementarityConstraint");
  RegisterBinding<ExponentialConeConstraint>(&m, "ExponentialConeConstraint");

  // Mirror procedure for costs
  py::class_<Cost, EvaluatorBase, std::shared_ptr<Cost>> cost(
      m, "Cost", doc.Cost.doc);

  py::class_<LinearCost, Cost, std::shared_ptr<LinearCost>>(
      m, "LinearCost", doc.LinearCost.doc)
      .def("a", &LinearCost::a, doc.LinearCost.a.doc)
      .def("b", &LinearCost::b, doc.LinearCost.b.doc)
      .def("UpdateCoefficients",
          [](LinearCost& self, const Eigen::VectorXd& new_a, double new_b) {
            self.UpdateCoefficients(new_a, new_b);
          },
          py::arg("new_a"), py::arg("new_b") = 0,
          doc.LinearCost.UpdateCoefficients.doc);

  py::class_<QuadraticCost, Cost, std::shared_ptr<QuadraticCost>>(
      m, "QuadraticCost", doc.QuadraticCost.doc)
      .def("Q", &QuadraticCost::Q, doc.QuadraticCost.Q.doc)
      .def("b", &QuadraticCost::b, doc.QuadraticCost.b.doc)
      .def("c", &QuadraticCost::c, doc.QuadraticCost.c.doc)
      .def("UpdateCoefficients",
          [](QuadraticCost& self, const Eigen::MatrixXd& new_Q,
              const Eigen::VectorXd& new_b,
              double new_c) { self.UpdateCoefficients(new_Q, new_b, new_c); },
          py::arg("new_Q"), py::arg("new_b"), py::arg("new_c") = 0,
          doc.QuadraticCost.UpdateCoefficients.doc);

  RegisterBinding<Cost>(&m, "Cost");
  RegisterBinding<LinearCost>(&m, "LinearCost");
  RegisterBinding<QuadraticCost>(&m, "QuadraticCost");

  py::class_<VisualizationCallback, EvaluatorBase,
      std::shared_ptr<VisualizationCallback>>(
      m, "VisualizationCallback", doc.VisualizationCallback.doc);

  RegisterBinding<VisualizationCallback>(&m, "VisualizationCallback");

  // Bind the free functions in choose_best_solver.h and solve.h.
  m  // BR
      .def("ChooseBestSolver", &solvers::ChooseBestSolver, py::arg("prog"),
          doc.ChooseBestSolver.doc)
      .def(
          "MakeSolver", &solvers::MakeSolver, py::arg("id"), doc.MakeSolver.doc)
      .def("Solve",
          py::overload_cast<const MathematicalProgram&,
              const optional<Eigen::VectorXd>&, const optional<SolverOptions>&>(
              &solvers::Solve),
          py::arg("prog"), py::arg("initial_guess") = py::none(),
          py::arg("solver_options") = py::none(), doc.Solve.doc_3args)
      .def("GetInfeasibleConstraints", &solvers::GetInfeasibleConstraints,
          py::arg("prog"), py::arg("result"), py::arg("tol") = nullopt,
          doc.GetInfeasibleConstraints.doc);
}  // NOLINT(readability/fn_size)

}  // namespace pydrake
}  // namespace drake
