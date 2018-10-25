#include <cstddef>
#include <memory>

#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/bindings/pydrake/util/deprecation_pybind.h"
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
  // TODO(m-chaturvedi) Add Pybind11 documentation.
  auto binding_cls = py::class_<B>(scope, pyname.c_str())
                         .def("evaluator", &B::evaluator)
                         .def("constraint", &B::evaluator)
                         .def("variables", &B::variables);
  // Deprecate `constraint`.
  DeprecateAttribute(
      binding_cls, "constraint",
      "`constraint` is deprecated; please use `evaluator` instead.");
  // Register overloads for MathematicalProgram class
  // TODO(m-chaturvedi) Add Pybind11 documentation.
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
              Eigen::VectorXd* y) const override {
    (*y)[0] = double_func_(x);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
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

  py::class_<MathematicalProgramSolverInterface>(
      m, "MathematicalProgramSolverInterface",
      doc.MathematicalProgramSolverInterface.doc)
      .def("available", &MathematicalProgramSolverInterface::available,
        doc.MathematicalProgramSolverInterface.available.doc)
      .def("solver_id", &MathematicalProgramSolverInterface::solver_id,
        doc.MathematicalProgramSolverInterface.solver_id.doc)
      .def("Solve", &MathematicalProgramSolverInterface::Solve,
        doc.MathematicalProgramSolverInterface.Solve.doc)
      // TODO(m-chaturvedi) Add Pybind11 documentation.
      .def("solver_type",
           [](const MathematicalProgramSolverInterface& self) {
             return SolverTypeConverter::IdToType(self.solver_id());
           })
      .def("SolverName", [](const MathematicalProgramSolverInterface& self) {
        return self.solver_id().name();
      });

  py::class_<SolverId>(m, "SolverId", doc.SolverId.doc).def("name",
    &SolverId::name, doc.SolverId.name.doc);

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

  py::class_<MathematicalProgram> prog_cls(m, "MathematicalProgram",
                                           doc.MathematicalProgram.doc);
  prog_cls.def(py::init<>(), doc.MathematicalProgram.ctor.doc)
      .def("NewContinuousVariables",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<VectorXDecisionVariable (MathematicalProgram::*)(
               int, const std::string&)>(
               &MathematicalProgram::NewContinuousVariables),
           py::arg("rows"), py::arg("name") = "x",
           doc.MathematicalProgram.NewContinuousVariables.doc)
      .def("NewContinuousVariables",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<MatrixXDecisionVariable (MathematicalProgram::*)(
               int, int, const std::string&)>(
               &MathematicalProgram::NewContinuousVariables),
           py::arg("rows"), py::arg("cols"), py::arg("name") = "x",
           doc.MathematicalProgram.NewContinuousVariables.doc_2)
      .def("NewBinaryVariables",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<VectorXDecisionVariable (MathematicalProgram::*)(
               int, const std::string&)>(
               &MathematicalProgram::NewBinaryVariables),
           py::arg("rows"), py::arg("name") = "b",
           doc.MathematicalProgram.NewBinaryVariables.doc)
      .def("NewBinaryVariables",
           py::overload_cast<int, int, const string&>(
               &MathematicalProgram::NewBinaryVariables<Dynamic, Dynamic>),
           py::arg("rows"), py::arg("cols"), py::arg("name") = "b",
           doc.MathematicalProgram.NewBinaryVariables.doc_2)
      .def("NewSymmetricContinuousVariables",
           // `py::overload_cast` and `overload_cast_explict` struggle with
           // overloads that compete with templated methods.
           [](MathematicalProgram* self, int rows, const string& name) {
             return self->NewSymmetricContinuousVariables(rows, name);
           },
           py::arg("rows"), py::arg("name") = "Symmetric",
           doc.MathematicalProgram.NewSymmetricContinuousVariables.doc)
      .def("NewFreePolynomial", &MathematicalProgram::NewFreePolynomial,
           py::arg("indeterminates"), py::arg("deg"),
           py::arg("coeff_name") = "a",
           doc.MathematicalProgram.NewFreePolynomial.doc)
      .def("NewSosPolynomial",
           static_cast<
               std::pair<Polynomial, Binding<PositiveSemidefiniteConstraint>> (
                   MathematicalProgram::*)(
                   const Eigen::Ref<const VectorX<Monomial>>&)>(
               &MathematicalProgram::NewSosPolynomial),
           doc.MathematicalProgram.NewSosPolynomial.doc)
      .def("NewSosPolynomial",
           static_cast<
               std::pair<Polynomial, Binding<PositiveSemidefiniteConstraint>> (
                   MathematicalProgram::*)(const Variables&, int)>(
               &MathematicalProgram::NewSosPolynomial),
           doc.MathematicalProgram.NewSosPolynomial.doc_2)
      .def("NewIndeterminates",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<VectorXIndeterminate (MathematicalProgram::*)(
               int, const std::string&)>(
               &MathematicalProgram::NewIndeterminates),
           py::arg("rows"), py::arg("name") = "x",
           doc.MathematicalProgram.NewIndeterminates.doc)
      .def("NewIndeterminates",
           // NOLINTNEXTLINE(whitespace/parens)
           static_cast<MatrixXIndeterminate (MathematicalProgram::*)(
               int, int, const std::string&)>(
               &MathematicalProgram::NewIndeterminates),
           py::arg("rows"), py::arg("cols"), py::arg("name") = "X",
           doc.MathematicalProgram.NewIndeterminates.doc_2)
      .def("AddBoundingBoxConstraint",
           static_cast<Binding<BoundingBoxConstraint> (MathematicalProgram::*)(
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&)>(
               &MathematicalProgram::AddBoundingBoxConstraint),
           doc.MathematicalProgram.AddBoundingBoxConstraint.doc)
      .def("AddBoundingBoxConstraint",
           static_cast<Binding<BoundingBoxConstraint> (MathematicalProgram::*)(
               double, double, const symbolic::Variable&)>(
               &MathematicalProgram::AddBoundingBoxConstraint),
           doc.MathematicalProgram.AddBoundingBoxConstraint.doc_3)
      .def("AddBoundingBoxConstraint",
           [](MathematicalProgram* self, double lb, double ub,
              const Eigen::Ref<const MatrixX<symbolic::Variable>>& vars) {
             return self->AddBoundingBoxConstraint(lb, ub, vars);
           }, doc.MathematicalProgram.AddBoundingBoxConstraint.doc_4)
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
           py::arg("description") = "",
           doc.MathematicalProgram.AddConstraint.doc_3)
      .def("AddConstraint",
           static_cast<Binding<Constraint> (MathematicalProgram::*)(
               const Expression&, double, double)>(
               &MathematicalProgram::AddConstraint),
           doc.MathematicalProgram.AddConstraint.doc_2)
      .def("AddConstraint",
           static_cast<Binding<Constraint> (MathematicalProgram::*)(
               const Formula&)>(&MathematicalProgram::AddConstraint),
           doc.MathematicalProgram.AddConstraint.doc_5)
      .def("AddLinearConstraint",
           static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
               const Eigen::Ref<const Eigen::MatrixXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&)>(
               &MathematicalProgram::AddLinearConstraint),
           doc.MathematicalProgram.AddLinearConstraint.doc)
      .def("AddLinearConstraint",
           static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
               const Expression&, double, double)>(
               &MathematicalProgram::AddLinearConstraint),
           doc.MathematicalProgram.AddLinearConstraint.doc_5)
      .def("AddLinearConstraint",
           static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
               const Formula&)>(&MathematicalProgram::AddLinearConstraint),
           doc.MathematicalProgram.AddLinearConstraint.doc_7)
      .def("AddLinearEqualityConstraint",
           static_cast<Binding<LinearEqualityConstraint> (
               MathematicalProgram::*)(
               const Eigen::Ref<const Eigen::MatrixXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&)>(
               &MathematicalProgram::AddLinearEqualityConstraint),
           doc.MathematicalProgram.AddLinearEqualityConstraint.doc_7)
      .def("AddLinearEqualityConstraint",
           static_cast<Binding<LinearEqualityConstraint> (
               MathematicalProgram::*)(const Expression&, double)>(
               &MathematicalProgram::AddLinearEqualityConstraint),
           doc.MathematicalProgram.AddLinearEqualityConstraint.doc)
      .def("AddLinearEqualityConstraint",
           static_cast<Binding<LinearEqualityConstraint> (
               MathematicalProgram::*)(const Formula&)>(
               &MathematicalProgram::AddLinearEqualityConstraint),
           doc.MathematicalProgram.AddLinearEqualityConstraint.doc_2)
      .def("AddLorentzConeConstraint",
           static_cast<Binding<LorentzConeConstraint> (MathematicalProgram::*)(
               const Eigen::Ref<const VectorX<drake::symbolic::Expression>>&)>(
               &MathematicalProgram::AddLorentzConeConstraint),
           doc.MathematicalProgram.AddLorentzConeConstraint.doc)
      .def("AddPositiveSemidefiniteConstraint",
           [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixXDecisionVariable>& vars) {
             return self->AddPositiveSemidefiniteConstraint(vars);
           }, doc.MathematicalProgram.AddPositiveSemidefiniteConstraint.doc)
      .def("AddLinearComplementarityConstraint",
           static_cast<Binding<LinearComplementarityConstraint> (
               MathematicalProgram::*)(
               const Eigen::Ref<const Eigen::MatrixXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&)>(
               &MathematicalProgram::AddLinearComplementarityConstraint),
           doc.MathematicalProgram.AddLinearComplementarityConstraint.doc)
      .def("AddPositiveSemidefiniteConstraint",
           [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixX<Expression>>& e) {
             return self->AddPositiveSemidefiniteConstraint(e);
           }, doc.MathematicalProgram.AddPositiveSemidefiniteConstraint.doc)
      .def("AddCost",
           [](MathematicalProgram* self, py::function func,
              const Eigen::Ref<const VectorXDecisionVariable>& vars,
              std::string& description) {
             return self->AddCost(std::make_shared<PyFunctionCost>(
                                      vars.size(), func, description),
                                  vars);
           },
           py::arg("func"), py::arg("vars"), py::arg("description") = "",
           doc.MathematicalProgram.AddCost.doc)
      .def("AddCost",
          static_cast<Binding<Cost> (MathematicalProgram::*)(
          const Expression&)>(&MathematicalProgram::AddCost),
          doc.MathematicalProgram.AddCost.doc)
      .def("AddLinearCost",
           static_cast<Binding<LinearCost> (MathematicalProgram::*)(
               const Expression&)>(&MathematicalProgram::AddLinearCost),
          doc.MathematicalProgram.AddLinearCost.doc)
      .def("AddQuadraticCost",
           static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(
               const Eigen::Ref<const Eigen::MatrixXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&)>(
               &MathematicalProgram::AddQuadraticCost),
           doc.MathematicalProgram.AddQuadraticCost.doc)
      .def("AddQuadraticCost",
           static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(
               const Expression&)>(&MathematicalProgram::AddQuadraticCost),
           doc.MathematicalProgram.AddQuadraticCost.doc)
      .def("AddQuadraticErrorCost",
           overload_cast_explicit<
               Binding<QuadraticCost>, const Eigen::Ref<const Eigen::MatrixXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&>(
               &MathematicalProgram::AddQuadraticErrorCost),
           py::arg("Q"), py::arg("x_desired"), py::arg("vars"),
           doc.MathematicalProgram.AddQuadraticErrorCost.doc)
      .def("AddL2NormCost",
           overload_cast_explicit<
               Binding<QuadraticCost>, const Eigen::Ref<const Eigen::MatrixXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const VectorXDecisionVariable>&>(
               &MathematicalProgram::AddL2NormCost),
           py::arg("A"), py::arg("b"), py::arg("vars"),
           doc.MathematicalProgram.AddL2NormCost.doc)
      .def("AddSosConstraint",
           static_cast<std::pair<Binding<PositiveSemidefiniteConstraint>,
                                 Binding<LinearEqualityConstraint>> (
               MathematicalProgram::*)(
               const Polynomial&, const Eigen::Ref<const VectorX<Monomial>>&)>(
               &MathematicalProgram::AddSosConstraint),
           doc.MathematicalProgram.AddSosConstraint.doc)
      .def("AddSosConstraint",
           static_cast<std::pair<Binding<PositiveSemidefiniteConstraint>,
                                 Binding<LinearEqualityConstraint>> (
               MathematicalProgram::*)(const Polynomial&)>(
               &MathematicalProgram::AddSosConstraint),
           doc.MathematicalProgram.AddSosConstraint.doc)
      .def("AddSosConstraint",
           static_cast<std::pair<Binding<PositiveSemidefiniteConstraint>,
                                 Binding<LinearEqualityConstraint>> (
               MathematicalProgram::*)(
               const Expression&, const Eigen::Ref<const VectorX<Monomial>>&)>(
               &MathematicalProgram::AddSosConstraint),
           doc.MathematicalProgram.AddSosConstraint.doc)
      .def("AddSosConstraint",
           static_cast<std::pair<Binding<PositiveSemidefiniteConstraint>,
                                 Binding<LinearEqualityConstraint>> (
               MathematicalProgram::*)(const Expression&)>(
               &MathematicalProgram::AddSosConstraint),
           doc.MathematicalProgram.AddSosConstraint.doc)
      .def("AddVisualizationCallback",
          static_cast<Binding<VisualizationCallback> (MathematicalProgram::*)(
              const VisualizationCallback::CallbackFunction&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddVisualizationCallback),
          doc.MathematicalProgram.AddVisualizationCallback.doc)
      .def("Solve", &MathematicalProgram::Solve,
          doc.MathematicalProgram.Solve.doc)
      .def("GetSolverId", &MathematicalProgram::GetSolverId,
          doc.MathematicalProgram.GetSolverId.doc)
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
      .def("FindDecisionVariableIndex",
           &MathematicalProgram::FindDecisionVariableIndex,
          doc.MathematicalProgram.FindDecisionVariableIndex.doc)
      .def("num_vars", &MathematicalProgram::num_vars,
          doc.MathematicalProgram.num_vars.doc)
      .def("decision_variables", &MathematicalProgram::decision_variables,
          doc.MathematicalProgram.decision_variables.doc)
      .def("GetSolution",
           [](const MathematicalProgram& prog, const Variable& var) {
             return prog.GetSolution(var);
           }, doc.MathematicalProgram.GetSolution.doc)
      .def("GetSolution",
           [](const MathematicalProgram& prog,
              const VectorXDecisionVariable& var) {
             return prog.GetSolution(var);
           }, doc.MathematicalProgram.GetSolution.doc)
      .def("GetSolution",
           [](const MathematicalProgram& prog,
              const MatrixXDecisionVariable& var) {
             return prog.GetSolution(var);
           }, doc.MathematicalProgram.GetSolution.doc)
      .def("SubstituteSolution",
          [](const MathematicalProgram& prog,
            const symbolic::Expression& e) {
          return prog.SubstituteSolution(e);
          }, doc.MathematicalProgram.SubstituteSolution.doc)
      .def("SubstituteSolution",
          [](const MathematicalProgram& prog,
            const symbolic::Polynomial& p) {
          return prog.SubstituteSolution(p);
          }, doc.MathematicalProgram.SubstituteSolution.doc)
      .def("GetInitialGuess",
          [](MathematicalProgram& prog,
             const symbolic::Variable& decision_variable) {
            return prog.GetInitialGuess(decision_variable);
          }, doc.MathematicalProgram.GetInitialGuess.doc)
      .def("GetInitialGuess",
          [](MathematicalProgram& prog,
             const VectorXDecisionVariable& decision_variables) {
            return prog.GetInitialGuess(decision_variables);
          }, doc.MathematicalProgram.GetInitialGuess.doc)
      .def("GetInitialGuess",
          [](MathematicalProgram& prog,
             const MatrixXDecisionVariable& decision_variables) {
            return prog.GetInitialGuess(decision_variables);
          }, doc.MathematicalProgram.GetInitialGuess.doc)
      .def("SetInitialGuess",
          [](MathematicalProgram& prog,
             const symbolic::Variable& decision_variable,
             double variable_guess_value) {
            prog.SetInitialGuess(decision_variable, variable_guess_value);
          }, doc.MathematicalProgram.SetInitialGuess.doc)
      .def("SetInitialGuess",
          [](MathematicalProgram& prog,
             const MatrixXDecisionVariable& decision_variable_mat,
             const Eigen::MatrixXd& x0) {
            prog.SetInitialGuess(decision_variable_mat, x0);
          }, doc.MathematicalProgram.SetInitialGuess.doc)
      .def("SetInitialGuessForAllVariables",
          [](MathematicalProgram& prog,
             const Eigen::VectorXd& x0) {
            prog.SetInitialGuessForAllVariables(x0);
          }, doc.MathematicalProgram.SetInitialGuessForAllVariables.doc)
      .def("SetSolverOption", &SetSolverOptionBySolverType<double>,
          doc.MathematicalProgram.SetSolverOption.doc)
      .def("SetSolverOption", &SetSolverOptionBySolverType<int>,
          doc.MathematicalProgram.SetSolverOption.doc_2)
      .def("SetSolverOption", &SetSolverOptionBySolverType<string>,
          doc.MathematicalProgram.SetSolverOption.doc_3)
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

  // TODO(eric.cousineau): Expose Eval() in a Python-friendly fashion.
  py::class_<EvaluatorBase, std::shared_ptr<EvaluatorBase>>(m, "EvaluatorBase")
      .def("num_outputs", &EvaluatorBase::num_outputs,
        doc.EvaluatorBase.num_outputs.doc)
      .def("num_vars", &EvaluatorBase::num_vars,
        doc.EvaluatorBase.num_vars.doc);

  py::class_<Constraint, EvaluatorBase, std::shared_ptr<Constraint>>(
      m, "Constraint", doc.Constraint.doc)
      .def("num_constraints", &Constraint::num_constraints,
        doc.Constraint.num_constraints.doc)
      .def("lower_bound", &Constraint::lower_bound,
        doc.Constraint.lower_bound.doc)
      .def("upper_bound", &Constraint::upper_bound,
        doc.Constraint.upper_bound.doc);

  py::class_<LinearConstraint, Constraint, std::shared_ptr<LinearConstraint>>(
      m, "LinearConstraint", doc.LinearConstraint.doc)
      .def("A", &LinearConstraint::A, doc.LinearConstraint.A.doc)
      .def("UpdateCoefficients",
          [](LinearConstraint& self, const Eigen::MatrixXd& new_A,
             const Eigen::VectorXd& new_lb, const Eigen::VectorXd& new_ub) {
            self.UpdateCoefficients(new_A, new_lb, new_ub);
          }, py::arg("new_A"), py::arg("new_lb"), py::arg("new_ub"),
          doc.LinearConstraint.UpdateCoefficients.doc)
      .def("UpdateLowerBound",
          [](LinearConstraint& self, const Eigen::VectorXd& new_lb) {
            self.UpdateLowerBound(new_lb);
          }, py::arg("new_lb"), doc.Constraint.UpdateLowerBound.doc)
      .def("UpdateUpperBound",
          [](LinearConstraint& self, const Eigen::VectorXd& new_ub) {
            self.UpdateUpperBound(new_ub);
          }, py::arg("new_ub"), doc.Constraint.UpdateUpperBound.doc)
      .def("set_bounds",
          [](LinearConstraint& self, const Eigen::VectorXd& new_lb,
             const Eigen::VectorXd& new_ub) {
            self.set_bounds(new_lb, new_ub);
          }, py::arg("new_lb"), py::arg("new_ub"),
          doc.Constraint.set_bounds.doc);

  py::class_<LorentzConeConstraint, Constraint,
             std::shared_ptr<LorentzConeConstraint>>(
    m, "LorentzConeConstraint", doc.LorentzConeConstraint.doc)
    .def("A", &LorentzConeConstraint::A, doc.LorentzConeConstraint.A.doc);
  py::class_<LinearEqualityConstraint, LinearConstraint,
             std::shared_ptr<LinearEqualityConstraint>>(
      m, "LinearEqualityConstraint",
      doc.LinearEqualityConstraint.doc)
      .def("UpdateCoefficients",
          [](LinearEqualityConstraint& self, const Eigen::MatrixXd& Aeq,
             const Eigen::VectorXd& beq) {
            self.UpdateCoefficients(Aeq, beq);
          }, py::arg("Aeq"), py::arg("beq"),
          doc.LinearEqualityConstraint.UpdateCoefficients.doc);

  py::class_<BoundingBoxConstraint, LinearConstraint,
             std::shared_ptr<BoundingBoxConstraint>>(m, "BoundingBoxConstraint",
    doc.BoundingBoxConstraint.ctor.doc);

  py::class_<PositiveSemidefiniteConstraint, Constraint,
             std::shared_ptr<PositiveSemidefiniteConstraint>>(
      m, "PositiveSemidefiniteConstraint",
      doc.PositiveSemidefiniteConstraint.ctor.doc);

  py::class_<LinearComplementarityConstraint, Constraint,
             std::shared_ptr<LinearComplementarityConstraint>>(
      m, "LinearComplementarityConstraint",
      doc.LinearComplementarityConstraint.ctor.doc);

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
  py::class_<Cost, std::shared_ptr<Cost>> cost(m, "Cost", doc.Cost.doc);

  py::class_<LinearCost, Cost, std::shared_ptr<LinearCost>>(m, "LinearCost",
    doc.LinearCost.doc)
      .def("a", &LinearCost::a, doc.LinearCost.a.doc)
      .def("b", &LinearCost::b, doc.LinearCost.b.doc)
      .def("UpdateCoefficients",
          [](LinearCost& self, const Eigen::VectorXd& new_a,
             double new_b) {
            self.UpdateCoefficients(new_a, new_b);
          }, py::arg("new_a"), py::arg("new_b") = 0,
          doc.LinearCost.UpdateCoefficients.doc);

  py::class_<QuadraticCost, Cost, std::shared_ptr<QuadraticCost>>(
      m, "QuadraticCost", doc.QuadraticCost.ctor.doc)
      .def("Q", &QuadraticCost::Q, doc.QuadraticCost.Q.doc)
      .def("b", &QuadraticCost::b, doc.QuadraticCost.b.doc)
      .def("c", &QuadraticCost::c, doc.QuadraticCost.c.doc)
      .def("UpdateCoefficients",
          [](QuadraticCost& self, const Eigen::MatrixXd& new_Q,
             const Eigen::VectorXd& new_b, double new_c) {
            self.UpdateCoefficients(new_Q, new_b, new_c);
          }, py::arg("new_Q"), py::arg("new_b"), py::arg("new_c") = 0,
      doc.QuadraticCost.UpdateCoefficients.doc);

  RegisterBinding<Cost>(&m, &prog_cls, "Cost");
  RegisterBinding<LinearCost>(&m, &prog_cls, "LinearCost");
  RegisterBinding<QuadraticCost>(&m, &prog_cls, "QuadraticCost");

  py::class_<VisualizationCallback, EvaluatorBase,
             std::shared_ptr<VisualizationCallback>>(m, "VisualizationCallback",
    doc.VisualizationCallback.doc);

  RegisterBinding<VisualizationCallback>(&m, &prog_cls,
                                         "VisualizationCallback");
}

}  // namespace pydrake
}  // namespace drake
