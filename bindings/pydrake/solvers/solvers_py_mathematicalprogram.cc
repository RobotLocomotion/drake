#include <cstddef>
#include <memory>
#include <set>

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/get_program_type.h"
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
using solvers::CommonSolverOption;
using solvers::Constraint;
using solvers::Cost;
using solvers::EvaluatorBase;
using solvers::ExponentialConeConstraint;
using solvers::ExpressionConstraint;
using solvers::ExpressionCost;
using solvers::L1NormCost;
using solvers::L2NormCost;
using solvers::LinearComplementarityConstraint;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::LinearMatrixInequalityConstraint;
using solvers::LInfNormCost;
using solvers::LorentzConeConstraint;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::MatrixXDecisionVariable;
using solvers::MatrixXIndeterminate;
using solvers::PerspectiveQuadraticCost;
using solvers::PositiveSemidefiniteConstraint;
using solvers::ProgramType;
using solvers::QuadraticConstraint;
using solvers::QuadraticCost;
using solvers::RotatedLorentzConeConstraint;
using solvers::SolutionResult;
using solvers::SolverId;
using solvers::SolverInterface;
using solvers::SolverOptions;
using solvers::SolverType;
using solvers::SolverTypeConverter;
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
enum class ArrayShapeType { Scalar, Vector };

// Checks array shape, provides user-friendly message if it fails.
void CheckArrayShape(
    py::str var_name, py::array x, ArrayShapeType shape, int size) {
  bool ndim_is_good{};
  py::str ndim_hint;
  if (shape == ArrayShapeType::Scalar) {
    ndim_is_good = (x.ndim() == 0);
    ndim_hint = "0 (scalar)";
  } else {
    ndim_is_good = (x.ndim() == 1 || x.ndim() == 2);
    ndim_hint = "1 or 2 (vector)";
  }
  if (!ndim_is_good || x.size() != size) {
    throw std::runtime_error(
        py::str("{} must be of .ndim = {} and .size = {}. "
                "Got .ndim = {} and .size = {} instead.")
            .format(var_name, ndim_hint, size, x.ndim(), x.size()));
  }
}

// Checks array type, provides user-friendly message if it fails.
template <typename T>
void CheckReturnedArrayType(py::str cls_name, py::array y) {
  py::module m = py::module::import("pydrake.solvers._extra");
  m.attr("_check_returned_array_type")(cls_name, y, GetPyParam<T>()[0]);
}

// Wraps user function to provide better user-friendliness.
template <typename T, typename Func>
Func WrapUserFunc(py::str cls_name, py::function func, int num_vars,
    int num_outputs, ArrayShapeType output_shape) {
  // TODO(eric.cousineau): It would be nicer to write this in Python.
  // TODO(eric.cousineau): Consider using `py::detail::make_caster<>`. However,
  // this may mean the argument is converted twice.
  py::cpp_function wrapped = [=](py::array x) {
    // Check input.
    // WARNING: If the input is badly sized, we will only reach this error in
    // Release mode. In debug mode, an assertion error will be triggered.
    CheckArrayShape(py::str("{}: Input").format(cls_name), x,
        ArrayShapeType::Vector, num_vars);
    // N.B. We use `py::object` instead of `py::array` for the return type
    /// because for dtype=object, you cannot implicitly cast `np.array(T())`
    // (numpy scalar) to `T` (object), at least for AutoDiffXd.
    py::object y = func(x);
    // Check output.
    CheckArrayShape(py::str("{}: Return value").format(cls_name), y,
        output_shape, num_outputs);
    CheckReturnedArrayType<T>(cls_name, y);
    return y;
  };
  return wrapped.cast<Func>();
}

// TODO(eric.cousineau): Make a Python virtual base, and implement this in
// Python instead.
class PyFunctionCost : public Cost {
 public:
  using DoubleFunc = std::function<double(const Eigen::VectorXd&)>;
  using AutoDiffFunc = std::function<AutoDiffXd(const VectorX<AutoDiffXd>&)>;

  // Note that we do not allow Python implementations of Cost to be declared as
  // thread safe.
  PyFunctionCost(
      int num_vars, const py::function& func, const std::string& description)
      : Cost(num_vars, description),
        double_func_(Wrap<double, DoubleFunc>(func)),
        autodiff_func_(Wrap<AutoDiffXd, AutoDiffFunc>(func)) {}

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
  template <typename T, typename Func>
  Func Wrap(py::function func) {
    return WrapUserFunc<T, Func>("PyFunctionCost", func, num_vars(),
        num_outputs(), ArrayShapeType::Scalar);
  }

  const DoubleFunc double_func_;
  const AutoDiffFunc autodiff_func_;
};

// TODO(eric.cousineau): Make a Python virtual base, and implement this in
// Python instead.
class PyFunctionConstraint : public Constraint {
 public:
  using DoubleFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
  using AutoDiffFunc =
      std::function<VectorX<AutoDiffXd>(const VectorX<AutoDiffXd>&)>;

  // Note that we do not allow Python implementations of Constraint to be
  // declared as thread safe.
  PyFunctionConstraint(int num_vars, const py::function& func,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const std::string& description)
      : Constraint(lb.size(), num_vars, lb, ub, description),
        double_func_(Wrap<double, DoubleFunc>(func)),
        autodiff_func_(Wrap<AutoDiffXd, AutoDiffFunc>(func)) {}

  using Constraint::set_bounds;
  using Constraint::UpdateLowerBound;
  using Constraint::UpdateUpperBound;

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
  template <typename T, typename Func>
  Func Wrap(py::function func) {
    return WrapUserFunc<T, Func>("PyFunctionConstraint", func, num_vars(),
        num_outputs(), ArrayShapeType::Vector);
  }

  const DoubleFunc double_func_;
  const AutoDiffFunc autodiff_func_;
};

/// Helper to adapt SolverType to SolverId.
template <typename Value>
void SetSolverOptionBySolverType(MathematicalProgram* self,
    SolverType solver_type, const std::string& key, const Value& value) {
  self->SetSolverOption(SolverTypeConverter::TypeToId(solver_type), key, value);
}

// pybind11 trampoline class to permit overriding virtual functions in Python.
class PySolverInterface : public solvers::SolverInterface {
 public:
  using Base = solvers::SolverInterface;

  PySolverInterface() : Base() {}

  // The following methods are for the pybind11 trampoline class to permit C++
  // to call the correct Python override. This code path is only activated for
  // Python implementations of the class (whose inheritance will pass through
  // `PySolverInterface`). C++ implementations will use the bindings on the
  // interface below.

  bool available() const override {
    PYBIND11_OVERLOAD_PURE(bool, solvers::SolverInterface, available);
  }

  bool enabled() const override {
    PYBIND11_OVERLOAD_PURE(bool, solvers::SolverInterface, enabled);
  }

  void Solve(const solvers::MathematicalProgram& prog,
      const std::optional<Eigen::VectorXd>& initial_guess,
      const std::optional<solvers::SolverOptions>& solver_options,
      solvers::MathematicalProgramResult* result) const override {
    PYBIND11_OVERLOAD_PURE(void, solvers::SolverInterface, Solve, prog,
        initial_guess, solver_options, result);
  }

  solvers::SolverId solver_id() const override {
    PYBIND11_OVERLOAD_PURE(
        solvers::SolverId, solvers::SolverInterface, solver_id);
  }

  bool AreProgramAttributesSatisfied(
      const solvers::MathematicalProgram& prog) const override {
    PYBIND11_OVERLOAD_PURE(
        bool, solvers::SolverInterface, AreProgramAttributesSatisfied, prog);
  }

  std::string ExplainUnsatisfiedProgramAttributes(
      const MathematicalProgram& prog) const override {
    PYBIND11_OVERLOAD_PURE(std::string, solvers::SolverInterface,
        ExplainUnsatisfiedProgramAttributes, prog);
  }
};

void BindSolverInterface(py::module m) {
  constexpr auto& doc = pydrake_doc.drake.solvers;
  py::class_<SolverInterface, PySolverInterface>(
      m, "SolverInterface", doc.SolverInterface.doc)
      .def(py::init([]() { return std::make_unique<PySolverInterface>(); }),
          doc.SolverInterface.ctor.doc)
      // The following bindings are present to allow Python to call C++
      // implementations of this interface.
      .def("available", &SolverInterface::available,
          doc.SolverInterface.available.doc)
      .def(
          "enabled", &SolverInterface::enabled, doc.SolverInterface.enabled.doc)
      .def("solver_id", &SolverInterface::solver_id,
          doc.SolverInterface.solver_id.doc)
      .def(
          "AreProgramAttributesSatisfied",
          [](const SolverInterface& self,
              const solvers::MathematicalProgram& prog) {
            return self.AreProgramAttributesSatisfied(prog);
          },
          py::arg("prog"),
          doc.SolverInterface.AreProgramAttributesSatisfied.doc)
      .def(
          "ExplainUnsatisfiedProgramAttributes",
          [](const SolverInterface& self,
              const solvers::MathematicalProgram& prog) {
            return self.ExplainUnsatisfiedProgramAttributes(prog);
          },
          py::arg("prog"),
          doc.SolverInterface.ExplainUnsatisfiedProgramAttributes.doc)
      .def(
          "Solve",
          [](const SolverInterface& self,
              const solvers::MathematicalProgram& prog,
              const std::optional<Eigen::VectorXd>& initial_guess,
              const std::optional<solvers::SolverOptions>& solver_options,
              solvers::MathematicalProgramResult* result) {
            self.Solve(prog, initial_guess, solver_options, result);
          },
          py::arg("prog"), py::arg("initial_guess"), py::arg("solver_options"),
          py::arg("result"), doc.SolverInterface.Solve.doc)
      .def(
          "Solve",
          // This method really lives on SolverBase, but we manually write it
          // out here to avoid all of the overloading / inheritance hassles.
          [](const SolverInterface& self, const MathematicalProgram& prog,
              const std::optional<Eigen::VectorXd>& initial_guess,
              const std::optional<SolverOptions>& solver_options) {
            MathematicalProgramResult result;
            self.Solve(prog, initial_guess, solver_options, &result);
            return result;
          },
          py::arg("prog"), py::arg("initial_guess") = std::nullopt,
          py::arg("solver_options") = std::nullopt, doc.SolverBase.Solve.doc)
      // TODO(m-chaturvedi) Add Pybind11 documentation.
      .def("solver_type",
          [](const SolverInterface& self) {
            return SolverTypeConverter::IdToType(self.solver_id());
          })
      .def("SolverName",
          [](const SolverInterface& self) { return self.solver_id().name(); });
}

void BindMathematicalProgramResult(py::module m) {
  constexpr auto& doc = pydrake_doc.drake.solvers;
  py::class_<MathematicalProgramResult>(
      m, "MathematicalProgramResult", doc.MathematicalProgramResult.doc)
      .def(py::init<>(), doc.MathematicalProgramResult.ctor.doc)
      .def("is_success", &MathematicalProgramResult::is_success,
          doc.MathematicalProgramResult.is_success.doc)
      .def("set_x_val", &MathematicalProgramResult::set_x_val, py::arg("x_val"),
          doc.MathematicalProgramResult.set_x_val.doc)
      .def("get_x_val", &MathematicalProgramResult::get_x_val,
          doc.MathematicalProgramResult.get_x_val.doc)
      .def("get_solution_result",
          &MathematicalProgramResult::get_solution_result,
          doc.MathematicalProgramResult.get_solution_result.doc)
      .def("set_solution_result",
          &MathematicalProgramResult::set_solution_result,
          doc.MathematicalProgramResult.set_solution_result.doc)
      .def("get_optimal_cost", &MathematicalProgramResult::get_optimal_cost,
          doc.MathematicalProgramResult.get_optimal_cost.doc)
      .def("get_solver_id", &MathematicalProgramResult::get_solver_id,
          doc.MathematicalProgramResult.get_solver_id.doc)
      .def(
          "get_solver_details",
          [](const MathematicalProgramResult& self) {
            const auto& abstract = self.get_abstract_solver_details();
            // TODO(#9398): Figure out why `py_rvp::reference` is necessary.
            py::object value_ref = py::cast(&abstract, py_rvp::reference);
            return value_ref.attr("get_value")();
          },
          py_rvp::reference_internal,
          doc.MathematicalProgramResult.get_solver_details.doc)
      .def(
          "GetSolution",
          [](const MathematicalProgramResult& self) {
            return self.GetSolution();
          },
          doc.MathematicalProgramResult.GetSolution.doc_0args)
      .def(
          "GetSolution",
          [](const MathematicalProgramResult& self, const Variable& var) {
            return self.GetSolution(var);
          },
          doc.MathematicalProgramResult.GetSolution.doc_1args_var)
      .def("SetSolution", &MathematicalProgramResult::SetSolution,
          py::arg("var"), py::arg("value"),
          doc.MathematicalProgramResult.SetSolution.doc)
      .def(
          "GetSolution",
          [](const MathematicalProgramResult& self,
              const VectorXDecisionVariable& var) {
            return self.GetSolution(var);
          },
          doc.MathematicalProgramResult.GetSolution
              .doc_1args_constEigenMatrixBase)
      .def(
          "GetSolution",
          [](const MathematicalProgramResult& self,
              const MatrixXDecisionVariable& var) {
            return self.GetSolution(var);
          },
          doc.MathematicalProgramResult.GetSolution
              .doc_1args_constEigenMatrixBase)
      .def(
          "GetSolution",
          [](const MathematicalProgramResult& self,
              const symbolic::Expression& e) { return self.GetSolution(e); },
          doc.MathematicalProgramResult.GetSolution.doc_1args_e)
      .def(
          "GetSolution",
          [](const MathematicalProgramResult& self,
              const symbolic::Polynomial& p) { return self.GetSolution(p); },
          doc.MathematicalProgramResult.GetSolution.doc_1args_p)
      .def("GetSolution",
          [](const MathematicalProgramResult& self,
              const MatrixX<symbolic::Expression>& mat) {
            return self.GetSolution(mat);
          })
      .def(
          "GetSuboptimalSolution",
          [](const MathematicalProgramResult& self,
              const symbolic::Variable& var, int solution_number) {
            return self.GetSuboptimalSolution(var, solution_number);
          },
          doc.MathematicalProgramResult.GetSuboptimalSolution
              .doc_2args_var_solution_number)
      .def(
          "GetSuboptimalSolution",
          [](const MathematicalProgramResult& self,
              const VectorXDecisionVariable& var, int solution_number) {
            return self.GetSuboptimalSolution(var, solution_number);
          },
          doc.MathematicalProgramResult.GetSuboptimalSolution
              .doc_2args_constEigenMatrixBase_int)
      .def(
          "GetSuboptimalSolution",
          [](const MathematicalProgramResult& self,
              const MatrixXDecisionVariable& var, int solution_number) {
            return self.GetSuboptimalSolution(var, solution_number);
          },
          doc.MathematicalProgramResult.GetSuboptimalSolution
              .doc_2args_constEigenMatrixBase_int)
      .def("num_suboptimal_solution",
          &MathematicalProgramResult::num_suboptimal_solution,
          doc.MathematicalProgramResult.num_suboptimal_solution.doc)
      .def("get_suboptimal_objective",
          &MathematicalProgramResult::get_suboptimal_objective,
          py::arg("solution_number"),
          doc.MathematicalProgramResult.get_suboptimal_objective.doc)
      .def(
          "GetDualSolution",
          [](const MathematicalProgramResult& self,
              const Binding<EvaluatorBase>& constraint) {
            return self.GetDualSolution(constraint);
          },
          doc.MathematicalProgramResult.GetDualSolution.doc)
      .def(
          "EvalBinding",
          [](const MathematicalProgramResult& self,
              const Binding<EvaluatorBase>& binding) {
            return self.EvalBinding(binding);
          },
          doc.MathematicalProgramResult.EvalBinding.doc)
      .def("GetInfeasibleConstraints",
          &MathematicalProgramResult::GetInfeasibleConstraints, py::arg("prog"),
          py::arg("tol") = std::nullopt,
          doc.MathematicalProgramResult.GetInfeasibleConstraints.doc)
      .def("GetInfeasibleConstraintNames",
          &MathematicalProgramResult::GetInfeasibleConstraintNames,
          py::arg("prog"), py::arg("tol") = std::nullopt,
          doc.MathematicalProgramResult.GetInfeasibleConstraintNames.doc);
}

void BindMathematicalProgram(py::module m) {
  constexpr auto& doc = pydrake_doc.drake.solvers;
  py::class_<MathematicalProgram> prog_cls(m, "MathematicalProgram",
      py::dynamic_attr(), doc.MathematicalProgram.doc);
  prog_cls.def(py::init<>(), doc.MathematicalProgram.ctor.doc);
  DefClone(&prog_cls);

  py::enum_<MathematicalProgram::NonnegativePolynomial>(prog_cls,
      "NonnegativePolynomial",
      doc.MathematicalProgram.NonnegativePolynomial.doc)
      .value("kSos", MathematicalProgram::NonnegativePolynomial::kSos,
          doc.MathematicalProgram.NonnegativePolynomial.kSos.doc)
      .value("kSdsos", MathematicalProgram::NonnegativePolynomial::kSdsos,
          doc.MathematicalProgram.NonnegativePolynomial.kSdsos.doc)
      .value("kDsos", MathematicalProgram::NonnegativePolynomial::kDsos,
          doc.MathematicalProgram.NonnegativePolynomial.kDsos.doc);

  prog_cls  // BR
      .def("__str__", &MathematicalProgram::to_string,
          doc.MathematicalProgram.to_string.doc)
      .def("IsThreadSafe", &MathematicalProgram::IsThreadSafe,
          doc.MathematicalProgram.IsThreadSafe.doc)
      .def("ToLatex", &MathematicalProgram::ToLatex, py::arg("precision") = 3,
          doc.MathematicalProgram.ToLatex.doc)
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
      .def(
          "NewSymmetricContinuousVariables",
          // `py::overload_cast` and `overload_cast_explict` struggle with
          // overloads that compete with templated methods.
          [](MathematicalProgram* self, int rows, const string& name) {
            return self->NewSymmetricContinuousVariables(rows, name);
          },
          py::arg("rows"), py::arg("name") = "Symmetric",
          doc.MathematicalProgram.NewSymmetricContinuousVariables.doc_2args)
      .def("AddDecisionVariables", &MathematicalProgram::AddDecisionVariables,
          py::arg("decision_variables"),
          doc.MathematicalProgram.AddDecisionVariables.doc)
      .def("NewFreePolynomial", &MathematicalProgram::NewFreePolynomial,
          py::arg("indeterminates"), py::arg("deg"),
          py::arg("coeff_name") = "a",
          doc.MathematicalProgram.NewFreePolynomial.doc)
      .def("NewEvenDegreeFreePolynomial",
          &MathematicalProgram::NewEvenDegreeFreePolynomial,
          py::arg("indeterminates"), py::arg("degree"),
          py::arg("coeff_name") = "a",
          doc.MathematicalProgram.NewEvenDegreeFreePolynomial.doc)
      .def("NewOddDegreeFreePolynomial",
          &MathematicalProgram::NewOddDegreeFreePolynomial,
          py::arg("indeterminates"), py::arg("degree"),
          py::arg("coeff_name") = "a",
          doc.MathematicalProgram.NewOddDegreeFreePolynomial.doc)
      .def("NewSosPolynomial",
          static_cast<std::pair<Polynomial, MatrixXDecisionVariable> (
              MathematicalProgram::*)(
              const Eigen::Ref<const VectorX<Monomial>>&,
              MathematicalProgram::NonnegativePolynomial,
              const std::string& gram_name)>(
              &MathematicalProgram::NewSosPolynomial),
          py::arg("monomial_basis"),
          py::arg("type") = MathematicalProgram::NonnegativePolynomial::kSos,
          py::arg("gram_name") = "S",
          doc.MathematicalProgram.NewSosPolynomial
              .doc_3args_monomial_basis_type_gram_name)
      .def("NewSosPolynomial",
          static_cast<Polynomial (MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Variable>>&,
              const Eigen::Ref<const VectorX<Monomial>>&,
              MathematicalProgram::NonnegativePolynomial)>(
              &MathematicalProgram::NewSosPolynomial),
          py::arg("gramian"), py::arg("monomial_basis"),
          py::arg("type") = MathematicalProgram::NonnegativePolynomial::kSos,
          doc.MathematicalProgram.NewSosPolynomial
              .doc_3args_gramian_monomial_basis_type)
      .def("NewSosPolynomial",
          static_cast<std::pair<Polynomial, MatrixXDecisionVariable> (
              MathematicalProgram::*)(const Variables&, int,
              MathematicalProgram::NonnegativePolynomial, const std::string&)>(
              &MathematicalProgram::NewSosPolynomial),
          py::arg("indeterminates"), py::arg("degree"),
          py::arg("type") = MathematicalProgram::NonnegativePolynomial::kSos,
          py::arg("gram_name") = "S",
          doc.MathematicalProgram.NewSosPolynomial
              .doc_4args_indeterminates_degree_type_gram_name)
      .def("NewEvenDegreeNonnegativePolynomial",
          &MathematicalProgram::NewEvenDegreeNonnegativePolynomial,
          py::arg("indeterminates"), py::arg("degree"), py::arg("type"),
          doc.MathematicalProgram.NewEvenDegreeNonnegativePolynomial.doc)
      .def("NewEvenDegreeSosPolynomial",
          &MathematicalProgram::NewEvenDegreeSosPolynomial,
          py::arg("indeterminates"), py::arg("degree"),
          doc.MathematicalProgram.NewEvenDegreeSosPolynomial.doc)
      .def("NewEvenDegreeSdsosPolynomial",
          &MathematicalProgram::NewEvenDegreeSdsosPolynomial,
          py::arg("indeterminates"), py::arg("degree"),
          doc.MathematicalProgram.NewEvenDegreeSdsosPolynomial.doc)
      .def("NewEvenDegreeDsosPolynomial",
          &MathematicalProgram::NewEvenDegreeDsosPolynomial,
          py::arg("indeterminates"), py::arg("degree"),
          doc.MathematicalProgram.NewEvenDegreeDsosPolynomial.doc)
      .def("MakePolynomial", &MathematicalProgram::MakePolynomial, py::arg("e"),
          doc.MathematicalProgram.MakePolynomial.doc)
      .def("Reparse", &MathematicalProgram::Reparse, py::arg("p"),
          doc.MathematicalProgram.Reparse.doc)
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
      .def("AddIndeterminate", &MathematicalProgram::AddIndeterminate,
          py::arg("new_indeterminate"),
          doc.MathematicalProgram.AddIndeterminate.doc)
      .def("AddIndeterminates",
          py::overload_cast<const Eigen::Ref<const MatrixXIndeterminate>&>(
              &MathematicalProgram::AddIndeterminates),
          py::arg("new_indeterminates"),
          doc.MathematicalProgram.AddIndeterminates.doc)
      .def("AddIndeterminates",
          py::overload_cast<const symbolic::Variables&>(
              &MathematicalProgram::AddIndeterminates),
          py::arg("new_indeterminates"),
          doc.MathematicalProgram.AddIndeterminates.doc)
      .def("AddVisualizationCallback",
          static_cast<Binding<VisualizationCallback> (MathematicalProgram::*)(
              const VisualizationCallback::CallbackFunction&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddVisualizationCallback),
          doc.MathematicalProgram.AddVisualizationCallback.doc)
      .def(
          "AddCost",
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
          "Adds a cost function.")
      .def(
          "AddCost",
          [](MathematicalProgram* self, const Binding<Cost>& binding) {
            return self->AddCost(binding);
          },
          py::arg("binding"),
          doc.MathematicalProgram.AddCost.doc_1args_binding_cost)
      .def("AddCost",
          static_cast<Binding<Cost> (MathematicalProgram::*)(
              const Expression&)>(&MathematicalProgram::AddCost),
          py::arg("e"), doc.MathematicalProgram.AddCost.doc_1args_e)
      .def(
          "AddCost",
          [](MathematicalProgram* self, Cost* obj,
              const Eigen::Ref<const VectorXDecisionVariable>& vars) {
            // Maintain python wrapper to avoid hazards like #20131.
            py::object obj_py = py::cast(obj);
            return self->AddCost(
                make_shared_ptr_from_py_object<Cost>(obj_py), vars);
          },
          py::arg("obj"), py::arg("vars"),
          doc.MathematicalProgram.AddCost.doc_2args_obj_vars)
      .def("AddLinearCost",
          static_cast<Binding<LinearCost> (MathematicalProgram::*)(
              const Expression&)>(&MathematicalProgram::AddLinearCost),
          py::arg("e"), doc.MathematicalProgram.AddLinearCost.doc_1args)
      .def("AddLinearCost",
          static_cast<Binding<LinearCost> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::VectorXd>&, double,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearCost),
          py::arg("a"), py::arg("b"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearCost.doc_3args)
      .def("AddLinearCost",
          static_cast<Binding<LinearCost> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearCost),
          py::arg("a"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearCost.doc_2args)
      .def("AddQuadraticCost",
          static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(
              const Expression&, std::optional<bool>)>(
              &MathematicalProgram::AddQuadraticCost),
          py::arg("e"), py::arg("is_convex") = py::none(),
          doc.MathematicalProgram.AddQuadraticCost.doc_2args)
      .def("AddQuadraticCost",
          static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&,
              std::optional<bool>)>(&MathematicalProgram::AddQuadraticCost),
          py::arg("Q"), py::arg("b"), py::arg("vars"),
          py::arg("is_convex") = py::none(),
          doc.MathematicalProgram.AddQuadraticCost.doc_4args)
      .def("AddQuadraticCost",
          static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&, double,
              const Eigen::Ref<const VectorXDecisionVariable>&,
              std::optional<bool>)>(&MathematicalProgram::AddQuadraticCost),
          py::arg("Q"), py::arg("b"), py::arg("c"), py::arg("vars"),
          py::arg("is_convex") = py::none(),
          doc.MathematicalProgram.AddQuadraticCost.doc_5args)
      .def("AddQuadraticErrorCost",
          static_cast<Binding<QuadraticCost> (MathematicalProgram::*)(double,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddQuadraticErrorCost),
          py::arg("w"), py::arg("x_desired"), py::arg("vars"),
          doc.MathematicalProgram.AddQuadraticErrorCost
              .doc_3args_w_x_desired_vars)
      .def("AddQuadraticErrorCost",
          overload_cast_explicit<Binding<QuadraticCost>,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&>(
              &MathematicalProgram::AddQuadraticErrorCost),
          py::arg("Q"), py::arg("x_desired"), py::arg("vars"),
          doc.MathematicalProgram.AddQuadraticErrorCost
              .doc_3args_Q_x_desired_vars)
      .def("Add2NormSquaredCost",
          overload_cast_explicit<Binding<QuadraticCost>,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&>(
              &MathematicalProgram::Add2NormSquaredCost),
          py::arg("A"), py::arg("b"), py::arg("vars"),
          doc.MathematicalProgram.Add2NormSquaredCost.doc)
      .def("AddL2NormCost",
          overload_cast_explicit<Binding<L2NormCost>,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&>(
              &MathematicalProgram::AddL2NormCost),
          py::arg("A"), py::arg("b"), py::arg("vars"),
          doc.MathematicalProgram.AddL2NormCost.doc_3args_A_b_vars)
      .def("AddL2NormCost",
          overload_cast_explicit<Binding<L2NormCost>,
              const symbolic::Expression&, double, double>(
              &MathematicalProgram::AddL2NormCost),
          py::arg("e"), py::arg("psd_tol") = 1e-8,
          py::arg("coefficient_tol") = 1e-8,
          doc.MathematicalProgram.AddL2NormCost.doc_expression)
      .def("AddL2NormCostUsingConicConstraint",
          &MathematicalProgram::AddL2NormCostUsingConicConstraint, py::arg("A"),
          py::arg("b"), py::arg("vars"),
          doc.MathematicalProgram.AddL2NormCostUsingConicConstraint.doc)
      .def("AddMaximizeLogDeterminantCost",
          static_cast<std::tuple<Binding<LinearCost>,
              VectorX<symbolic::Variable>, MatrixX<symbolic::Expression>> (
              MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Expression>>& X)>(
              &MathematicalProgram::AddMaximizeLogDeterminantCost),
          py::arg("X"),
          doc.MathematicalProgram.AddMaximizeLogDeterminantCost.doc)
      .def("AddLogDeterminantLowerBoundConstraint",
          &MathematicalProgram::AddLogDeterminantLowerBoundConstraint,
          py::arg("X"), py::arg("lower"),
          doc.MathematicalProgram.AddLogDeterminantLowerBoundConstraint.doc)
      .def("AddMaximizeGeometricMeanCost",
          overload_cast_explicit<Binding<LinearCost>,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorX<symbolic::Variable>>&>(
              &MathematicalProgram::AddMaximizeGeometricMeanCost),
          py::arg("A"), py::arg("b"), py::arg("x"),
          doc.MathematicalProgram.AddMaximizeGeometricMeanCost.doc_3args)
      .def("AddMaximizeGeometricMeanCost",
          overload_cast_explicit<Binding<LinearCost>,
              const Eigen::Ref<const VectorX<symbolic::Variable>>&, double>(
              &MathematicalProgram::AddMaximizeGeometricMeanCost),
          py::arg("x"), py::arg("c"),
          doc.MathematicalProgram.AddMaximizeGeometricMeanCost.doc_2args)
      .def(
          "AddConstraint",
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
      .def(
          "AddConstraint",
          [](MathematicalProgram* self, Constraint* constraint,
              const Eigen::Ref<const VectorXDecisionVariable>& vars) {
            // Maintain python wrapper to avoid hazards like #20131.
            py::object constraint_py = py::cast(constraint);
            return self->AddConstraint(
                make_shared_ptr_from_py_object<Constraint>(constraint_py),
                vars);
          },
          py::arg("constraint"), py::arg("vars"),
          doc.MathematicalProgram.AddConstraint.doc_2args_con_vars)
      .def(
          "AddConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixX<Formula>>& formulas) {
            return self->AddConstraint(formulas);
          },
          py::arg("formulas"),
          doc.MathematicalProgram.AddConstraint.doc_1args_constEigenDenseBase)
      .def(
          "AddConstraint",
          [](MathematicalProgram* self, const Binding<Constraint>& binding) {
            return self->AddConstraint(binding);
          },
          py::arg("binding"),
          doc.MathematicalProgram.AddConstraint.doc_1args_binding)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearConstraint),
          py::arg("A"), py::arg("lb"), py::arg("ub"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearConstraint.doc_4args_A_lb_ub_dense)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::RowVectorXd>&, double, double,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearConstraint),
          py::arg("a"), py::arg("lb"), py::arg("ub"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearConstraint.doc_4args_a_lb_ub_vars)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Eigen::SparseMatrix<double>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearConstraint),
          py::arg("A"), py::arg("lb"), py::arg("ub"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearConstraint.doc_4args_A_lb_ub_sparse)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Expression&, double, double)>(
              &MathematicalProgram::AddLinearConstraint),
          py::arg("e"), py::arg("lb"), py::arg("ub"),
          doc.MathematicalProgram.AddLinearConstraint.doc_3args_e_lb_ub)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Expression>>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&)>(
              &MathematicalProgram::AddLinearConstraint),
          py::arg("v"), py::arg("lb"), py::arg("ub"),
          doc.MathematicalProgram.AddLinearConstraint.doc_3args_v_lb_ub)
      .def("AddLinearConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Formula&)>(&MathematicalProgram::AddLinearConstraint),
          py::arg("f"), doc.MathematicalProgram.AddLinearConstraint.doc_1args_f)
      .def(
          "AddLinearConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixX<Formula>>& formulas) {
            return self->AddLinearConstraint(formulas.array());
          },
          py::arg("formulas"),
          doc.MathematicalProgram.AddLinearConstraint.doc_1args_formulas)
      .def("AddLinearEqualityConstraint",
          static_cast<Binding<LinearEqualityConstraint> (
              MathematicalProgram::*)(const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearEqualityConstraint),
          py::arg("Aeq"), py::arg("beq"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearEqualityConstraint
              .doc_3args_Aeq_beq_dense)
      .def("AddLinearEqualityConstraint",
          static_cast<Binding<LinearEqualityConstraint> (
              MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::RowVectorXd>&, double,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearEqualityConstraint),
          py::arg("a"), py::arg("beq"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearEqualityConstraint
              .doc_3args_a_beq_vars)
      .def("AddLinearEqualityConstraint",
          static_cast<Binding<LinearEqualityConstraint> (
              MathematicalProgram::*)(const Eigen::SparseMatrix<double>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearEqualityConstraint),
          py::arg("Aeq"), py::arg("beq"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearEqualityConstraint
              .doc_3args_Aeq_beq_sparse)
      .def("AddLinearEqualityConstraint",
          static_cast<Binding<LinearEqualityConstraint> (
              MathematicalProgram::*)(const Expression&, double)>(
              &MathematicalProgram::AddLinearEqualityConstraint),
          py::arg("e"), py::arg("b"),
          doc.MathematicalProgram.AddLinearEqualityConstraint.doc_2args_e_b)
      .def("AddLinearEqualityConstraint",
          static_cast<Binding<LinearEqualityConstraint> (
              MathematicalProgram::*)(const Formula&)>(
              &MathematicalProgram::AddLinearEqualityConstraint),
          py::arg("f"),
          doc.MathematicalProgram.AddLinearEqualityConstraint.doc_1args_f)
      .def("AddLinearEqualityConstraint",
          static_cast<Binding<LinearEqualityConstraint> (
              MathematicalProgram::*)(const Eigen::Ref<const Eigen::Array<
                  symbolic::Formula, Eigen::Dynamic, Eigen::Dynamic>>&)>(
              &MathematicalProgram::AddLinearEqualityConstraint),
          py::arg("formulas"),
          doc.MathematicalProgram.AddLinearEqualityConstraint
              .doc_1args_formulas)
      .def(
          "AddLinearEqualityConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
              const Eigen::Ref<const Eigen::VectorXd>& b) {
            return self->AddLinearEqualityConstraint(v, b);
          },
          py::arg("v"), py::arg("b"),
          doc.MathematicalProgram.AddLinearEqualityConstraint
              .doc_2args_constEigenMatrixBase_constEigenMatrixBase)
      .def("AddBoundingBoxConstraint",
          static_cast<Binding<BoundingBoxConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const MatrixXDecisionVariable>&)>(
              &MathematicalProgram::AddBoundingBoxConstraint),
          doc.MathematicalProgram.AddBoundingBoxConstraint.doc_3args_lb_ub_vars)
      .def("AddBoundingBoxConstraint",
          static_cast<Binding<BoundingBoxConstraint> (MathematicalProgram::*)(
              double, double, const symbolic::Variable&)>(
              &MathematicalProgram::AddBoundingBoxConstraint),
          doc.MathematicalProgram.AddBoundingBoxConstraint.doc_3args_lb_ub_var)
      .def(
          "AddBoundingBoxConstraint",
          [](MathematicalProgram* self, double lb, double ub,
              const Eigen::Ref<const MatrixX<symbolic::Variable>>& vars) {
            return self->AddBoundingBoxConstraint(lb, ub, vars);
          },
          doc.MathematicalProgram.AddBoundingBoxConstraint
              .doc_3args_double_double_constEigenMatrixBase)
      .def("AddQuadraticConstraint",
          static_cast<Binding<QuadraticConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&, double, double,
              const Eigen::Ref<const VectorXDecisionVariable>&,
              std::optional<QuadraticConstraint::HessianType>)>(
              &MathematicalProgram::AddQuadraticConstraint),
          py::arg("Q"), py::arg("b"), py::arg("lb"), py::arg("ub"),
          py::arg("vars"), py::arg("hessian_type") = std::nullopt,
          doc.MathematicalProgram.AddQuadraticConstraint.doc_6args)
      .def("AddQuadraticConstraint",
          static_cast<Binding<QuadraticConstraint> (MathematicalProgram::*)(
              const symbolic::Expression&, double, double,
              std::optional<QuadraticConstraint::HessianType>)>(
              &MathematicalProgram::AddQuadraticConstraint),
          py::arg("e"), py::arg("lb"), py::arg("ub"),
          py::arg("hessian_type") = std::nullopt,
          doc.MathematicalProgram.AddQuadraticConstraint.doc_4args)
      .def("AddLorentzConeConstraint",
          static_cast<Binding<LorentzConeConstraint> (MathematicalProgram::*)(
              const symbolic::Formula&, LorentzConeConstraint::EvalType, double,
              double)>(&MathematicalProgram::AddLorentzConeConstraint),
          py::arg("f"),
          py::arg("eval_type") = LorentzConeConstraint::EvalType::kConvexSmooth,
          py::arg("psd_tol") = 1e-8, py::arg("coefficient_tol") = 1e-8,
          doc.MathematicalProgram.AddLorentzConeConstraint.doc_formula)
      .def("AddLorentzConeConstraint",
          static_cast<Binding<LorentzConeConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const VectorX<drake::symbolic::Expression>>&,
              LorentzConeConstraint::EvalType)>(
              &MathematicalProgram::AddLorentzConeConstraint),
          py::arg("v"),
          py::arg("eval_type") = LorentzConeConstraint::EvalType::kConvexSmooth,
          doc.MathematicalProgram.AddLorentzConeConstraint
              .doc_2args_v_eval_type)
      .def(
          "AddLorentzConeConstraint",
          [](MathematicalProgram* self,
              const symbolic::Expression& linear_expression,
              const symbolic::Expression& quadratic_expression, double tol,
              LorentzConeConstraint::EvalType eval_type) {
            return self->AddLorentzConeConstraint(
                linear_expression, quadratic_expression, tol, eval_type);
          },
          py::arg("linear_expression"), py::arg("quadratic_expression"),
          py::arg("tol") = 0.,
          py::arg("eval_type") = LorentzConeConstraint::EvalType::kConvexSmooth,
          doc.MathematicalProgram.AddLorentzConeConstraint
              .doc_4args_linear_expression_quadratic_expression_tol_eval_type)
      .def(
          "AddLorentzConeConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const Eigen::MatrixXd>& A,
              const Eigen::Ref<const Eigen::VectorXd>& b,
              const Eigen::Ref<const VectorXDecisionVariable>& vars,
              LorentzConeConstraint::EvalType eval_type) {
            return self->AddLorentzConeConstraint(A, b, vars, eval_type);
          },
          py::arg("A"), py::arg("b"), py::arg("vars"),
          py::arg("eval_type") = LorentzConeConstraint::EvalType::kConvexSmooth,
          doc.MathematicalProgram.AddLorentzConeConstraint
              .doc_4args_A_b_vars_eval_type)
      .def(
          "AddRotatedLorentzConeConstraint",
          [](MathematicalProgram* self,
              const symbolic::Expression& linear_expression1,
              const symbolic::Expression& linear_expression2,
              const symbolic::Expression& quadratic_expression, double tol) {
            return self->AddRotatedLorentzConeConstraint(linear_expression1,
                linear_expression2, quadratic_expression, tol);
          },
          py::arg("linear_expression1"), py::arg("linear_expression2"),
          py::arg("quadratic_expression"), py::arg("tol") = 0,
          doc.MathematicalProgram.AddRotatedLorentzConeConstraint
              .doc_4args_linear_expression1_linear_expression2_quadratic_expression_tol)
      .def(
          "AddRotatedLorentzConeConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const VectorX<symbolic::Expression>>& v) {
            return self->AddRotatedLorentzConeConstraint(v);
          },
          py::arg("v"),
          doc.MathematicalProgram.AddRotatedLorentzConeConstraint.doc_1args_v)
      .def(
          "AddRotatedLorentzConeConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const Eigen::MatrixXd>& A,
              const Eigen::Ref<const Eigen::VectorXd>& b,
              const Eigen::Ref<const VectorXDecisionVariable>& vars) {
            return self->AddRotatedLorentzConeConstraint(A, b, vars);
          },
          py::arg("A"), py::arg("b"), py::arg("vars"),
          doc.MathematicalProgram.AddRotatedLorentzConeConstraint
              .doc_3args_A_b_vars)
      .def(
          "AddQuadraticAsRotatedLorentzConeConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const Eigen::MatrixXd>& Q,
              const Eigen::Ref<const Eigen::VectorXd>& b, double c,
              const Eigen::Ref<const VectorXDecisionVariable>& vars,
              double psd_tol) {
            return self->AddQuadraticAsRotatedLorentzConeConstraint(
                Q, b, c, vars, psd_tol);
          },
          py::arg("Q"), py::arg("b"), py::arg("c"), py::arg("vars"),
          py::arg("psd_tol") = 0.,
          doc.MathematicalProgram.AddQuadraticAsRotatedLorentzConeConstraint
              .doc)
      .def("AddLinearComplementarityConstraint",
          static_cast<Binding<LinearComplementarityConstraint> (
              MathematicalProgram::*)(const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearComplementarityConstraint),
          py::arg("M"), py::arg("q"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearComplementarityConstraint.doc)
      .def(
          "AddPositiveSemidefiniteConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixXDecisionVariable>& vars) {
            return self->AddPositiveSemidefiniteConstraint(vars);
          },
          doc.MathematicalProgram.AddPositiveSemidefiniteConstraint
              .doc_1args_symmetric_matrix_var)
      .def(
          "AddPositiveSemidefiniteConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixX<Expression>>& e) {
            return self->AddPositiveSemidefiniteConstraint(e);
          },
          doc.MathematicalProgram.AddPositiveSemidefiniteConstraint.doc_1args_e)
      .def(
          "AddPrincipalSubmatrixIsPsdConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixXDecisionVariable>& vars,
              std::set<int> minor_indices) {
            return self->AddPrincipalSubmatrixIsPsdConstraint(
                vars, minor_indices);
          },
          doc.MathematicalProgram.AddPrincipalSubmatrixIsPsdConstraint
              .doc_2args_symmetric_matrix_var_minor_indices)
      .def(
          "AddPrincipalSubmatrixIsPsdConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixX<Expression>>& e,
              std::set<int> minor_indices) {
            return self->AddPrincipalSubmatrixIsPsdConstraint(e, minor_indices);
          },
          doc.MathematicalProgram.AddPrincipalSubmatrixIsPsdConstraint
              .doc_2args_e_minor_indices)
      .def(
          "AddLinearMatrixInequalityConstraint",
          [](MathematicalProgram* self, std::vector<Eigen::MatrixXd> F,
              const Eigen::Ref<const VectorXDecisionVariable>& vars) {
            return self->AddLinearMatrixInequalityConstraint(
                std::move(F), vars);
          },
          py::arg("F"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearMatrixInequalityConstraint.doc_2args)
      .def(
          "AddLinearMatrixInequalityConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixX<symbolic::Expression>>& X) {
            return self->AddLinearMatrixInequalityConstraint(X);
          },
          py::arg("X"),
          doc.MathematicalProgram.AddLinearMatrixInequalityConstraint.doc_1args)
      .def("AddPositiveDiagonallyDominantMatrixConstraint",
          &MathematicalProgram::AddPositiveDiagonallyDominantMatrixConstraint,
          py::arg("X"),
          doc.MathematicalProgram.AddPositiveDiagonallyDominantMatrixConstraint
              .doc)
      .def("TightenPsdConstraintToDd",
          &MathematicalProgram::TightenPsdConstraintToDd, py::arg("constraint"),
          doc.MathematicalProgram.TightenPsdConstraintToDd.doc)
      .def("AddPositiveDiagonallyDominantDualConeMatrixConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Expression>>&)>(
              &MathematicalProgram::
                  AddPositiveDiagonallyDominantDualConeMatrixConstraint),
          py::arg("X"),
          doc.MathematicalProgram
              .AddPositiveDiagonallyDominantDualConeMatrixConstraint
              .doc_expression)
      .def("AddPositiveDiagonallyDominantDualConeMatrixConstraint",
          static_cast<Binding<LinearConstraint> (MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Variable>>&)>(
              &MathematicalProgram::
                  AddPositiveDiagonallyDominantDualConeMatrixConstraint),
          py::arg("X"),
          doc.MathematicalProgram
              .AddPositiveDiagonallyDominantDualConeMatrixConstraint
              .doc_variable)
      .def("RelaxPsdConstraintToDdDualCone",
          &MathematicalProgram::RelaxPsdConstraintToDdDualCone,
          py::arg("constraint"),
          doc.MathematicalProgram.RelaxPsdConstraintToDdDualCone.doc)
      .def("AddScaledDiagonallyDominantMatrixConstraint",
          static_cast<std::vector<std::vector<Matrix2<symbolic::Expression>>> (
              MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Expression>>&)>(
              &MathematicalProgram::
                  AddScaledDiagonallyDominantMatrixConstraint),
          py::arg("X"),
          doc.MathematicalProgram.AddScaledDiagonallyDominantMatrixConstraint
              .doc_expression)
      .def("AddScaledDiagonallyDominantMatrixConstraint",
          static_cast<std::vector<std::vector<Matrix2<symbolic::Variable>>> (
              MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Variable>>&)>(
              &MathematicalProgram::
                  AddScaledDiagonallyDominantMatrixConstraint),
          py::arg("X"),
          doc.MathematicalProgram.AddScaledDiagonallyDominantMatrixConstraint
              .doc_variable)
      .def("TightenPsdConstraintToSdd",
          &MathematicalProgram::TightenPsdConstraintToSdd,
          py::arg("constraint"),
          doc.MathematicalProgram.TightenPsdConstraintToSdd.doc)
      .def("AddScaledDiagonallyDominantDualConeMatrixConstraint",
          static_cast<std::vector<Binding<RotatedLorentzConeConstraint>> (
              MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Expression>>&)>(
              &MathematicalProgram::
                  AddScaledDiagonallyDominantDualConeMatrixConstraint),
          py::arg("X"),
          doc.MathematicalProgram
              .AddScaledDiagonallyDominantDualConeMatrixConstraint
              .doc_expression)
      .def("AddScaledDiagonallyDominantDualConeMatrixConstraint",
          static_cast<std::vector<Binding<RotatedLorentzConeConstraint>> (
              MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Variable>>&)>(
              &MathematicalProgram::
                  AddScaledDiagonallyDominantDualConeMatrixConstraint),
          py::arg("X"),
          doc.MathematicalProgram
              .AddScaledDiagonallyDominantDualConeMatrixConstraint.doc_variable)
      .def("RelaxPsdConstraintToSddDualCone",
          &MathematicalProgram::RelaxPsdConstraintToSddDualCone,
          py::arg("constraint"),
          doc.MathematicalProgram.RelaxPsdConstraintToSddDualCone.doc)
      .def("AddSosConstraint",
          static_cast<MatrixXDecisionVariable (MathematicalProgram::*)(
              const Polynomial&, const Eigen::Ref<const VectorX<Monomial>>&,
              MathematicalProgram::NonnegativePolynomial,
              const std::string& gram_name)>(
              &MathematicalProgram::AddSosConstraint),
          py::arg("p"), py::arg("monomial_basis"),
          py::arg("type") = MathematicalProgram::NonnegativePolynomial::kSos,
          py::arg("gram_name") = "S",
          doc.MathematicalProgram.AddSosConstraint
              .doc_4args_p_monomial_basis_type_gram_name)
      .def("AddSosConstraint",
          static_cast<std::pair<MatrixXDecisionVariable,
              VectorX<symbolic::Monomial>> (MathematicalProgram::*)(
              const Polynomial&, MathematicalProgram::NonnegativePolynomial,
              const std::string& gram_name)>(
              &MathematicalProgram::AddSosConstraint),
          py::arg("p"),
          py::arg("type") = MathematicalProgram::NonnegativePolynomial::kSos,
          py::arg("gram_name") = "S",
          doc.MathematicalProgram.AddSosConstraint.doc_3args_p_type_gram_name)
      .def("AddSosConstraint",
          static_cast<MatrixXDecisionVariable (MathematicalProgram::*)(
              const Expression&, const Eigen::Ref<const VectorX<Monomial>>&,
              MathematicalProgram::NonnegativePolynomial,
              const std::string& gram_name)>(
              &MathematicalProgram::AddSosConstraint),
          py::arg("e"), py::arg("monomial_basis"),
          py::arg("type") = MathematicalProgram::NonnegativePolynomial::kSos,
          py::arg("gram_name") = "S",
          doc.MathematicalProgram.AddSosConstraint
              .doc_4args_e_monomial_basis_type_gram_name)
      .def("AddSosConstraint",
          static_cast<std::pair<MatrixXDecisionVariable,
              VectorX<symbolic::Monomial>> (MathematicalProgram::*)(
              const Expression&, MathematicalProgram::NonnegativePolynomial,
              const std::string& gram_name)>(
              &MathematicalProgram::AddSosConstraint),
          py::arg("e"),
          py::arg("type") = MathematicalProgram::NonnegativePolynomial::kSos,
          py::arg("gram_name") = "S",
          doc.MathematicalProgram.AddSosConstraint.doc_3args_e_type_gram_name)
      .def("AddEqualityConstraintBetweenPolynomials",
          &MathematicalProgram::AddEqualityConstraintBetweenPolynomials,
          py::arg("p1"), py::arg("p2"),
          doc.MathematicalProgram.AddEqualityConstraintBetweenPolynomials.doc)
      .def(
          "AddExponentialConeConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const Eigen::Matrix3Xd>& A,
              const Eigen::Ref<const Eigen::Vector3d>& b,
              const Eigen::Ref<const VectorXDecisionVariable>& vars) {
            return self->AddExponentialConeConstraint(A.sparseView(), b, vars);
          },
          py::arg("A"), py::arg("b"), py::arg("vars"),
          doc.MathematicalProgram.AddExponentialConeConstraint.doc_3args)
      .def(
          "AddExponentialConeConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const Vector3<symbolic::Expression>>& z) {
            return self->AddExponentialConeConstraint(z);
          },
          py::arg("z"),
          doc.MathematicalProgram.AddExponentialConeConstraint.doc_1args)
      .def(
          "GetInitialGuess",
          [](MathematicalProgram& prog,
              const symbolic::Variable& decision_variable) {
            return prog.GetInitialGuess(decision_variable);
          },
          doc.MathematicalProgram.GetInitialGuess.doc_1args_decision_variable)
      .def(
          "GetInitialGuess",
          [](MathematicalProgram& prog,
              const VectorXDecisionVariable& decision_variables) {
            return prog.GetInitialGuess(decision_variables);
          },
          doc.MathematicalProgram.GetInitialGuess
              .doc_1args_constEigenMatrixBase)
      .def(
          "GetInitialGuess",
          [](MathematicalProgram& prog,
              const MatrixXDecisionVariable& decision_variables) {
            return prog.GetInitialGuess(decision_variables);
          },
          doc.MathematicalProgram.GetInitialGuess
              .doc_1args_constEigenMatrixBase)
      .def(
          "SetInitialGuess",
          [](MathematicalProgram& prog,
              const symbolic::Variable& decision_variable,
              double variable_guess_value) {
            prog.SetInitialGuess(decision_variable, variable_guess_value);
          },
          doc.MathematicalProgram.SetInitialGuess
              .doc_2args_decision_variable_variable_guess_value)
      .def(
          "SetInitialGuess",
          [](MathematicalProgram& prog,
              const MatrixXDecisionVariable& decision_variable_mat,
              const Eigen::MatrixXd& x0) {
            prog.SetInitialGuess(decision_variable_mat, x0);
          },
          doc.MathematicalProgram.SetInitialGuess
              .doc_2args_constEigenMatrixBase_constEigenMatrixBase)
      .def(
          "SetInitialGuessForAllVariables",
          [](MathematicalProgram& prog, const Eigen::VectorXd& x0) {
            prog.SetInitialGuessForAllVariables(x0);
          },
          doc.MathematicalProgram.SetInitialGuessForAllVariables.doc)
      .def("SetDecisionVariableValueInVector",
          py::overload_cast<const symbolic::Variable&, double,
              EigenPtr<Eigen::VectorXd>>(
              &MathematicalProgram::SetDecisionVariableValueInVector,
              py::const_),
          py::arg("decision_variable"), py::arg("decision_variable_new_value"),
          py::arg("values"),
          doc.MathematicalProgram.SetDecisionVariableValueInVector
              .doc_3args_decision_variable_decision_variable_new_value_values)
      .def("SetDecisionVariableValueInVector",
          py::overload_cast<const Eigen::Ref<const MatrixXDecisionVariable>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              EigenPtr<Eigen::VectorXd>>(
              &MathematicalProgram::SetDecisionVariableValueInVector,
              py::const_),
          py::arg("decision_variables"),
          py::arg("decision_variables_new_values"), py::arg("values"),
          doc.MathematicalProgram.SetDecisionVariableValueInVector
              .doc_3args_decision_variables_decision_variables_new_values_values)
      .def("SetSolverOption",
          py::overload_cast<const SolverId&, const std::string&, double>(
              &MathematicalProgram::SetSolverOption),
          py::arg("solver_id"), py::arg("solver_option"),
          py::arg("option_value"),
          doc.MathematicalProgram.SetSolverOption.doc_double_option)
      .def("SetSolverOption",
          py::overload_cast<const SolverId&, const std::string&, int>(
              &MathematicalProgram::SetSolverOption),
          py::arg("solver_id"), py::arg("solver_option"),
          py::arg("option_value"),
          doc.MathematicalProgram.SetSolverOption.doc_int_option)
      .def("SetSolverOption",
          py::overload_cast<const SolverId&, const std::string&,
              const std::string&>(&MathematicalProgram::SetSolverOption),
          py::arg("solver_id"), py::arg("solver_option"),
          py::arg("option_value"),
          doc.MathematicalProgram.SetSolverOption.doc_string_option)
      .def("SetSolverOption", &SetSolverOptionBySolverType<double>,
          doc.MathematicalProgram.SetSolverOption.doc_double_option)
      .def("SetSolverOption", &SetSolverOptionBySolverType<int>,
          doc.MathematicalProgram.SetSolverOption.doc_int_option)
      .def("SetSolverOption", &SetSolverOptionBySolverType<string>,
          doc.MathematicalProgram.SetSolverOption.doc_string_option)
      .def("SetSolverOptions", &MathematicalProgram::SetSolverOptions,
          doc.MathematicalProgram.SetSolverOptions.doc)
      .def("solver_options", &MathematicalProgram::solver_options,
          py_rvp::reference_internal,
          doc.MathematicalProgram.solver_options.doc);
// Deprecated 2025-09-01.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  prog_cls  // BR
      .def("GetSolverOptions",
          WrapDeprecated(
              doc.MathematicalProgram.GetSolverOptionsDouble.doc_deprecated,
              [](MathematicalProgram& prog, SolverId solver_id) {
                py::dict out;
                py::object update = out.attr("update");
                update(prog.GetSolverOptionsDouble(solver_id));
                update(prog.GetSolverOptionsInt(solver_id));
                update(prog.GetSolverOptionsStr(solver_id));
                return out;
              }))
      .def("GetSolverOptions",
          WrapDeprecated(
              doc.MathematicalProgram.GetSolverOptionsDouble.doc_deprecated,
              [](MathematicalProgram& prog, SolverType solver_type) {
                py::dict out;
                py::object update = out.attr("update");
                const SolverId id = SolverTypeConverter::TypeToId(solver_type);
                update(prog.GetSolverOptionsDouble(id));
                update(prog.GetSolverOptionsInt(id));
                update(prog.GetSolverOptionsStr(id));
                return out;
              }));
#pragma GCC diagnostic pop
  prog_cls  // BR
      .def("generic_costs", &MathematicalProgram::generic_costs,
          doc.MathematicalProgram.generic_costs.doc)
      .def("generic_constraints", &MathematicalProgram::generic_constraints,
          doc.MathematicalProgram.generic_constraints.doc)
      .def("linear_equality_constraints",
          &MathematicalProgram::linear_equality_constraints,
          doc.MathematicalProgram.linear_equality_constraints.doc)
      .def("linear_costs", &MathematicalProgram::linear_costs,
          doc.MathematicalProgram.linear_costs.doc)
      .def("quadratic_costs", &MathematicalProgram::quadratic_costs,
          doc.MathematicalProgram.quadratic_costs.doc)
      .def("l2norm_costs", &MathematicalProgram::l2norm_costs,
          doc.MathematicalProgram.l2norm_costs.doc)
      .def("linear_constraints", &MathematicalProgram::linear_constraints,
          doc.MathematicalProgram.linear_constraints.doc)
      .def("quadratic_constraints", &MathematicalProgram::quadratic_constraints,
          doc.MathematicalProgram.quadratic_constraints.doc)
      .def("lorentz_cone_constraints",
          &MathematicalProgram::lorentz_cone_constraints,
          doc.MathematicalProgram.lorentz_cone_constraints.doc)
      .def("rotated_lorentz_cone_constraints",
          &MathematicalProgram::rotated_lorentz_cone_constraints,
          doc.MathematicalProgram.rotated_lorentz_cone_constraints.doc)
      .def("positive_semidefinite_constraints",
          &MathematicalProgram::positive_semidefinite_constraints,
          doc.MathematicalProgram.positive_semidefinite_constraints.doc)
      .def("linear_matrix_inequality_constraints",
          &MathematicalProgram::linear_matrix_inequality_constraints,
          doc.MathematicalProgram.linear_matrix_inequality_constraints.doc)
      .def("exponential_cone_constraints",
          &MathematicalProgram::exponential_cone_constraints,
          doc.MathematicalProgram.exponential_cone_constraints.doc)
      .def("bounding_box_constraints",
          &MathematicalProgram::bounding_box_constraints,
          doc.MathematicalProgram.bounding_box_constraints.doc)
      .def("linear_complementarity_constraints",
          &MathematicalProgram::linear_complementarity_constraints,
          doc.MathematicalProgram.linear_complementarity_constraints.doc)
      .def("visualization_callbacks",
          &MathematicalProgram::visualization_callbacks,
          doc.MathematicalProgram.visualization_callbacks.doc)
      .def("GetAllCosts", &MathematicalProgram::GetAllCosts,
          doc.MathematicalProgram.GetAllCosts.doc)
      .def("GetLinearConstraints",
          &MathematicalProgram::GetAllLinearConstraints,
          doc.MathematicalProgram.GetAllLinearConstraints.doc)
      .def("GetAllConstraints", &MathematicalProgram::GetAllConstraints,
          doc.MathematicalProgram.GetAllConstraints.doc)
      .def("num_vars", &MathematicalProgram::num_vars,
          doc.MathematicalProgram.num_vars.doc)
      .def("num_indeterminates", &MathematicalProgram::num_indeterminates,
          doc.MathematicalProgram.num_indeterminates.doc)
      .def("initial_guess", &MathematicalProgram::initial_guess,
          doc.MathematicalProgram.initial_guess.doc)
      .def("FindDecisionVariableIndex",
          &MathematicalProgram::FindDecisionVariableIndex, py::arg("var"),
          doc.MathematicalProgram.FindDecisionVariableIndex.doc)
      .def("FindDecisionVariableIndices",
          &MathematicalProgram::FindDecisionVariableIndices, py::arg("vars"),
          doc.MathematicalProgram.FindDecisionVariableIndices.doc)
      .def("FindIndeterminateIndex",
          &MathematicalProgram::FindIndeterminateIndex, py::arg("var"),
          doc.MathematicalProgram.FindIndeterminateIndex.doc)
      .def(
          "EvalBindingVectorized",
          [](const MathematicalProgram& prog,
              const Binding<EvaluatorBase>& binding,
              const MatrixX<double>& prog_var_vals) {
            DRAKE_DEMAND(prog_var_vals.rows() == prog.num_vars());
            MatrixX<double> Y(
                binding.evaluator()->num_outputs(), prog_var_vals.cols());
            for (int i = 0; i < prog_var_vals.cols(); ++i) {
              Y.col(i) = prog.EvalBinding(binding, prog_var_vals.col(i));
            }
            return Y;
          },
          py::arg("binding"), py::arg("prog_var_vals"),
          R"""(A "vectorized" version of EvalBinding.  It evaluates the binding
for every column of ``prog_var_vals``. )""")
      .def(
          "EvalBinding",
          [](const MathematicalProgram& prog,
              const Binding<EvaluatorBase>& binding,
              const VectorX<double>& prog_var_vals) {
            return prog.EvalBinding(binding, prog_var_vals);
          },
          py::arg("binding"), py::arg("prog_var_vals"),
          doc.MathematicalProgram.EvalBinding.doc)
      .def(
          "EvalBinding",
          [](const MathematicalProgram& prog,
              const Binding<EvaluatorBase>& binding,
              const VectorX<AutoDiffXd>& prog_var_vals) {
            return prog.EvalBinding(binding, prog_var_vals);
          },
          py::arg("binding"), py::arg("prog_var_vals"),
          doc.MathematicalProgram.EvalBinding.doc)
      .def(
          "EvalBindingAtInitialGuess",
          [](const MathematicalProgram& prog,
              const Binding<EvaluatorBase>& binding) {
            return prog.EvalBindingAtInitialGuess(binding);
          },
          py::arg("binding"),
          doc.MathematicalProgram.EvalBindingAtInitialGuess.doc)
      .def(
          "EvalBindings",
          [](const MathematicalProgram& prog,
              const std::vector<Binding<EvaluatorBase>>& binding,
              const VectorX<double>& prog_var_vals) {
            return prog.EvalBindings(binding, prog_var_vals);
          },
          py::arg("bindings"), py::arg("prog_var_vals"),
          doc.MathematicalProgram.EvalBindings.doc)
      .def(
          "EvalBindings",
          [](const MathematicalProgram& prog,
              const std::vector<Binding<EvaluatorBase>>& binding,
              const VectorX<AutoDiffXd>& prog_var_vals) {
            return prog.EvalBindings(binding, prog_var_vals);
          },
          py::arg("bindings"), py::arg("prog_var_vals"),
          doc.MathematicalProgram.EvalBindings.doc)
      .def(
          "GetBindingVariableValues",
          [](const MathematicalProgram& prog,
              const Binding<EvaluatorBase>& binding,
              const VectorX<double>& prog_var_vals) {
            return prog.GetBindingVariableValues(binding, prog_var_vals);
          },
          py::arg("binding"), py::arg("prog_var_vals"),
          doc.MathematicalProgram.GetBindingVariableValues.doc)
      .def(
          "CheckSatisfied",
          [](const MathematicalProgram& prog,
              const Binding<Constraint>& binding,
              const VectorX<double>& prog_var_vals, double tol) {
            return prog.CheckSatisfied(binding, prog_var_vals, tol);
          },
          py::arg("binding"), py::arg("prog_var_vals"), py::arg("tol") = 1e-6,
          doc.MathematicalProgram.CheckSatisfied.doc)
      .def(
          "CheckSatisfied",
          [](const MathematicalProgram& prog,
              const std::vector<Binding<Constraint>>& bindings,
              const VectorX<double>& prog_var_vals, double tol) {
            return prog.CheckSatisfied(bindings, prog_var_vals, tol);
          },
          py::arg("bindings"), py::arg("prog_var_vals"), py::arg("tol") = 1e-6,
          doc.MathematicalProgram.CheckSatisfied.doc_vector)
      .def("CheckSatisfiedAtInitialGuess",
          overload_cast_explicit<bool, const Binding<Constraint>&, double>(
              &MathematicalProgram::CheckSatisfiedAtInitialGuess),
          py::arg("binding"), py::arg("tol") = 1e-6,
          doc.MathematicalProgram.CheckSatisfiedAtInitialGuess.doc)
      .def("CheckSatisfiedAtInitialGuess",
          overload_cast_explicit<bool, const std::vector<Binding<Constraint>>&,
              double>(&MathematicalProgram::CheckSatisfiedAtInitialGuess),
          py::arg("bindings"), py::arg("tol") = 1e-6,
          doc.MathematicalProgram.CheckSatisfiedAtInitialGuess.doc_vector)
      .def("indeterminates", &MathematicalProgram::indeterminates,
          // dtype = object arrays must be copied, and cannot be referenced.
          py_rvp::copy, doc.MathematicalProgram.indeterminates.doc)
      .def("indeterminate", &MathematicalProgram::indeterminate, py::arg("i"),
          doc.MathematicalProgram.indeterminate.doc)
      .def("required_capabilities", &MathematicalProgram::required_capabilities,
          doc.MathematicalProgram.required_capabilities.doc)
      .def("indeterminates_index", &MathematicalProgram::indeterminates_index,
          doc.MathematicalProgram.indeterminates_index.doc)
      .def("decision_variables", &MathematicalProgram::decision_variables,
          // dtype = object arrays must be copied, and cannot be referenced.
          py_rvp::copy, doc.MathematicalProgram.decision_variables.doc)
      .def("decision_variable", &MathematicalProgram::decision_variable,
          py::arg("i"), doc.MathematicalProgram.decision_variable.doc)
      .def("decision_variable_index",
          &MathematicalProgram::decision_variable_index,
          doc.MathematicalProgram.decision_variable_index.doc)
      .def("GetVariableScaling", &MathematicalProgram::GetVariableScaling,
          doc.MathematicalProgram.GetVariableScaling.doc)
      .def("SetVariableScaling", &MathematicalProgram::SetVariableScaling,
          py::arg("var"), py::arg("s"),
          doc.MathematicalProgram.SetVariableScaling.doc)
      .def("ClearVariableScaling", &MathematicalProgram::ClearVariableScaling,
          doc.MathematicalProgram.ClearVariableScaling.doc)
      .def("RemoveDecisionVariable",
          &MathematicalProgram::RemoveDecisionVariable, py::arg("var"),
          doc.MathematicalProgram.RemoveDecisionVariable.doc)
      .def("RemoveCost", &MathematicalProgram::RemoveCost, py::arg("cost"),
          doc.MathematicalProgram.RemoveCost.doc)
      .def("RemoveConstraint", &MathematicalProgram::RemoveConstraint,
          py::arg("constraint"), doc.MathematicalProgram.RemoveConstraint.doc)
      .def("RemoveVisualizationCallback",
          &MathematicalProgram::RemoveVisualizationCallback,
          py::arg("callback"),
          doc.MathematicalProgram.RemoveVisualizationCallback.doc);
}  // NOLINT(readability/fn_size)

void BindSolutionResult(py::module m) {
  constexpr auto& doc = pydrake_doc.drake.solvers;
  py::enum_<SolutionResult> solution_result_enum(
      m, "SolutionResult", doc.SolutionResult.doc);
  solution_result_enum
      .value("kSolutionFound", SolutionResult::kSolutionFound,
          doc.SolutionResult.kSolutionFound.doc)
      .value("kInvalidInput", SolutionResult::kInvalidInput,
          doc.SolutionResult.kInvalidInput.doc)
      .value("kInfeasibleConstraints", SolutionResult::kInfeasibleConstraints,
          doc.SolutionResult.kInfeasibleConstraints.doc)
      .value("kUnbounded", SolutionResult::kUnbounded,
          doc.SolutionResult.kUnbounded.doc)
      .value("kSolverSpecificError", SolutionResult::kSolverSpecificError,
          doc.SolutionResult.kSolverSpecificError.doc)
      .value("kInfeasibleOrUnbounded", SolutionResult::kInfeasibleOrUnbounded,
          doc.SolutionResult.kInfeasibleOrUnbounded.doc)
      .value("kIterationLimit", SolutionResult::kIterationLimit,
          doc.SolutionResult.kIterationLimit.doc)
      .value("kDualInfeasible", SolutionResult::kDualInfeasible,
          doc.SolutionResult.kDualInfeasible.doc)
      .value("kSolutionResultNotSet", SolutionResult::kSolutionResultNotSet,
          doc.SolutionResult.kSolutionResultNotSet.doc);
}

void BindPyFunctionConstraint(py::module m) {
  py::class_<PyFunctionConstraint, Constraint,
      std::shared_ptr<PyFunctionConstraint>>(m, "PyFunctionConstraint",
      "Constraint with its evaluator as a Python function")
      .def("UpdateLowerBound", &PyFunctionConstraint::UpdateLowerBound,
          py::arg("new_lb"), "Update the lower bound of the constraint.")
      .def("UpdateUpperBound", &PyFunctionConstraint::UpdateUpperBound,
          py::arg("new_ub"), "Update the upper bound of the constraint.")
      .def("set_bounds", &PyFunctionConstraint::set_bounds,
          // TODO(hongkai.dai): use new_lb and new_ub as kwargs.
          py::arg("lower_bound"), py::arg("upper_bound"),
          "Set both the lower and upper bounds of the constraint.");
}

void BindFreeFunctions(py::module m) {
  constexpr auto& doc = pydrake_doc.drake.solvers;
  // Bind the free functions in choose_best_solver.h and solve.h.
  m  // BR
      .def("ChooseBestSolver", &solvers::ChooseBestSolver, py::arg("prog"),
          doc.ChooseBestSolver.doc)
      .def(
          "MakeSolver", &solvers::MakeSolver, py::arg("id"), doc.MakeSolver.doc)
      .def("MakeFirstAvailableSolver", &solvers::MakeFirstAvailableSolver,
          py::arg("solver_ids"), doc.MakeFirstAvailableSolver.doc)
      .def("GetAvailableSolvers", &solvers::GetAvailableSolvers,
          py::arg("prog_type"), doc.GetAvailableSolvers.doc)
      .def("Solve",
          py::overload_cast<const MathematicalProgram&,
              const std::optional<Eigen::VectorXd>&,
              const std::optional<SolverOptions>&>(&solvers::Solve),
          py::arg("prog"), py::arg("initial_guess") = py::none(),
          py::arg("solver_options") = py::none(), doc.Solve.doc_3args)
      .def("GetProgramType", &solvers::GetProgramType, doc.GetProgramType.doc)
      .def(
          "SolveInParallel",
          // The pybind11 infrastructure cannot handle setting a vector to null,
          // nor having nulls inside of a vector. We must use a lambda signature
          // where pointers are never null by adding `optional<>` decorations.
          // Inside of the lambda we'll demote nullopts back to nullptrs. Note
          // that SolverOptions is not necessarily cheap to copy, so we still
          // carefully accept it by-pointer. The VectorXd is always necessarily
          // copied when going form numpy to Eigen so we still pass it by-value.
          [](std::vector<const MathematicalProgram*> progs,
              std::optional<std::vector<std::optional<Eigen::VectorXd>>>
                  initial_guesses,
              std::optional<std::vector<std::optional<SolverOptions*>>>
                  solver_options,
              std::optional<std::vector<std::optional<SolverId>>> solver_ids,
              const Parallelism& parallelism, bool dynamic_schedule) {
            std::vector<const Eigen::VectorXd*> initial_guesses_ptrs;
            if (initial_guesses.has_value()) {
              initial_guesses_ptrs.reserve(initial_guesses->size());
              for (const auto& guess : *initial_guesses) {
                initial_guesses_ptrs.push_back(guess ? &(*guess) : nullptr);
              }
            }
            std::vector<const SolverOptions*> solver_options_ptrs;
            if (solver_options.has_value()) {
              solver_options_ptrs.reserve(solver_options->size());
              for (const auto& option : *solver_options) {
                solver_options_ptrs.push_back(option ? *option : nullptr);
              }
            }
            return solvers::SolveInParallel(progs,
                initial_guesses.has_value() ? &initial_guesses_ptrs : nullptr,
                solver_options.has_value() ? &solver_options_ptrs : nullptr,
                solver_ids.has_value() ? &(*solver_ids) : nullptr, parallelism,
                dynamic_schedule);
          },
          py::arg("progs"), py::arg("initial_guesses") = std::nullopt,
          py::arg("solver_options") = std::nullopt,
          py::arg("solver_ids") = std::nullopt,
          py::arg("parallelism") = Parallelism::Max(),
          py::arg("dynamic_schedule") = false,
          py::call_guard<py::gil_scoped_release>(),
          doc.SolveInParallel
              .doc_6args_progs_initial_guesses_solver_options_solver_ids_parallelism_dynamic_schedule)
      .def(
          "SolveInParallel",
          [](std::vector<const MathematicalProgram*> progs,
              std::optional<std::vector<std::optional<Eigen::VectorXd>>>
                  initial_guesses,
              const SolverOptions* solver_options,
              const std::optional<SolverId>& solver_id,
              const Parallelism& parallelism, bool dynamic_schedule) {
            std::vector<const Eigen::VectorXd*> initial_guesses_ptrs;
            if (initial_guesses.has_value()) {
              initial_guesses_ptrs.reserve(initial_guesses->size());
              for (const auto& guess : *initial_guesses) {
                initial_guesses_ptrs.push_back(guess ? &(*guess) : nullptr);
              }
            }
            return solvers::SolveInParallel(progs,
                initial_guesses.has_value() ? &initial_guesses_ptrs : nullptr,
                solver_options, solver_id, parallelism, dynamic_schedule);
          },
          py::arg("progs"), py::arg("initial_guesses") = std::nullopt,
          py::arg("solver_options") = std::nullopt,
          py::arg("solver_id") = std::nullopt,
          py::arg("parallelism") = Parallelism::Max(),
          py::arg("dynamic_schedule") = false,
          py::call_guard<py::gil_scoped_release>(),
          doc.SolveInParallel
              .doc_6args_progs_initial_guesses_solver_options_solver_id_parallelism_dynamic_schedule);
}

}  // namespace

namespace internal {
void DefineSolversMathematicalProgram(py::module m) {
  // This list must remain in topological dependency order.
  BindPyFunctionConstraint(m);
  BindMathematicalProgram(m);
  BindSolutionResult(m);
  BindMathematicalProgramResult(m);
  BindSolverInterface(m);
  BindFreeFunctions(m);
}
}  // namespace internal

}  // namespace pydrake
}  // namespace drake
