#include <cstddef>
#include <memory>

#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
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
using solvers::L2NormCost;
using solvers::LinearComplementarityConstraint;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::LinearMatrixInequalityConstraint;
using solvers::LorentzConeConstraint;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::MatrixXDecisionVariable;
using solvers::MatrixXIndeterminate;
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
auto RegisterBinding(py::handle* scope) {
  constexpr auto& cls_doc = pydrake_doc.drake.solvers.Binding;
  typedef Binding<C> B;
  const string pyname = TemporaryClassName<B>();
  py::class_<B> binding_cls(*scope, pyname.c_str());
  AddTemplateClass(*scope, "Binding", binding_cls, GetPyParam<C>());
  binding_cls  // BR
      .def("evaluator", &B::evaluator, cls_doc.evaluator.doc)
      .def("variables", &B::variables, cls_doc.variables.doc)
      .def("__str__", &B::to_string, cls_doc.to_string.doc);
  if (!std::is_same_v<C, EvaluatorBase>) {
    // This is required for implicit argument conversion. See below for
    // `EvaluatorBase`'s generic constructor for attempting downcasting.
    // TODO(eric.cousineau): See if there is a more elegant mechanism for this.
    py::implicitly_convertible<B, Binding<EvaluatorBase>>();
  }
  if (std::is_base_of_v<Constraint, C> && !std::is_same_v<C, Constraint>) {
    py::implicitly_convertible<B, Binding<Constraint>>();
  }
  if (std::is_base_of_v<Cost, C> && !std::is_same_v<C, Cost>) {
    py::implicitly_convertible<B, Binding<Cost>>();
  }
  return binding_cls;
}

template <typename C, typename PyClass>
void DefBindingCastConstructor(PyClass* cls) {
  static_assert(std::is_same_v<Binding<C>, typename PyClass::type>,
      "Bound type must be Binding<C>");
  (*cls)  // BR
      .def(py::init([](py::object binding) {
        // Define a type-erased downcast to mirror the implicit
        // "downcast-ability" of Binding<> types.
        return std::make_unique<Binding<C>>(
            binding.attr("evaluator")().cast<std::shared_ptr<C>>(),
            binding.attr("variables")().cast<VectorXDecisionVariable>());
      }));
}

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
void CheckArrayType(py::str var_name, py::array x) {
  py::module m = py::module::import("pydrake.solvers.mathematicalprogram");
  m.attr("_check_array_type")(var_name, x, GetPyParam<T>()[0]);
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
    CheckArrayShape(
        py::str("{}: Output").format(cls_name), y, output_shape, num_outputs);
    CheckArrayType<T>(py::str("{}: Output").format(cls_name), y);
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

// pybind11 trampoline class to permit overriding virtual functions in Python.
class PySolverInterface : public py::wrapper<solvers::SolverInterface> {
 public:
  using Base = py::wrapper<solvers::SolverInterface>;

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

class StubEvaluatorBase : public EvaluatorBase {
 public:
  StubEvaluatorBase() : EvaluatorBase(1, 1) {}

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>&,
      Eigen::VectorXd*) const override {
    throw std::runtime_error("Not implemented");
  }

  void DoEval(
      const Eigen::Ref<const AutoDiffVecXd>&, AutoDiffVecXd*) const override {
    throw std::runtime_error("Not implemented");
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
      VectorX<symbolic::Expression>*) const override {
    throw std::runtime_error("Not implemented");
  }
};

void DefTesting(py::module m) {
  // To test binding casting.
  py::class_<StubEvaluatorBase, EvaluatorBase,
      std::shared_ptr<StubEvaluatorBase>>(m, "StubEvaluatorBase");
  RegisterBinding<StubEvaluatorBase>(&m)  // BR
      .def_static(
          "Make", [](const Eigen::Ref<const VectorXDecisionVariable>& v) {
            return Binding<StubEvaluatorBase>(
                std::make_shared<StubEvaluatorBase>(), v);
          });
  m  // BR
      .def("AcceptBindingEvaluatorBase", [](const Binding<EvaluatorBase>&) {})
      .def("AcceptBindingCost", [](const Binding<Cost>&) {})
      .def("AcceptBindingConstraint", [](const Binding<Constraint>&) {});
}

}  // namespace

void BindSolverInterfaceAndFlags(py::module m) {
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

  py::class_<SolverId>(m, "SolverId", doc.SolverId.doc)
      .def(py::init<std::string>(), py::arg("name"), doc.SolverId.ctor.doc)
      .def("name", &SolverId::name, doc.SolverId.name.doc)
      .def("__hash__",
          [](const SolverId& self) { return std::hash<SolverId>{}(self); })
      .def(
          "__eq__",
          [](const SolverId& self, const SolverId& other) {
            return self == other;
          },
          py::is_operator())
      .def(
          "__ne__",
          [](const SolverId& self, const SolverId& other) {
            return self != other;
          },
          py::is_operator());

  py::enum_<ProgramType>(m, "ProgramType", doc.ProgramType.doc)
      .value("kLP", ProgramType::kLP, doc.ProgramType.kLP.doc)
      .value("kQP", ProgramType::kQP, doc.ProgramType.kQP.doc)
      .value("kSOCP", ProgramType::kSOCP, doc.ProgramType.kSOCP.doc)
      .value("kSDP", ProgramType::kSDP, doc.ProgramType.kSDP.doc)
      .value("kGP", ProgramType::kGP, doc.ProgramType.kGP.doc)
      .value("kCGP", ProgramType::kCGP, doc.ProgramType.kCGP.doc)
      .value("kMILP", ProgramType::kMILP, doc.ProgramType.kMILP.doc)
      .value("kMIQP", ProgramType::kMIQP, doc.ProgramType.kMIQP.doc)
      .value("kMISOCP", ProgramType::kMISOCP, doc.ProgramType.kMISOCP.doc)
      .value("kMISDP", ProgramType::kMISDP, doc.ProgramType.kMISDP.doc)
      .value("kQuadraticCostConicConstraint",
          ProgramType::kQuadraticCostConicConstraint,
          doc.ProgramType.kQuadraticCostConicConstraint.doc)
      .value("kNLP", ProgramType::kNLP, doc.ProgramType.kNLP.doc)
      .value("kLCP", ProgramType::kLCP, doc.ProgramType.kLCP.doc)
      .value("kUnknown", ProgramType::kUnknown, doc.ProgramType.kUnknown.doc);

  py::enum_<SolverType>(m, "SolverType", doc.SolverType.doc)
      .value("kClp", SolverType::kClp, doc.SolverType.kClp.doc)
      .value("kCsdp", SolverType::kCsdp, doc.SolverType.kCsdp.doc)
      .value("kDReal", SolverType::kDReal, doc.SolverType.kDReal.doc)
      .value("kEqualityConstrainedQP", SolverType::kEqualityConstrainedQP,
          doc.SolverType.kEqualityConstrainedQP.doc)
      .value("kGurobi", SolverType::kGurobi, doc.SolverType.kGurobi.doc)
      .value("kIbex", SolverType::kIbex, doc.SolverType.kIbex.doc)
      .value("kIpopt", SolverType::kIpopt, doc.SolverType.kIpopt.doc)
      .value("kLinearSystem", SolverType::kLinearSystem,
          doc.SolverType.kLinearSystem.doc)
      .value("kMobyLCP", SolverType::kMobyLCP, doc.SolverType.kMobyLCP.doc)
      .value("kMosek", SolverType::kMosek, doc.SolverType.kMosek.doc)
      .value("kNlopt", SolverType::kNlopt, doc.SolverType.kNlopt.doc)
      .value("kOsqp", SolverType::kOsqp, doc.SolverType.kOsqp.doc)
      .value("kScs", SolverType::kScs, doc.SolverType.kScs.doc)
      .value("kSnopt", SolverType::kSnopt, doc.SolverType.kSnopt.doc);

  // TODO(jwnimmer-tri) Bind the accessors for SolverOptions.
  py::class_<SolverOptions>(m, "SolverOptions", doc.SolverOptions.doc)
      .def(py::init<>(), doc.SolverOptions.ctor.doc)
      .def("SetOption",
          py::overload_cast<const SolverId&, const std::string&, double>(
              &SolverOptions::SetOption),
          py::arg("solver_id"), py::arg("solver_option"),
          py::arg("option_value"),
          doc.SolverOptions.SetOption.doc_double_option)
      .def("SetOption",
          py::overload_cast<const SolverId&, const std::string&, int>(
              &SolverOptions::SetOption),
          py::arg("solver_id"), py::arg("solver_option"),
          py::arg("option_value"), doc.SolverOptions.SetOption.doc_int_option)
      .def("SetOption",
          py::overload_cast<const SolverId&, const std::string&,
              const std::string&>(&SolverOptions::SetOption),
          py::arg("solver_id"), py::arg("solver_option"),
          py::arg("option_value"), doc.SolverOptions.SetOption.doc_str_option)
      .def("SetOption",
          py::overload_cast<CommonSolverOption, SolverOptions::OptionValue>(
              &SolverOptions::SetOption),
          py::arg("key"), py::arg("value"),
          doc.SolverOptions.SetOption.doc_common_option)
      .def(
          "GetOptions",
          [](const SolverOptions& solver_options, SolverId solver_id) {
            py::dict out;
            py::object update = out.attr("update");
            update(solver_options.GetOptionsDouble(solver_id));
            update(solver_options.GetOptionsInt(solver_id));
            update(solver_options.GetOptionsStr(solver_id));
            return out;
          },
          py::arg("solver_id"), doc.SolverOptions.GetOptionsDouble.doc)
      .def("common_solver_options", &SolverOptions::common_solver_options,
          doc.SolverOptions.common_solver_options.doc)
      .def("get_print_file_name", &SolverOptions::get_print_file_name,
          doc.SolverOptions.get_print_file_name.doc)
      .def("get_print_to_console", &SolverOptions::get_print_to_console,
          doc.SolverOptions.get_print_to_console.doc);

  py::enum_<CommonSolverOption>(
      m, "CommonSolverOption", doc.CommonSolverOption.doc)
      .value("kPrintFileName", CommonSolverOption::kPrintFileName,
          doc.CommonSolverOption.kPrintFileName.doc)
      .value("kPrintToConsole", CommonSolverOption::kPrintToConsole,
          doc.CommonSolverOption.kPrintToConsole.doc);
}

void BindMathematicalProgram(py::module m) {
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
      .def("num_suboptimal_solution()",
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

  py::class_<MathematicalProgram> prog_cls(
      m, "MathematicalProgram", doc.MathematicalProgram.doc);
  prog_cls.def(py::init<>(), doc.MathematicalProgram.ctor.doc);
  DefClone(&prog_cls);
  prog_cls  // BR
      .def("__str__", &MathematicalProgram::to_string,
          doc.MathematicalProgram.to_string.doc)
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
      .def("NewNonnegativePolynomial",
          static_cast<std::pair<Polynomial, MatrixXDecisionVariable> (
              MathematicalProgram::*)(
              const Eigen::Ref<const VectorX<symbolic::Monomial>>&,
              MathematicalProgram::NonnegativePolynomial)>(
              &MathematicalProgram::NewNonnegativePolynomial),
          py::arg("monomial_basis"), py::arg("type"),
          doc.MathematicalProgram.NewNonnegativePolynomial
              .doc_2args_monomial_basis_type)
      .def("NewNonnegativePolynomial",
          static_cast<symbolic::Polynomial (MathematicalProgram::*)(
              const Eigen::Ref<const MatrixX<symbolic::Variable>>&,
              const Eigen::Ref<const VectorX<symbolic::Monomial>>&,
              MathematicalProgram::NonnegativePolynomial)>(
              &MathematicalProgram::NewNonnegativePolynomial),
          py::arg("gramian"), py::arg("monomial_basis"), py::arg("type"),
          doc.MathematicalProgram.NewNonnegativePolynomial
              .doc_3args_gramian_monomial_basis_type)
      .def("NewNonnegativePolynomial",
          static_cast<std::pair<symbolic::Polynomial, MatrixXDecisionVariable> (
              MathematicalProgram::*)(const symbolic::Variables&, int degree,
              MathematicalProgram::NonnegativePolynomial)>(
              &MathematicalProgram::NewNonnegativePolynomial),
          py::arg("indeterminates"), py::arg("degree"), py::arg("type"),
          doc.MathematicalProgram.NewNonnegativePolynomial
              .doc_3args_indeterminates_degree_type)
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
      .def("AddIndeterminates", &MathematicalProgram::AddIndeterminates,
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
      .def("AddCost",
          static_cast<Binding<Cost> (MathematicalProgram::*)(
              const Expression&)>(&MathematicalProgram::AddCost),
          // N.B. There is no corresponding C++ method, so the docstring here
          // is a literal, not a reference to documentation_pybind.h
          "Adds a cost expression.")
      .def(
          "AddCost",
          [](MathematicalProgram* self, const std::shared_ptr<Cost>& obj,
              const Eigen::Ref<const VectorXDecisionVariable>& vars) {
            return self->AddCost(obj, vars);
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
          overload_cast_explicit<Binding<QuadraticCost>,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&>(
              &MathematicalProgram::AddQuadraticErrorCost),
          py::arg("Q"), py::arg("x_desired"), py::arg("vars"),
          doc.MathematicalProgram.AddQuadraticErrorCost.doc)
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
      .def("AddConstraint",
          static_cast<Binding<Constraint> (MathematicalProgram::*)(
              std::shared_ptr<Constraint>,
              const Eigen::Ref<const VectorXDecisionVariable>& vars)>(
              &MathematicalProgram::AddConstraint),
          py::arg("constraint"), py::arg("vars"),
          doc.MathematicalProgram.AddConstraint.doc_2args_con_vars)
      .def(
          "AddConstraint",
          [](MathematicalProgram* self,
              const Eigen::Ref<const MatrixX<Formula>>& formulas) {
            return self->AddConstraint(formulas);
          },
          py::arg("formulas"),
          doc.MathematicalProgram.AddConstraint.doc_matrix_formula)
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
      .def(
          "AddLinearConstraint",
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
          py::arg("Aeq"), py::arg("beq"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearEqualityConstraint
              .doc_3args_Aeq_beq_vars)
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
      .def(
          "AddBoundingBoxConstraint",
          [](MathematicalProgram* self, double lb, double ub,
              const Eigen::Ref<const MatrixX<symbolic::Variable>>& vars) {
            return self->AddBoundingBoxConstraint(lb, ub, vars);
          },
          doc.MathematicalProgram.AddBoundingBoxConstraint
              .doc_3args_double_double_constEigenMatrixBase)
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
      .def("AddLinearComplementarityConstraint",
          static_cast<Binding<LinearComplementarityConstraint> (
              MathematicalProgram::*)(const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::VectorXd>&,
              const Eigen::Ref<const VectorXDecisionVariable>&)>(
              &MathematicalProgram::AddLinearComplementarityConstraint),
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
          "AddLinearMatrixInequalityConstraint",
          [](MathematicalProgram* self,
              const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
              const Eigen::Ref<const VectorXDecisionVariable>& vars) {
            return self->AddLinearMatrixInequalityConstraint(F, vars);
          },
          py::arg("F"), py::arg("vars"),
          doc.MathematicalProgram.AddLinearMatrixInequalityConstraint.doc)
      .def("AddPositiveDiagonallyDominantMatrixConstraint",
          &MathematicalProgram::AddPositiveDiagonallyDominantMatrixConstraint,
          py::arg("X"),
          doc.MathematicalProgram.AddPositiveDiagonallyDominantMatrixConstraint
              .doc)
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
      .def("AddSosConstraint",
          static_cast<MatrixXDecisionVariable (MathematicalProgram::*)(
              const Polynomial&, const Eigen::Ref<const VectorX<Monomial>>&)>(
              &MathematicalProgram::AddSosConstraint),
          py::arg("p"), py::arg("monomial_basis"),
          doc.MathematicalProgram.AddSosConstraint.doc_2args_p_monomial_basis)
      .def("AddSosConstraint",
          static_cast<
              std::pair<MatrixXDecisionVariable, VectorX<symbolic::Monomial>> (
                  MathematicalProgram::*)(const Polynomial&)>(
              &MathematicalProgram::AddSosConstraint),
          py::arg("p"), doc.MathematicalProgram.AddSosConstraint.doc_1args_p)
      .def("AddSosConstraint",
          static_cast<MatrixXDecisionVariable (MathematicalProgram::*)(
              const Expression&, const Eigen::Ref<const VectorX<Monomial>>&)>(
              &MathematicalProgram::AddSosConstraint),
          py::arg("e"), py::arg("monomial_basis"),
          doc.MathematicalProgram.AddSosConstraint.doc_2args_e_monomial_basis)
      .def("AddSosConstraint",
          static_cast<
              std::pair<MatrixXDecisionVariable, VectorX<symbolic::Monomial>> (
                  MathematicalProgram::*)(const Expression&)>(
              &MathematicalProgram::AddSosConstraint),
          py::arg("e"), doc.MathematicalProgram.AddSosConstraint.doc_1args_e)
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
      .def(
          "SetDecisionVariableValueInVector",
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
      .def(
          "SetDecisionVariableValueInVector",
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
      // TODO(m-chaturvedi) Add Pybind11 documentation.
      .def("GetSolverOptions",
          [](MathematicalProgram& prog, SolverId solver_id) {
            py::dict out;
            py::object update = out.attr("update");
            update(prog.GetSolverOptionsDouble(solver_id));
            update(prog.GetSolverOptionsInt(solver_id));
            update(prog.GetSolverOptionsStr(solver_id));
            return out;
          })
      .def("GetSolverOptions",
          [](MathematicalProgram& prog, SolverType solver_type) {
            py::dict out;
            py::object update = out.attr("update");
            const SolverId id = SolverTypeConverter::TypeToId(solver_type);
            update(prog.GetSolverOptionsDouble(id));
            update(prog.GetSolverOptionsInt(id));
            update(prog.GetSolverOptionsStr(id));
            return out;
          })
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
          doc.MathematicalProgram.indeterminates.doc)
      .def("indeterminate", &MathematicalProgram::indeterminate, py::arg("i"),
          doc.MathematicalProgram.indeterminate.doc)
      .def("indeterminates_index", &MathematicalProgram::indeterminates_index,
          doc.MathematicalProgram.indeterminates_index.doc)
      .def("decision_variables", &MathematicalProgram::decision_variables,
          doc.MathematicalProgram.decision_variables.doc)
      .def("decision_variable", &MathematicalProgram::decision_variable,
          py::arg("i"), doc.MathematicalProgram.decision_variable.doc)
      .def("decision_variable_index",
          &MathematicalProgram::decision_variable_index,
          doc.MathematicalProgram.decision_variable_index.doc)
      .def("RemoveCost", &MathematicalProgram::RemoveCost, py::arg("cost"),
          doc.MathematicalProgram.RemoveCost.doc)
      .def("RemoveConstraint", &MathematicalProgram::RemoveConstraint,
          py::arg("constraint"), doc.MathematicalProgram.RemoveConstraint.doc);

  py::enum_<MathematicalProgram::NonnegativePolynomial>(prog_cls,
      "NonnegativePolynomial",
      doc.MathematicalProgram.NonnegativePolynomial.doc)
      .value("kSos", MathematicalProgram::NonnegativePolynomial::kSos,
          doc.MathematicalProgram.NonnegativePolynomial.kSos.doc)
      .value("kSdsos", MathematicalProgram::NonnegativePolynomial::kSdsos,
          doc.MathematicalProgram.NonnegativePolynomial.kSdsos.doc)
      .value("kDsos", MathematicalProgram::NonnegativePolynomial::kDsos,
          doc.MathematicalProgram.NonnegativePolynomial.kDsos.doc);

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
}  // NOLINT(readability/fn_size)

void BindEvaluatorsAndBindings(py::module m) {
  constexpr auto& doc = pydrake_doc.drake.solvers;
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
            cls_doc.set_description.doc)
        .def("SetGradientSparsityPattern", &Class::SetGradientSparsityPattern,
            py::arg("gradient_sparsity_pattern"),
            cls_doc.SetGradientSparsityPattern.doc)
        .def("gradient_sparsity_pattern", &Class::gradient_sparsity_pattern,
            cls_doc.gradient_sparsity_pattern.doc);
    auto bind_eval = [&cls, &cls_doc](auto dummy_x, auto dummy_y) {
      using T_x = decltype(dummy_x);
      using T_y = decltype(dummy_y);
      cls.def(
          "Eval",
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

  auto evaluator_binding = RegisterBinding<EvaluatorBase>(&m);
  DefBindingCastConstructor<EvaluatorBase>(&evaluator_binding);

  py::class_<Constraint, EvaluatorBase, std::shared_ptr<Constraint>>(
      m, "Constraint", doc.Constraint.doc)
      .def("num_constraints", &Constraint::num_constraints,
          doc.Constraint.num_constraints.doc)
      .def("lower_bound", &Constraint::lower_bound,
          doc.Constraint.lower_bound.doc)
      .def("upper_bound", &Constraint::upper_bound,
          doc.Constraint.upper_bound.doc)
      .def(
          "CheckSatisfiedVectorized",
          [](Constraint& self, const Eigen::Ref<const Eigen::MatrixXd>& x,
              double tol) {
            DRAKE_DEMAND(x.rows() == self.num_vars());
            std::vector<bool> satisfied(x.cols());
            for (int i = 0; i < x.cols(); ++i) {
              satisfied[i] = self.CheckSatisfied(x.col(i), tol);
            }
            return satisfied;
          },
          py::arg("x"), py::arg("tol"),
          R"""(A "vectorized" version of CheckSatisfied.  It evaluates the constraint for every column of ``x``. )""")
      .def(
          "CheckSatisfied",
          [](Constraint& self, const Eigen::Ref<const Eigen::VectorXd>& x,
              double tol) { return self.CheckSatisfied(x, tol); },
          py::arg("x"), py::arg("tol") = 1E-6,
          doc.Constraint.CheckSatisfied.doc)
      .def(
          "CheckSatisfied",
          [](Constraint& self, const Eigen::Ref<const AutoDiffVecXd>& x,
              double tol) { return self.CheckSatisfied(x, tol); },
          py::arg("x"), py::arg("tol") = 1E-6,
          doc.Constraint.CheckSatisfied.doc)
      .def(
          "CheckSatisfied",
          [](Constraint& self,
              const Eigen::Ref<const VectorX<symbolic::Variable>>& x) {
            return self.CheckSatisfied(x);
          },
          py::arg("x"), doc.Constraint.CheckSatisfied.doc);

  py::class_<PyFunctionConstraint, Constraint,
      std::shared_ptr<PyFunctionConstraint>>(m, "PyFunctionConstraint",
      "Constraint with its evaluator as a Python function")
      .def("UpdateLowerBound", &PyFunctionConstraint::UpdateLowerBound,
          py::arg("new_lb"), "Update the lower bound of the constraint.")
      .def("UpdateUpperBound", &PyFunctionConstraint::UpdateUpperBound,
          py::arg("new_ub"), "Update the upper bound of the constraint.")
      .def("set_bounds", &PyFunctionConstraint::set_bounds,
          py::arg("lower_bound"), py::arg("upper_bound"),
          "Set both the lower and upper bounds of the constraint.");

  py::class_<LinearConstraint, Constraint, std::shared_ptr<LinearConstraint>>(
      m, "LinearConstraint", doc.LinearConstraint.doc)
      .def(py::init([](const Eigen::MatrixXd& A, const Eigen::VectorXd& lb,
                        const Eigen::VectorXd& ub) {
        return std::make_unique<LinearConstraint>(A, lb, ub);
      }),
          py::arg("A"), py::arg("lb"), py::arg("ub"),
          doc.LinearConstraint.ctor.doc)
      .def("A", &LinearConstraint::A, doc.LinearConstraint.A.doc)
      .def(
          "UpdateCoefficients",
          [](LinearConstraint& self, const Eigen::MatrixXd& new_A,
              const Eigen::VectorXd& new_lb, const Eigen::VectorXd& new_ub) {
            self.UpdateCoefficients(new_A, new_lb, new_ub);
          },
          py::arg("new_A"), py::arg("new_lb"), py::arg("new_ub"),
          doc.LinearConstraint.UpdateCoefficients.doc)
      .def(
          "UpdateLowerBound",
          [](LinearConstraint& self, const Eigen::VectorXd& new_lb) {
            self.UpdateLowerBound(new_lb);
          },
          py::arg("new_lb"), doc.Constraint.UpdateLowerBound.doc)
      .def(
          "UpdateUpperBound",
          [](LinearConstraint& self, const Eigen::VectorXd& new_ub) {
            self.UpdateUpperBound(new_ub);
          },
          py::arg("new_ub"), doc.Constraint.UpdateUpperBound.doc)
      .def(
          "set_bounds",
          [](LinearConstraint& self, const Eigen::VectorXd& new_lb,
              const Eigen::VectorXd& new_ub) {
            self.set_bounds(new_lb, new_ub);
          },
          py::arg("new_lb"), py::arg("new_ub"), doc.Constraint.set_bounds.doc);

  py::class_<LorentzConeConstraint, Constraint,
      std::shared_ptr<LorentzConeConstraint>>
      lorentz_cone_cls(
          m, "LorentzConeConstraint", doc.LorentzConeConstraint.doc);

  py::enum_<LorentzConeConstraint::EvalType>(
      lorentz_cone_cls, "EvalType", doc.LorentzConeConstraint.EvalType.doc)
      .value("kConvex", LorentzConeConstraint::EvalType::kConvex,
          doc.LorentzConeConstraint.EvalType.kConvex.doc)
      .value("kConvexSmooth", LorentzConeConstraint::EvalType::kConvexSmooth,
          doc.LorentzConeConstraint.EvalType.kConvexSmooth.doc)
      .value("kNonconvex", LorentzConeConstraint::EvalType::kNonconvex,
          doc.LorentzConeConstraint.EvalType.kNonconvex.doc);

  lorentz_cone_cls
      .def(py::init([](const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                        LorentzConeConstraint::EvalType eval_type) {
        return std::make_unique<LorentzConeConstraint>(A, b, eval_type);
      }),
          py::arg("A"), py::arg("b"),
          py::arg("eval_type") = LorentzConeConstraint::EvalType::kConvexSmooth,
          doc.LorentzConeConstraint.ctor.doc)
      .def("A", &LorentzConeConstraint::A, doc.LorentzConeConstraint.A.doc)
      .def("b", &LorentzConeConstraint::b, doc.LorentzConeConstraint.b.doc)
      .def("eval_type", &LorentzConeConstraint::eval_type,
          doc.LorentzConeConstraint.eval_type.doc);

  py::class_<RotatedLorentzConeConstraint, Constraint,
      std::shared_ptr<RotatedLorentzConeConstraint>>(
      m, "RotatedLorentzConeConstraint", doc.RotatedLorentzConeConstraint.doc)
      .def(py::init([](const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
        return std::make_unique<RotatedLorentzConeConstraint>(A, b);
      }),
          py::arg("A"), py::arg("b"), doc.RotatedLorentzConeConstraint.ctor.doc)
      .def("A", &RotatedLorentzConeConstraint::A,
          doc.RotatedLorentzConeConstraint.A.doc)
      .def("b", &RotatedLorentzConeConstraint::b,
          doc.RotatedLorentzConeConstraint.b.doc);

  py::class_<LinearEqualityConstraint, LinearConstraint,
      std::shared_ptr<LinearEqualityConstraint>>(
      m, "LinearEqualityConstraint", doc.LinearEqualityConstraint.doc)
      .def(py::init([](const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq) {
        return std::make_unique<LinearEqualityConstraint>(Aeq, beq);
      }),
          py::arg("Aeq"), py::arg("beq"),
          doc.LinearEqualityConstraint.ctor
              .doc_2args_constEigenMatrixBase_constEigenMatrixBase)
      .def(py::init([](const Eigen::RowVectorXd& a, double beq) {
        return std::make_unique<LinearEqualityConstraint>(a, beq);
      }),
          py::arg("a"), py::arg("beq"),
          doc.LinearEqualityConstraint.ctor.doc_2args_a_beq)
      .def(
          "UpdateCoefficients",
          [](LinearEqualityConstraint& self,  // BR
              const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq) {
            self.UpdateCoefficients(Aeq, beq);
          },
          py::arg("Aeq"), py::arg("beq"),
          doc.LinearEqualityConstraint.UpdateCoefficients.doc);

  py::class_<BoundingBoxConstraint, LinearConstraint,
      std::shared_ptr<BoundingBoxConstraint>>(
      m, "BoundingBoxConstraint", doc.BoundingBoxConstraint.doc)
      .def(py::init([](const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) {
        return std::make_unique<BoundingBoxConstraint>(lb, ub);
      }),
          py::arg("lb"), py::arg("ub"), doc.BoundingBoxConstraint.ctor.doc);

  py::class_<QuadraticConstraint, Constraint,
      std::shared_ptr<QuadraticConstraint>>(
      m, "QuadraticConstraint", doc.QuadraticConstraint.doc)
      .def(py::init([](const Eigen::Ref<const Eigen::MatrixXd>& Q0,
                        const Eigen::Ref<const Eigen::VectorXd>& b, double lb,
                        double ub) {
        return std::make_unique<QuadraticConstraint>(Q0, b, lb, ub);
      }),
          py::arg("Q0"), py::arg("b"), py::arg("lb"), py::arg("ub"),
          doc.QuadraticConstraint.ctor.doc)
      .def("Q", &QuadraticConstraint::Q, doc.QuadraticConstraint.Q.doc)
      .def("b", &QuadraticConstraint::b, doc.QuadraticConstraint.b.doc);

  py::class_<PositiveSemidefiniteConstraint, Constraint,
      std::shared_ptr<PositiveSemidefiniteConstraint>>(m,
      "PositiveSemidefiniteConstraint", doc.PositiveSemidefiniteConstraint.doc)
      .def(py::init([](int rows) {
        return std::make_unique<PositiveSemidefiniteConstraint>(rows);
      }),
          py::arg("rows"), doc.PositiveSemidefiniteConstraint.ctor.doc)
      .def("matrix_rows", &PositiveSemidefiniteConstraint::matrix_rows,
          doc.PositiveSemidefiniteConstraint.matrix_rows.doc);

  py::class_<LinearMatrixInequalityConstraint, Constraint,
      std::shared_ptr<LinearMatrixInequalityConstraint>>(m,
      "LinearMatrixInequalityConstraint",
      doc.LinearMatrixInequalityConstraint.doc)
      .def(py::init([](const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
                        double symmetry_tolerance) {
        return std::make_unique<LinearMatrixInequalityConstraint>(
            F, symmetry_tolerance);
      }),
          py::arg("F"), py::arg("symmetry_tolerance") = 1E-10,
          doc.LinearMatrixInequalityConstraint.ctor.doc)
      .def("F", &LinearMatrixInequalityConstraint::F,
          doc.LinearMatrixInequalityConstraint.F.doc)
      .def("matrix_rows", &LinearMatrixInequalityConstraint::matrix_rows,
          doc.LinearMatrixInequalityConstraint.matrix_rows.doc);

  py::class_<LinearComplementarityConstraint, Constraint,
      std::shared_ptr<LinearComplementarityConstraint>>(m,
      "LinearComplementarityConstraint",
      doc.LinearComplementarityConstraint.doc);

  py::class_<ExponentialConeConstraint, Constraint,
      std::shared_ptr<ExponentialConeConstraint>>(
      m, "ExponentialConeConstraint", doc.ExponentialConeConstraint.doc)
      .def(py::init([](const Eigen::MatrixXd& A, const Eigen::Vector3d& b) {
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();
        return std::make_unique<ExponentialConeConstraint>(A_sparse, b);
      }),
          py::arg("A"), py::arg("b"), doc.ExponentialConeConstraint.ctor.doc)
      .def(
          "A",
          [](const ExponentialConeConstraint& self) {
            return Eigen::MatrixXd(self.A());
          },
          doc.ExponentialConeConstraint.A.doc)
      .def("b", &ExponentialConeConstraint::b,
          doc.ExponentialConeConstraint.b.doc);

  auto constraint_binding = RegisterBinding<Constraint>(&m);
  DefBindingCastConstructor<Constraint>(&constraint_binding);
  RegisterBinding<LinearConstraint>(&m);
  RegisterBinding<LorentzConeConstraint>(&m);
  RegisterBinding<RotatedLorentzConeConstraint>(&m);
  RegisterBinding<LinearEqualityConstraint>(&m);
  RegisterBinding<BoundingBoxConstraint>(&m);
  RegisterBinding<PositiveSemidefiniteConstraint>(&m);
  RegisterBinding<LinearMatrixInequalityConstraint>(&m);
  RegisterBinding<LinearComplementarityConstraint>(&m);
  RegisterBinding<ExponentialConeConstraint>(&m);

  // Mirror procedure for costs
  py::class_<Cost, EvaluatorBase, std::shared_ptr<Cost>> cost(
      m, "Cost", doc.Cost.doc);

  py::class_<LinearCost, Cost, std::shared_ptr<LinearCost>>(
      m, "LinearCost", doc.LinearCost.doc)
      .def(py::init([](const Eigen::VectorXd& a, double b) {
        return std::make_unique<LinearCost>(a, b);
      }),
          py::arg("a"), py::arg("b"), doc.LinearCost.ctor.doc)
      .def("a", &LinearCost::a, doc.LinearCost.a.doc)
      .def("b", &LinearCost::b, doc.LinearCost.b.doc)
      .def(
          "UpdateCoefficients",
          [](LinearCost& self, const Eigen::VectorXd& new_a, double new_b) {
            self.UpdateCoefficients(new_a, new_b);
          },
          py::arg("new_a"), py::arg("new_b") = 0,
          doc.LinearCost.UpdateCoefficients.doc);

  py::class_<QuadraticCost, Cost, std::shared_ptr<QuadraticCost>>(
      m, "QuadraticCost", doc.QuadraticCost.doc)
      .def(py::init([](const Eigen::MatrixXd& Q, const Eigen::VectorXd& b,
                        double c, std::optional<bool> is_convex) {
        return std::make_unique<QuadraticCost>(Q, b, c, is_convex);
      }),
          py::arg("Q"), py::arg("b"), py::arg("c"),
          py::arg("is_convex") = py::none(), doc.QuadraticCost.ctor.doc)
      .def("Q", &QuadraticCost::Q, doc.QuadraticCost.Q.doc)
      .def("b", &QuadraticCost::b, doc.QuadraticCost.b.doc)
      .def("c", &QuadraticCost::c, doc.QuadraticCost.c.doc)
      .def("is_convex", &QuadraticCost::is_convex,
          doc.QuadraticCost.is_convex.doc)
      .def(
          "UpdateCoefficients",
          [](QuadraticCost& self, const Eigen::MatrixXd& new_Q,
              const Eigen::VectorXd& new_b, double new_c,
              std::optional<bool> is_convex) {
            self.UpdateCoefficients(new_Q, new_b, new_c, is_convex);
          },
          py::arg("new_Q"), py::arg("new_b"), py::arg("new_c") = 0,
          py::arg("is_convex") = py::none(),
          doc.QuadraticCost.UpdateCoefficients.doc);

  py::class_<L2NormCost, Cost, std::shared_ptr<L2NormCost>>(
      m, "L2NormCost", doc.L2NormCost.doc)
      .def(py::init([](const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
        return std::make_unique<L2NormCost>(A, b);
      }),
          py::arg("A"), py::arg("b"), doc.L2NormCost.ctor.doc)
      .def("A", &L2NormCost::A, doc.L2NormCost.A.doc)
      .def("b", &L2NormCost::b, doc.L2NormCost.b.doc)
      .def(
          "UpdateCoefficients",
          [](L2NormCost& self, const Eigen::MatrixXd& new_A,
              const Eigen::VectorXd& new_b) {
            self.UpdateCoefficients(new_A, new_b);
          },
          py::arg("new_A"), py::arg("new_b") = 0,
          doc.L2NormCost.UpdateCoefficients.doc);

  auto cost_binding = RegisterBinding<Cost>(&m);
  DefBindingCastConstructor<Cost>(&cost_binding);
  RegisterBinding<LinearCost>(&m);
  RegisterBinding<QuadraticCost>(&m);
  RegisterBinding<L2NormCost>(&m);

  py::class_<VisualizationCallback, EvaluatorBase,
      std::shared_ptr<VisualizationCallback>>(
      m, "VisualizationCallback", doc.VisualizationCallback.doc);

  RegisterBinding<VisualizationCallback>(&m);
}  // NOLINT(readability/fn_size)

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
      .def("Solve",
          py::overload_cast<const MathematicalProgram&,
              const std::optional<Eigen::VectorXd>&,
              const std::optional<SolverOptions>&>(&solvers::Solve),
          py::arg("prog"), py::arg("initial_guess") = py::none(),
          py::arg("solver_options") = py::none(), doc.Solve.doc_3args)
      .def("GetProgramType", &solvers::GetProgramType, doc.GetProgramType.doc);
}

PYBIND11_MODULE(mathematicalprogram, m) {
  m.doc() = R"""(
Bindings for MathematicalProgram

If you are formulating constraints using symbolic formulas, please review the
top-level documentation for :py:mod:`pydrake.math`.
)""";

  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.symbolic");

  BindEvaluatorsAndBindings(m);
  BindSolverInterfaceAndFlags(m);
  BindMathematicalProgram(m);
  BindFreeFunctions(m);

  DefTesting(m.def_submodule("_testing"));

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
