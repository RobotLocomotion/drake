#include <memory>

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/evaluator_base.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/binding.h"

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
using solvers::PerspectiveQuadraticCost;
using solvers::PositiveSemidefiniteConstraint;
using solvers::QuadraticConstraint;
using solvers::QuadraticCost;
using solvers::RotatedLorentzConeConstraint;
using solvers::VectorXDecisionVariable;
using solvers::VisualizationCallback;
namespace {

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
      .def(
          py::init<const std::shared_ptr<C>&, const VectorXDecisionVariable&>(),
          py::arg("c"), py::arg("v"), cls_doc.ctor.doc)
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
  py::module m = py::module::import("pydrake.solvers._extra");
  m.attr("_check_array_type")(var_name, x, GetPyParam<T>()[0]);
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
          // TODO(hongkai.dai): use new_lb and new_ub as kwargs.
          py::arg("lower_bound"), py::arg("upper_bound"),
          "Set both the lower and upper bounds of the constraint.");

  py::class_<LinearConstraint, Constraint, std::shared_ptr<LinearConstraint>>
      linear_constraint_cls(m, "LinearConstraint", doc.LinearConstraint.doc);
  linear_constraint_cls
      .def(py::init([](const Eigen::MatrixXd& A, const Eigen::VectorXd& lb,
                        const Eigen::VectorXd& ub) {
        return std::make_unique<LinearConstraint>(A, lb, ub);
      }),
          py::arg("A"), py::arg("lb"), py::arg("ub"),
          doc.LinearConstraint.ctor.doc_dense_A)
      .def(py::init([](const Eigen::SparseMatrix<double>& A,
                        const Eigen::Ref<const Eigen::VectorXd>& lb,
                        const Eigen::Ref<const Eigen::VectorXd>& ub) {
        return std::make_unique<LinearConstraint>(A, lb, ub);
      }),
          py::arg("A"), py::arg("lb"), py::arg("ub"),
          doc.LinearConstraint.ctor.doc_sparse_A)
      .def("GetDenseA", &LinearConstraint::GetDenseA,
          doc.LinearConstraint.GetDenseA.doc)
      .def("get_sparse_A", &LinearConstraint::get_sparse_A,
          doc.LinearConstraint.get_sparse_A.doc)
      .def(
          "UpdateCoefficients",
          [](LinearConstraint& self, const Eigen::MatrixXd& new_A,
              const Eigen::VectorXd& new_lb, const Eigen::VectorXd& new_ub) {
            self.UpdateCoefficients(new_A, new_lb, new_ub);
          },
          py::arg("new_A"), py::arg("new_lb"), py::arg("new_ub"),
          doc.LinearConstraint.UpdateCoefficients.doc_dense_A)
      .def(
          "UpdateCoefficients",
          [](LinearConstraint& self, const Eigen::SparseMatrix<double>& new_A,
              const Eigen::VectorXd& new_lb, const Eigen::VectorXd& new_ub) {
            self.UpdateCoefficients(new_A, new_lb, new_ub);
          },
          py::arg("new_A"), py::arg("new_lb"), py::arg("new_ub"),
          doc.LinearConstraint.UpdateCoefficients.doc_sparse_A)
      .def("RemoveTinyCoefficient", &LinearConstraint::RemoveTinyCoefficient,
          py::arg("tol"), doc.LinearConstraint.RemoveTinyCoefficient.doc)
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
          doc.LorentzConeConstraint.eval_type.doc)
      .def("UpdateCoefficients", &LorentzConeConstraint::UpdateCoefficients,
          py::arg("new_A"), py::arg("new_b"),
          doc.LorentzConeConstraint.UpdateCoefficients.doc);

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
          doc.RotatedLorentzConeConstraint.b.doc)
      .def("UpdateCoefficients",
          &RotatedLorentzConeConstraint::UpdateCoefficients, py::arg("new_A"),
          py::arg("new_b"),
          doc.RotatedLorentzConeConstraint.UpdateCoefficients.doc);

  py::class_<LinearEqualityConstraint, LinearConstraint,
      std::shared_ptr<LinearEqualityConstraint>>(
      m, "LinearEqualityConstraint", doc.LinearEqualityConstraint.doc)
      .def(py::init([](const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq) {
        return std::make_unique<LinearEqualityConstraint>(Aeq, beq);
      }),
          py::arg("Aeq"), py::arg("beq"),
          doc.LinearEqualityConstraint.ctor.doc_dense_Aeq)
      .def(py::init([](const Eigen::SparseMatrix<double>& Aeq,
                        const Eigen::VectorXd& beq) {
        return std::make_unique<LinearEqualityConstraint>(Aeq, beq);
      }),
          py::arg("Aeq"), py::arg("beq"),
          doc.LinearEqualityConstraint.ctor.doc_sparse_Aeq)
      .def(py::init([](const Eigen::RowVectorXd& a, double beq) {
        return std::make_unique<LinearEqualityConstraint>(a, beq);
      }),
          py::arg("a"), py::arg("beq"),
          doc.LinearEqualityConstraint.ctor.doc_row_a)
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
      std::shared_ptr<QuadraticConstraint>>
      quadratic_constraint_cls(
          m, "QuadraticConstraint", doc.QuadraticConstraint.doc);

  py::enum_<QuadraticConstraint::HessianType>(quadratic_constraint_cls,
      "HessianType", doc.QuadraticConstraint.HessianType.doc)
      .value("kPositiveSemidefinite",
          QuadraticConstraint::HessianType::kPositiveSemidefinite,
          doc.QuadraticConstraint.HessianType.kPositiveSemidefinite.doc)
      .value("kNegativeSemidefinite",
          QuadraticConstraint::HessianType::kNegativeSemidefinite,
          doc.QuadraticConstraint.HessianType.kNegativeSemidefinite.doc)
      .value("kIndefinite", QuadraticConstraint::HessianType::kIndefinite,
          doc.QuadraticConstraint.HessianType.kIndefinite.doc);

  quadratic_constraint_cls
      .def(py::init([](const Eigen::Ref<const Eigen::MatrixXd>& Q0,
                        const Eigen::Ref<const Eigen::VectorXd>& b, double lb,
                        double ub,
                        std::optional<QuadraticConstraint::HessianType>
                            hessian_type) {
        return std::make_unique<QuadraticConstraint>(
            Q0, b, lb, ub, hessian_type);
      }),
          py::arg("Q0"), py::arg("b"), py::arg("lb"), py::arg("ub"),
          py::arg("hessian_type") = std::nullopt,
          doc.QuadraticConstraint.ctor.doc)
      .def("Q", &QuadraticConstraint::Q, py_rvp::reference_internal,
          doc.QuadraticConstraint.Q.doc)
      .def("b", &QuadraticConstraint::b, py_rvp::reference_internal,
          doc.QuadraticConstraint.b.doc)
      .def("is_convex", &QuadraticConstraint::is_convex,
          doc.QuadraticConstraint.is_convex.doc)
      .def(
          "UpdateCoefficients",
          [](QuadraticConstraint& self,
              const Eigen::Ref<const Eigen::MatrixXd>& new_Q,
              const Eigen::Ref<const Eigen::VectorXd>& new_b,
              std::optional<QuadraticConstraint::HessianType> hessian_type) {
            self.UpdateCoefficients(new_Q, new_b, hessian_type);
          },
          py::arg("new_Q"), py::arg("new_b"),
          py::arg("hessian_type") = std::nullopt,
          doc.QuadraticConstraint.UpdateCoefficients.doc)
      .def("hessian_type", &QuadraticConstraint::hessian_type,
          doc.QuadraticConstraint.hessian_type.doc);

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
      doc.LinearComplementarityConstraint.doc)
      .def("M", &LinearComplementarityConstraint::M,
          doc.LinearComplementarityConstraint.M.doc)
      .def("q", &LinearComplementarityConstraint::q,
          doc.LinearComplementarityConstraint.q.doc);

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

  py::class_<ExpressionConstraint, Constraint,
      std::shared_ptr<ExpressionConstraint>>(
      m, "ExpressionConstraint", doc.ExpressionConstraint.doc)
      .def(py::init<const Eigen::Ref<const VectorX<symbolic::Expression>>&,
               const Eigen::Ref<const Eigen::VectorXd>&,
               const Eigen::Ref<const Eigen::VectorXd>&>(),
          py::arg("v"), py::arg("lb"), py::arg("ub"),
          doc.ExpressionConstraint.ctor.doc)
      .def("expressions", &ExpressionConstraint::expressions,
          // dtype = object arrays must be copied, and cannot be referenced.
          py_rvp::copy, doc.ExpressionConstraint.expressions.doc)
      .def("vars", &ExpressionConstraint::vars,
          // dtype = object arrays must be copied, and cannot be referenced.
          py_rvp::copy, doc.ExpressionConstraint.vars.doc);

  auto constraint_binding = RegisterBinding<Constraint>(&m);
  DefBindingCastConstructor<Constraint>(&constraint_binding);
  RegisterBinding<LinearConstraint>(&m);
  RegisterBinding<QuadraticConstraint>(&m);
  RegisterBinding<LorentzConeConstraint>(&m);
  RegisterBinding<RotatedLorentzConeConstraint>(&m);
  RegisterBinding<LinearEqualityConstraint>(&m);
  RegisterBinding<BoundingBoxConstraint>(&m);
  RegisterBinding<PositiveSemidefiniteConstraint>(&m);
  RegisterBinding<LinearMatrixInequalityConstraint>(&m);
  RegisterBinding<LinearComplementarityConstraint>(&m);
  RegisterBinding<ExponentialConeConstraint>(&m);
  // TODO(russt): PolynomialConstraint currently uses common::Polynomial, not
  // symbolic::Polynomial. Decide whether we want to bind the current c++
  // implementation as is, or convert it to symbolic::Polynomial first.
  RegisterBinding<ExpressionConstraint>(&m);

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

  py::class_<L1NormCost, Cost, std::shared_ptr<L1NormCost>>(
      m, "L1NormCost", doc.L1NormCost.doc)
      .def(py::init([](const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
        return std::make_unique<L1NormCost>(A, b);
      }),
          py::arg("A"), py::arg("b"), doc.L1NormCost.ctor.doc)
      .def("A", &L1NormCost::A, doc.L1NormCost.A.doc)
      .def("b", &L1NormCost::b, doc.L1NormCost.b.doc)
      .def(
          "UpdateCoefficients",
          [](L1NormCost& self, const Eigen::MatrixXd& new_A,
              const Eigen::VectorXd& new_b) {
            self.UpdateCoefficients(new_A, new_b);
          },
          py::arg("new_A"), py::arg("new_b") = 0,
          doc.L1NormCost.UpdateCoefficients.doc);

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

  py::class_<LInfNormCost, Cost, std::shared_ptr<LInfNormCost>>(
      m, "LInfNormCost", doc.LInfNormCost.doc)
      .def(py::init([](const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
        return std::make_unique<LInfNormCost>(A, b);
      }),
          py::arg("A"), py::arg("b"), doc.LInfNormCost.ctor.doc)
      .def("A", &LInfNormCost::A, doc.LInfNormCost.A.doc)
      .def("b", &LInfNormCost::b, doc.LInfNormCost.b.doc)
      .def(
          "UpdateCoefficients",
          [](LInfNormCost& self, const Eigen::MatrixXd& new_A,
              const Eigen::VectorXd& new_b) {
            self.UpdateCoefficients(new_A, new_b);
          },
          py::arg("new_A"), py::arg("new_b") = 0,
          doc.LInfNormCost.UpdateCoefficients.doc);

  py::class_<PerspectiveQuadraticCost, Cost,
      std::shared_ptr<PerspectiveQuadraticCost>>(
      m, "PerspectiveQuadraticCost", doc.PerspectiveQuadraticCost.doc)
      .def(py::init([](const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
        return std::make_unique<PerspectiveQuadraticCost>(A, b);
      }),
          py::arg("A"), py::arg("b"), doc.PerspectiveQuadraticCost.ctor.doc)
      .def(
          "A", &PerspectiveQuadraticCost::A, doc.PerspectiveQuadraticCost.A.doc)
      .def(
          "b", &PerspectiveQuadraticCost::b, doc.PerspectiveQuadraticCost.b.doc)
      .def(
          "UpdateCoefficients",
          [](PerspectiveQuadraticCost& self, const Eigen::MatrixXd& new_A,
              const Eigen::VectorXd& new_b) {
            self.UpdateCoefficients(new_A, new_b);
          },
          py::arg("new_A"), py::arg("new_b"),
          doc.PerspectiveQuadraticCost.UpdateCoefficients.doc);

  py::class_<ExpressionCost, Cost, std::shared_ptr<ExpressionCost>>(
      m, "ExpressionCost", doc.ExpressionCost.doc)
      .def(py::init<const symbolic::Expression&>(), py::arg("e"),
          doc.ExpressionCost.ctor.doc)
      .def("expression", &ExpressionCost::expression,
          py_rvp::reference_internal, doc.ExpressionCost.expression.doc)
      .def("vars", &ExpressionCost::vars,
          // dtype = object arrays must be copied, and cannot be referenced.
          py_rvp::copy, doc.ExpressionCost.vars.doc);

  auto cost_binding = RegisterBinding<Cost>(&m);
  DefBindingCastConstructor<Cost>(&cost_binding);
  RegisterBinding<LinearCost>(&m);
  RegisterBinding<QuadraticCost>(&m);
  RegisterBinding<L1NormCost>(&m);
  RegisterBinding<L2NormCost>(&m);
  RegisterBinding<LInfNormCost>(&m);
  RegisterBinding<PerspectiveQuadraticCost>(&m);
  // TODO(russt): PolynomialCost currently uses common::Polynomial, not
  // symbolic::Polynomial. Decide whether we want to bind the current c++
  // implementation as is, or convert it to symbolic::Polynomial first.
  RegisterBinding<ExpressionCost>(&m);

  py::class_<VisualizationCallback, EvaluatorBase,
      std::shared_ptr<VisualizationCallback>>(
      m, "VisualizationCallback", doc.VisualizationCallback.doc);

  RegisterBinding<VisualizationCallback>(&m);
}  // NOLINT(readability/fn_size)

namespace internal {
void DefineSolversEvaluators(py::module m) {
  BindEvaluatorsAndBindings(m);
  DefTesting(m.def_submodule("_testing"));
}
}  // namespace internal
}  // namespace pydrake
}  // namespace drake
