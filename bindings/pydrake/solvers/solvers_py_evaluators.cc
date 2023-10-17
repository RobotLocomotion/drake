#include <memory>

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/wrap_function.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/evaluator_base.h"
#include "drake/solvers/minimum_value_constraint.h"

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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
using solvers::MinimumValueConstraint;
#pragma GCC diagnostic pop
using solvers::MinimumValueLowerBoundConstraint;
using solvers::MinimumValueUpperBoundConstraint;
using solvers::PerspectiveQuadraticCost;
using solvers::PositiveSemidefiniteConstraint;
using solvers::QuadraticConstraint;
using solvers::QuadraticCost;
using solvers::RotatedLorentzConeConstraint;
using solvers::VectorXDecisionVariable;
using solvers::VisualizationCallback;
namespace {

/*
 * Register a Binding template.
 * @tparam C An EvaluatorBase (or its child) class.
 * @param scope The scope this will be added to (e.g., the module).
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
      .def(
          "ToLatex", &B::ToLatex, py::arg("precision") = 3, cls_doc.ToLatex.doc)
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
        .def("ToLatex", &Class::ToLatex, py::arg("vars"),
            py::arg("precision") = 3, cls_doc.ToLatex.doc)
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
      .def("is_dense_A_constructed", &LinearConstraint::is_dense_A_constructed,
          doc.LinearConstraint.is_dense_A_constructed.doc)
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::string dep_message =
      "MinimumValueConstraint is deprecated. Use "
      "MinimumValueLowerBoundConstraint if you want the smallest value to be "
      "lower bounded, or MinimumValueUpperBoundConstraint if you want the "
      "smallest value to be upper bounded. The deprecated code will be removed "
      "from Drake on or after 2024-02-01.\n\n";
  py::class_<MinimumValueConstraint, Constraint,
      std::shared_ptr<MinimumValueConstraint>>(
      m, "MinimumValueConstraint", doc.MinimumValueConstraint.doc_deprecated)
      .def(py_init_deprecated(
               dep_message + doc.MinimumValueConstraint.ctor.doc_6args,
               [](int num_vars, double minimum_value,
                   double influence_value_offset, int max_num_values,
                   // If I pass in const Eigen::Ref<const AutoDiffVecXd>& here
                   // then I got the RuntimeError: dtype=object arrays must be
                   // copied, and cannot be referenced.
                   std::function<AutoDiffVecXd(const AutoDiffVecXd&, double)>
                       value_function,
                   std::function<Eigen::VectorXd(
                       const Eigen::Ref<const Eigen::VectorXd>&, double)>
                       value_function_double) {
                 return std::make_unique<MinimumValueConstraint>(num_vars,
                     minimum_value, influence_value_offset, max_num_values,
                     value_function, value_function_double);
               }),
          py::arg("num_vars"), py::arg("minimum_value"),
          py::arg("influence_value_offset"), py::arg("max_num_values"),
          py::arg("value_function"),
          py::arg("value_function_double") = std::function<Eigen::VectorXd(
              const Eigen::Ref<const Eigen::VectorXd>&, double)>{},
          doc.MinimumValueConstraint.ctor.doc_6args)
      .def(py_init_deprecated(
               dep_message + doc.MinimumValueConstraint.ctor.doc_7args,
               [](int num_vars, double minimum_value_lower,
                   double minimum_value_upper, double influence_value,
                   int max_num_values,
                   // If I pass in const Eigen::Ref<const AutoDiffVecXd>& here
                   // then I got the RuntimeError: dtype=object arrays must be
                   // copied, and cannot be referenced.
                   std::function<AutoDiffVecXd(const AutoDiffVecXd&, double)>
                       value_function,
                   std::function<Eigen::VectorXd(
                       const Eigen::Ref<const Eigen::VectorXd>&, double)>
                       value_function_double) {
                 return std::make_unique<MinimumValueConstraint>(num_vars,
                     minimum_value_lower, minimum_value_upper, influence_value,
                     max_num_values, value_function, value_function_double);
               }),
          py::arg("num_vars"), py::arg("minimum_value_lower"),
          py::arg("minimum_value_upper"), py::arg("influence_value"),
          py::arg("max_num_values"), py::arg("value_function"),
          py::arg("value_function_double") = std::function<Eigen::VectorXd(
              const Eigen::Ref<const Eigen::VectorXd>&, double)>{},
          doc.MinimumValueConstraint.ctor.doc_7args)
      .def("minimum_value_lower", &MinimumValueConstraint::minimum_value_lower,
          doc.MinimumValueConstraint.minimum_value_lower.doc)
      .def("minimum_value_upper", &MinimumValueConstraint::minimum_value_upper,
          doc.MinimumValueConstraint.minimum_value_upper.doc)
      .def("influence_value", &MinimumValueConstraint::influence_value,
          doc.MinimumValueConstraint.influence_value.doc)
      .def(
          "set_penalty_function",
          [](MinimumValueConstraint* self,
              std::function<py::tuple(double, bool)> new_penalty_function) {
            auto penalty_fun = [new_penalty_function](double x, double* penalty,
                                   double* dpenalty) {
              py::tuple penalty_tuple(2);
              penalty_tuple = new_penalty_function(x, dpenalty != nullptr);
              *penalty = penalty_tuple[0].cast<double>();
              if (dpenalty) {
                *dpenalty = penalty_tuple[1].cast<double>();
              }
            };
            self->set_penalty_function(penalty_fun);
          },
          py::arg("new_penalty_function"),
          "Setter for the penalty function. The penalty function "
          "new_penalty_function(x: float, compute_grad: bool) -> tuple[float, "
          "Optional[float]] "
          "returns [penalty_value, penalty_gradient] when "
          "compute_grad=True, or [penalty_value, None] when "
          "compute_grad=False. See minimum_value_constraint.h on the "
          "requirement on MinimumValuePenaltyFunction.");
#pragma GCC diagnostic pop

  py::class_<MinimumValueLowerBoundConstraint, Constraint,
      std::shared_ptr<MinimumValueLowerBoundConstraint>>(m,
      "MinimumValueLowerBoundConstraint",
      doc.MinimumValueLowerBoundConstraint.doc)
      .def(py::init(
               [](int num_vars, double minimum_value_lower,
                   double influence_value_offset, int max_num_values,
                   // If I pass in const Eigen::Ref<const AutoDiffVecXd>& here
                   // then I got the RuntimeError: dtype=object arrays must be
                   // copied, and cannot be referenced.
                   std::function<AutoDiffVecXd(const AutoDiffVecXd&, double)>
                       value_function,
                   std::function<Eigen::VectorXd(
                       const Eigen::Ref<const Eigen::VectorXd>&, double)>
                       value_function_double) {
                 return std::make_unique<MinimumValueLowerBoundConstraint>(
                     num_vars, minimum_value_lower, influence_value_offset,
                     max_num_values, value_function, value_function_double);
               }),
          py::arg("num_vars"), py::arg("minimum_value_lower"),
          py::arg("influence_value_offset"), py::arg("max_num_values"),
          py::arg("value_function"),
          py::arg("value_function_double") = std::function<Eigen::VectorXd(
              const Eigen::Ref<const Eigen::VectorXd>&, double)>{},
          doc.MinimumValueLowerBoundConstraint.ctor.doc)
      .def("minimum_value_lower",
          &MinimumValueLowerBoundConstraint::minimum_value_lower,
          doc.MinimumValueLowerBoundConstraint.minimum_value_lower.doc)
      .def("influence_value",
          &MinimumValueLowerBoundConstraint::influence_value,
          doc.MinimumValueLowerBoundConstraint.influence_value.doc)
      .def(
          "set_penalty_function",
          [](MinimumValueLowerBoundConstraint* self,
              std::function<py::tuple(double, bool)> new_penalty_function) {
            auto penalty_fun = [new_penalty_function](double x, double* penalty,
                                   double* dpenalty) {
              py::tuple penalty_tuple(2);
              penalty_tuple = new_penalty_function(x, dpenalty != nullptr);
              *penalty = penalty_tuple[0].cast<double>();
              if (dpenalty) {
                *dpenalty = penalty_tuple[1].cast<double>();
              }
            };
            self->set_penalty_function(penalty_fun);
          },
          py::arg("new_penalty_function"),
          "Setter for the penalty function. The penalty function "
          "new_penalty_function(x: float, compute_grad: bool) -> tuple[float, "
          "Optional[float]] "
          "returns [penalty_value, penalty_gradient] when "
          "compute_grad=True, or [penalty_value, None] when "
          "compute_grad=False. See minimum_value_constraint.h on the "
          "requirement on MinimumValuePenaltyFunction.");

  py::class_<MinimumValueUpperBoundConstraint, Constraint,
      std::shared_ptr<MinimumValueUpperBoundConstraint>>(m,
      "MinimumValueUpperBoundConstraint",
      doc.MinimumValueUpperBoundConstraint.doc)
      .def(py::init(
               [](int num_vars, double minimum_value_upper,
                   double influence_value_offset, int max_num_values,
                   // If I pass in const Eigen::Ref<const AutoDiffVecXd>& here
                   // then I got the RuntimeError: dtype=object arrays must be
                   // copied, and cannot be referenced.
                   std::function<AutoDiffVecXd(const AutoDiffVecXd&, double)>
                       value_function,
                   std::function<Eigen::VectorXd(
                       const Eigen::Ref<const Eigen::VectorXd>&, double)>
                       value_function_double) {
                 return std::make_unique<MinimumValueUpperBoundConstraint>(
                     num_vars, minimum_value_upper, influence_value_offset,
                     max_num_values, value_function, value_function_double);
               }),
          py::arg("num_vars"), py::arg("minimum_value_upper"),
          py::arg("influence_value_offset"), py::arg("max_num_values"),
          py::arg("value_function"),
          py::arg("value_function_double") = std::function<Eigen::VectorXd(
              const Eigen::Ref<const Eigen::VectorXd>&, double)>{},
          doc.MinimumValueUpperBoundConstraint.ctor.doc)
      .def("minimum_value_upper",
          &MinimumValueUpperBoundConstraint::minimum_value_upper,
          doc.MinimumValueUpperBoundConstraint.minimum_value_upper.doc)
      .def("influence_value",
          &MinimumValueUpperBoundConstraint::influence_value,
          doc.MinimumValueUpperBoundConstraint.influence_value.doc)
      .def(
          "set_penalty_function",
          [](MinimumValueUpperBoundConstraint* self,
              std::function<py::tuple(double, bool)> new_penalty_function) {
            auto penalty_fun = [new_penalty_function](double x, double* penalty,
                                   double* dpenalty) {
              py::tuple penalty_tuple(2);
              penalty_tuple = new_penalty_function(x, dpenalty != nullptr);
              *penalty = penalty_tuple[0].cast<double>();
              if (dpenalty) {
                *dpenalty = penalty_tuple[1].cast<double>();
              }
            };
            self->set_penalty_function(penalty_fun);
          },
          py::arg("new_penalty_function"),
          "Setter for the penalty function. The penalty function "
          "new_penalty_function(x: float, compute_grad: bool) -> tuple[float, "
          "Optional[float]] "
          "returns [penalty_value, penalty_gradient] when "
          "compute_grad=True, or [penalty_value, None] when "
          "compute_grad=False. See minimum_value_constraint.h on the "
          "requirement on MinimumValuePenaltyFunction.");

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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  RegisterBinding<MinimumValueConstraint>(&m);
#pragma GCC diagnostic pop
  RegisterBinding<MinimumValueLowerBoundConstraint>(&m);
  RegisterBinding<MinimumValueUpperBoundConstraint>(&m);
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
