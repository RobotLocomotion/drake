#include "drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/pointer_cast.h"
#include "drake/common/symbolic/decompose.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/matrix_util.h"

using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::VectorXd;
using std::nullopt;
using std::optional;

namespace drake {
namespace planning {
namespace trajectory_optimization {

using math::BsplineBasis;
using math::EigenToStdVector;
using math::ExtractValue;
using math::InitializeAutoDiff;
using math::StdVectorToEigen;
using multibody::MultibodyForces;
using multibody::MultibodyPlant;
using solvers::Binding;
using solvers::BoundingBoxConstraint;
using solvers::Constraint;
using solvers::Cost;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::MatrixXDecisionVariable;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Variables;
using trajectories::BsplineTrajectory;

namespace {

const double kInf = std::numeric_limits<double>::infinity();

class PathConstraint : public Constraint {
 public:
  PathConstraint(std::shared_ptr<Constraint> wrapped_constraint,
                 std::vector<double> basis_function_values)
      : Constraint(
            wrapped_constraint->num_outputs(),
            basis_function_values.size() * wrapped_constraint->num_vars(),
            wrapped_constraint->lower_bound(),
            wrapped_constraint->upper_bound()),
        wrapped_constraint_(wrapped_constraint),
        basis_function_values_(std::move(basis_function_values)) {}

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    AutoDiffVecXd y_t;
    Eval(InitializeAutoDiff(x), &y_t);
    *y = ExtractValue(y_t);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    AutoDiffVecXd x_sum = basis_function_values_[0] *
                          x.segment(0, wrapped_constraint_->num_vars());
    const int num_terms = basis_function_values_.size();
    for (int i = 1; i < num_terms; ++i) {
      x_sum += basis_function_values_[i] *
               x.segment(i * wrapped_constraint_->num_vars(),
                         wrapped_constraint_->num_vars());
    }
    wrapped_constraint_->Eval(x_sum, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::runtime_error(
        "PathConstraint does not support evaluation with Expression.");
  }

 private:
  std::shared_ptr<Constraint> wrapped_constraint_;
  std::vector<double> basis_function_values_;
};

/* Implements a constraint of the form
  wrapped_constraint([q, v]), where
  duration = x[0]
  control_points = x[1:num_control_points].reshaped(
      num_positions, num_control_points)
  q = control_points * a_pos v = control_points * a_vel
*/
class WrappedVelocityConstraint : public Constraint {
 public:
  WrappedVelocityConstraint(std::shared_ptr<Constraint> wrapped_constraint,
                            VectorXd a_pos, VectorXd a_vel, int num_positions,
                            int num_control_points)
      : Constraint(wrapped_constraint->num_outputs(),
                   1 + num_positions * num_control_points,
                   wrapped_constraint->lower_bound(),
                   wrapped_constraint->upper_bound()),
        wrapped_constraint_(wrapped_constraint),
        a_pos_{std::move(a_pos)},
        a_vel_{std::move(a_vel)},
        num_positions_{num_positions},
        num_control_points_{num_control_points} {
    DRAKE_DEMAND(2 * num_positions_ == wrapped_constraint_->num_vars());
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    AutoDiffVecXd y_t;
    Eval(InitializeAutoDiff(x), &y_t);
    *y = ExtractValue(y_t);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    AutoDiffXd duration = x[0];
    MatrixX<AutoDiffXd> control_points =
        x.tail(num_positions_ * num_control_points_)
            .reshaped(num_positions_, num_control_points_);
    VectorX<AutoDiffXd> qv(wrapped_constraint_->num_vars());
    qv.head(num_positions_) = control_points * a_pos_.cast<AutoDiffXd>();
    qv.tail(num_positions_) = control_points * (a_vel_ / duration);
    wrapped_constraint_->Eval(qv, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::runtime_error(
        "WrappedDerivativeConstraint does not support evaluation with "
        "Expression.");
  }

 private:
  std::shared_ptr<Constraint> wrapped_constraint_;
  const VectorXd a_pos_;
  const VectorXd a_vel_;
  const int num_positions_;
  const int num_control_points_;
};

/* Implements a constraint of the form:
    duration = x[0]
    control_points.T = x[1:num_control_points]
    lb <= A * control_points.T / duration^order <= ub
*/
class DerivativeConstraint : public Constraint {
 public:
  DerivativeConstraint(const SparseMatrix<double>& A, int derivative_order,
                       double lb, double ub)
      : Constraint(A.rows(), A.cols() + 1, VectorXd::Constant(A.rows(), lb),
                   VectorXd::Constant(A.rows(), ub)),
        A_{A},
        derivative_order_{derivative_order} {
    DRAKE_DEMAND(derivative_order >= 1);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    AutoDiffVecXd y_t;
    Eval(InitializeAutoDiff(x), &y_t);
    *y = ExtractValue(y_t);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    AutoDiffXd duration = x[0];
    *y = A_ * x.tail(A_.cols()) / pow(duration, derivative_order_);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::runtime_error(
        "DerivativeConstraint does not support evaluation with Expression.");
  }

 private:
  const SparseMatrix<double> A_;
  const int derivative_order_;
};

// Constraint of the form tau_lb <= InverseDynamics(q, v, a) <= tau_ub. The
// constraint should be bound to the duration followed by all of the control
// points in the bspline. TODO(russt): The constraint actually only depends on
// a subset of the control points, depending on s, so we could consider
// optimizing this aspect.
class EffortConstraint : public Constraint {
 public:
  EffortConstraint(const std::shared_ptr<MultibodyPlant<AutoDiffXd>>& plant,
                   std::shared_ptr<systems::Context<AutoDiffXd>> plant_context,
                   const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                   const BsplineTrajectory<double>& bspline, double s)
      : Constraint(lb.size(), 1 + bspline.num_control_points() * bspline.rows(),
                   lb, ub),
        plant_(plant),
        plant_context_(std::move(plant_context)),
        num_control_points_(bspline.num_control_points()),
        s_(s) {
    // Note: consistency checks on the input arguments already happened in
    // AddEffortBoundsAtNormalizedTimes().
    M_q_ = bspline.EvaluateLinearInControlPoints(s, 0).cast<AutoDiffXd>();
    M_v_ = bspline.EvaluateLinearInControlPoints(s, 1).cast<AutoDiffXd>();
    M_vdot_ = bspline.EvaluateLinearInControlPoints(s, 2).cast<AutoDiffXd>();
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    AutoDiffVecXd y_t;
    Eval(InitializeAutoDiff(x), &y_t);
    *y = ExtractValue(y_t);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    y->resize(plant_->num_positions());
    AutoDiffXd duration = x[0];
    MatrixX<AutoDiffXd> control_points =
        x.tail(plant_->num_positions() * num_control_points_)
            .reshaped(plant_->num_positions(), num_control_points_);
    plant_context_->SetTime(s_ * duration);
    plant_->SetPositions(plant_context_.get(), control_points * M_q_);
    plant_->SetVelocities(plant_context_.get(),
                          control_points * M_v_ / duration);
    VectorX<AutoDiffXd> vdot = control_points * M_vdot_ / pow(duration, 2);
    drake::log()->info("q: {}", fmt_eigen(control_points * M_q_));
    drake::log()->info("v: {}", fmt_eigen(control_points * M_v_ / duration));
    drake::log()->info("vdot: {}", fmt_eigen(vdot.transpose()));
    MultibodyForces<AutoDiffXd> forces_(*plant_);
    plant_->CalcForceElementsContribution(*plant_context_, &forces_);
    *y = plant_->CalcInverseDynamics(*plant_context_, vdot, forces_);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::runtime_error(
        "EffortConstraint does not support evaluation with Expression.");
  }

 private:
  const std::shared_ptr<MultibodyPlant<AutoDiffXd>> plant_;
  std::shared_ptr<systems::Context<AutoDiffXd>> plant_context_;
  // M_q_, M_v_, M_vdot_ are matrices such that by multiplying control_points
  // with these matrices, we get the position q, dqds, and d²qds² respectively.
  VectorX<AutoDiffXd> M_q_;
  VectorX<AutoDiffXd> M_v_;
  VectorX<AutoDiffXd> M_vdot_;
  int num_control_points_;
  double s_;
};

// solvers::LinearConstraint stores linear constraints in the form A*x <= b.
// When constraints are naturally expressed in the form X*a <= b (the
// decision variables on the left), it is still a vector constraint, but we
// must do some work to rewrite it as A*x, and the result is a big sparse A.
// This method does precisely that. For MatrixXDecisionVariable X and
// VectorXd a, returns SparseMatrix A and VectorXDecisionVariable x such that
// Ax == Xa.
std::pair<SparseMatrix<double>, VectorXDecisionVariable> RewriteXa(
    const MatrixXDecisionVariable& X, const VectorXd& a) {
  const int num_nonzeros = (a.array() != 0).count();
  std::vector<Eigen::Triplet<double>> triplets(X.rows() * num_nonzeros);
  VectorXDecisionVariable x(X.rows() * num_nonzeros);
  // A * x = [aᵀ, 0, 0, ..., 0 ][Xᵀ(:,0)]
  //         [0, aᵀ, 0, ..., 0 ][Xᵀ(:,1)]
  //         [      ...        ][  ...  ]
  //         [0, 0, 0, ...., aᵀ][Xᵀ(:,m)]
  int n = 0;
  for (int row = 0; row < X.rows(); ++row) {
    for (int col = 0; col < X.cols(); ++col) {
      if (a(col) != 0) {
        triplets.push_back(Eigen::Triplet<double>(row, n, a(col)));
        x(n++) = X(row, col);
      }
    }
  }
  SparseMatrix<double> A(X.rows(), n);
  A.setFromTriplets(triplets.begin(), triplets.end());
  return std::make_pair(A, x);
}

}  // namespace

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    int num_positions, int num_control_points, int spline_order,
    double duration)
    : KinematicTrajectoryOptimization(BsplineTrajectory(
          BsplineBasis<double>(spline_order, num_control_points,
                               math::KnotVectorType::kClampedUniform, 0,
                               duration),
          std::vector<MatrixXd>(num_control_points,
                                MatrixXd::Zero(num_positions, 1)))) {}

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    const trajectories::BsplineTrajectory<double>& trajectory)
    : num_positions_(trajectory.rows()),
      num_control_points_(trajectory.num_control_points()) {
  // basis = trajectory.basis() normalized to s∈[0,1].
  const double duration = trajectory.end_time() - trajectory.start_time();
  std::vector<double> normalized_knots = trajectory.basis().knots();
  for (auto& knot : normalized_knots) {
    knot /= duration;
  }
  BsplineBasis<double> basis(trajectory.basis().order(), normalized_knots);
  bspline_ = BsplineTrajectory<double>(basis, trajectory.control_points());

  control_points_ = prog_.NewContinuousVariables(
      num_positions_, num_control_points_, "control_point");
  duration_ = prog_.NewContinuousVariables(1, "duration")[0];
  // duration >= 0.
  auto duration_bbox_binding = prog_.AddBoundingBoxConstraint(
      0, std::numeric_limits<double>::infinity(), duration_);
  duration_bbox_binding.evaluator()->set_description(
      "positive duration constraint");

  SetInitialGuess(trajectory);
}

void KinematicTrajectoryOptimization::SetInitialGuess(
    const trajectories::BsplineTrajectory<double>& trajectory) {
  prog_.SetInitialGuess(control_points_,
                        StdVectorToEigen<double>(trajectory.control_points()));
  prog_.SetInitialGuess(duration_,
                        trajectory.end_time() - trajectory.start_time());
}

BsplineTrajectory<double>
KinematicTrajectoryOptimization::ReconstructTrajectory(
    const MathematicalProgramResult& result) const {
  const double duration = result.GetSolution(duration_);
  std::vector<double> scaled_knots = basis().knots();
  for (auto& knot : scaled_knots) {
    knot *= duration;
  }

  return BsplineTrajectory<double>(
      BsplineBasis<double>(basis().order(), scaled_knots),
      EigenToStdVector<double>(result.GetSolution(control_points_)));
}

Binding<LinearConstraint>
KinematicTrajectoryOptimization::AddPathPositionConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorXd M = bspline_.EvaluateLinearInControlPoints(s);
  // lb <= control_points_ * M <= ub
  auto [A, x] = RewriteXa(control_points_, M);
  auto binding = prog_.AddLinearConstraint(A, lb, ub, x);
  binding.evaluator()->set_description(
      fmt::format("path position constraint at s={}", s));
  return binding;
}

Binding<Constraint> KinematicTrajectoryOptimization::AddPathPositionConstraint(
    const std::shared_ptr<Constraint>& constraint, double s) {
  DRAKE_DEMAND(constraint->num_vars() == num_positions_);
  DRAKE_DEMAND(0 <= s && s <= 1);
  std::vector<double> basis_function_values;
  basis_function_values.reserve(basis().order());
  std::vector<int> active_control_point_indices =
      basis().ComputeActiveBasisFunctionIndices(s);
  const int num_active_control_points =
      static_cast<int>(active_control_point_indices.size());
  VectorXDecisionVariable var_vector(num_active_control_points *
                                     num_positions());
  for (int i = 0; i < num_active_control_points; ++i) {
    const int control_point_index = active_control_point_indices[i];
    basis_function_values.push_back(
        basis().EvaluateBasisFunctionI(control_point_index, s));
    var_vector.segment(i * num_positions(), num_positions()) =
        control_points_.col(control_point_index);
  }
  auto binding = prog_.AddConstraint(
      std::make_shared<PathConstraint>(constraint, basis_function_values),
      var_vector);
  binding.evaluator()->set_description(
      fmt::format("path position constraint at s={}", s));
  return binding;
}

Binding<LinearConstraint>
KinematicTrajectoryOptimization::AddPathVelocityConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorXd M =
      bspline_.EvaluateLinearInControlPoints(s, /* derivative_order= */ 1);
  // lb <= control_points_ * M <= ub
  auto [A, x] = RewriteXa(control_points_, M);
  auto binding = prog_.AddLinearConstraint(A, lb, ub, x);
  binding.evaluator()->set_description(
      fmt::format("path velocity constraint at s={}", s));
  return binding;
}

Binding<Constraint>
KinematicTrajectoryOptimization::AddVelocityConstraintAtNormalizedTime(
    const std::shared_ptr<solvers::Constraint>& constraint, double s) {
  DRAKE_DEMAND(constraint->num_vars() == 2 * num_positions_);
  DRAKE_DEMAND(0 <= s && s <= 1);

  const VectorXd a_pos =
      bspline_.EvaluateLinearInControlPoints(s, /* derivative_order= */ 0);
  const VectorXd a_vel =
      bspline_.EvaluateLinearInControlPoints(s, /* derivative_order= */ 1);
  VectorXDecisionVariable vars(1 + control_points_.size());
  vars << duration_, control_points_.reshaped(control_points_.size(), 1);
  auto wrapped_constraint = std::make_shared<WrappedVelocityConstraint>(
      constraint, std::move(a_pos), std::move(a_vel), num_positions_,
      num_control_points_);
  auto binding = prog_.AddConstraint(wrapped_constraint, vars);
  binding.evaluator()->set_description(
      fmt::format("velocity constraint at s={}", s));
  return binding;
}

Binding<LinearConstraint>
KinematicTrajectoryOptimization::AddPathAccelerationConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorXd M =
      bspline_.EvaluateLinearInControlPoints(s, /* derivative_order= */ 2);
  // lb <= control_points_ * M <= ub
  auto [A, x] = RewriteXa(control_points_, M);
  auto binding = prog_.AddLinearConstraint(A, lb, ub, x);
  binding.evaluator()->set_description(
      fmt::format("path acceleration constraint at s={}", s));
  return binding;
}

Binding<BoundingBoxConstraint>
KinematicTrajectoryOptimization::AddDurationConstraint(optional<double> lb,
                                                       optional<double> ub) {
  auto binding = prog_.AddBoundingBoxConstraint(
      lb.value_or(0), ub.value_or(std::numeric_limits<double>::infinity()),
      duration_);
  binding.evaluator()->set_description("duration constraint");
  return binding;
}

std::vector<Binding<BoundingBoxConstraint>>
KinematicTrajectoryOptimization::AddPositionBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  // This leverages the convex hull property of the B-splines: if all of the
  // control points satisfy these convex constraints and the curve is inside
  // the convex hull of these constraints, then the curve satisfies the
  // constraints for all s (and therefore all t).
  std::vector<Binding<BoundingBoxConstraint>> binding;
  for (int i = 0; i < num_control_points(); ++i) {
    binding.emplace_back(
        prog_.AddBoundingBoxConstraint(lb, ub, control_points_.col(i)));
    binding[i].evaluator()->set_description(
        fmt::format("position bound {}", i));
  }
  return binding;
}

std::vector<Binding<LinearConstraint>>
KinematicTrajectoryOptimization::AddVelocityBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());

  // We have q̇(t) = drds * dsdt = ṙ(s) / duration, and duration >= 0, so we
  // use duration * lb <= ṙ(s) <= duration * ub.
  //
  // This also leverages the convex hull property of the B-splines: if all of
  // the control points satisfy these convex constraints and the curve is
  // inside the convex hull of these constraints, then the curve satisfies the
  // constraints for all t.
  std::vector<Binding<LinearConstraint>> bindings;
  const SparseMatrix<double> M =
      bspline_.AsLinearInControlPoints(/* derivative_order = */ 1);
  // duration * lb <= control_points * M <= duration * ub.
  VectorXDecisionVariable vars(1 + num_control_points_);
  vars[0] = duration_;
  SparseMatrix<double> A(M.cols(), M.rows() + 1);
  A.rightCols(M.rows()) = M.transpose();
  for (int i = 0; i < num_positions_; ++i) {
    vars.tail(num_control_points_) = control_points_.row(i).transpose();
    // lb(i)*duration <= control_points(i,:) * M, but transposed
    A.col(0) = VectorXd::Constant(M.cols(), -lb(i)).sparseView();
    bindings.emplace_back(prog_.AddLinearConstraint(
        A, /* lb= */ VectorXd::Zero(M.cols()),
        /* ub= */ VectorXd::Constant(M.cols(), kInf), vars));
    bindings.back().evaluator()->set_description(
        fmt::format("velocity lower bound for position {}", i));
    // ub(i)*duration >= control_points(i,:) * M, but transposed
    A.col(0) = VectorXd::Constant(M.cols(), -ub(i)).sparseView();
    bindings.emplace_back(prog_.AddLinearConstraint(
        A, /* lb= */ VectorXd::Constant(M.cols(), -kInf),
        /* ub= */ VectorXd::Zero(M.cols()), vars));
    bindings.back().evaluator()->set_description(
        fmt::format("velocity upper bound for position {}", i));
  }
  return bindings;
}

std::vector<Binding<Constraint>>
KinematicTrajectoryOptimization::AddAccelerationBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());

  // These are nonconvex constraints, but they again leverage the convex hull
  // property to enforce the guarantee ∀t by only constraining the values at
  // the control points.
  const SparseMatrix<double> M =
      bspline_.AsLinearInControlPoints(/* derivative_order= */ 2);
  SparseMatrix<double> A = M.transpose();
  VectorXDecisionVariable vars(1 + num_control_points_);
  vars[0] = duration_;
  std::vector<Binding<Constraint>> bindings;
  for (int i = 0; i < num_positions_; ++i) {
    // implements lb(i) <= control_points(i,:) * M / duration^2 <= ub(i)
    vars.tail(num_control_points_) = control_points_.row(i).transpose();
    bindings.emplace_back(prog_.AddConstraint(
        std::make_shared<DerivativeConstraint>(A, 2, lb(i), ub(i)), vars));
    bindings.back().evaluator()->set_description(
        fmt::format("acceleration bound for position {}", i));
  }
  return bindings;
}

std::vector<Binding<Constraint>> KinematicTrajectoryOptimization::AddJerkBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());

  // This again leverages the convex hull property to enforce the guarantee ∀t
  // by only constraining the values at the control points.
  const SparseMatrix<double> M =
      bspline_.AsLinearInControlPoints(/* derivative_order= */ 3);
  SparseMatrix<double> A = M.transpose();
  VectorXDecisionVariable vars(1 + num_control_points_);
  vars[0] = duration_;
  std::vector<Binding<Constraint>> bindings;
  for (int i = 0; i < num_positions_; ++i) {
    // implements lb(i) <= control_points(i,:) * M / duration^3 <= ub(i)
    vars.tail(num_control_points_) = control_points_.row(i).transpose();
    bindings.emplace_back(prog_.AddConstraint(
        std::make_shared<DerivativeConstraint>(A, 3, lb(i), ub(i)), vars));
    bindings.back().evaluator()->set_description(
        fmt::format("jerk bound for position {}", i));
  }
  return bindings;
}

std::vector<Binding<Constraint>>
KinematicTrajectoryOptimization::AddEffortBoundsAtNormalizedTimes(
    const MultibodyPlant<double>& plant,
    const Eigen::Ref<const Eigen::VectorXd>& s,
    const std::optional<Eigen::Ref<const Eigen::VectorXd>>& lb,
    const std::optional<Eigen::Ref<const Eigen::VectorXd>>& ub,
    const systems::Context<double>* plant_context) {
  DRAKE_THROW_UNLESS(s.array().minCoeff() >= 0 && s.array().maxCoeff() <= 1);
  const MatrixXd B = plant.MakeActuationMatrix();
  const VectorXd B_tau_lb = B * (lb.value_or(plant.GetEffortLowerLimits()));
  const VectorXd B_tau_ub = B * (ub.value_or(plant.GetEffortUpperLimits()));
  DRAKE_THROW_UNLESS(B_tau_lb.size() == num_positions());
  DRAKE_THROW_UNLESS(B_tau_ub.size() == num_positions());
  DRAKE_THROW_UNLESS((B_tau_lb.array() <= B_tau_ub.array()).all());
  if (plant_context != nullptr) {
    plant.ValidateContext(plant_context);
  }

  VectorXDecisionVariable vars(1 + num_control_points_ * num_positions_);
  vars << duration_, control_points_.reshaped(control_points_.size(), 1);
  std::shared_ptr<MultibodyPlant<AutoDiffXd>> plant_ad(
      systems::System<double>::ToAutoDiffXd(plant));
  std::vector<Binding<Constraint>> bindings;
  for (int i = 0; i < ssize(s); ++i) {
    // Note: we choose to make a separate context for each s so that the
    // constraints are thread-safe.
    std::shared_ptr<systems::Context<AutoDiffXd>> plant_context_ad(
        plant_ad->CreateDefaultContext());
    if (plant_context != nullptr) {
      plant_context_ad->SetTimeStateAndParametersFrom(*plant_context);
    }
    bindings.emplace_back(prog_.AddConstraint(
        std::make_shared<EffortConstraint>(plant_ad,
                                           std::move(plant_context_ad),
                                           B_tau_lb, B_tau_ub, bspline_, s[i]),
        vars));
    bindings.back().evaluator()->set_description(
        fmt::format("effort bound at s={}", s[i]));
  }
  return bindings;
}

Binding<LinearCost> KinematicTrajectoryOptimization::AddDurationCost(
    double weight) {
  auto binding = prog_.AddLinearCost(weight * duration_);
  binding.evaluator()->set_description("duration cost");
  return binding;
}

std::vector<Binding<Cost>> KinematicTrajectoryOptimization::AddPathLengthCost(
    double weight, bool use_conic_constraint) {
  MatrixXd A(num_positions_, 2 * num_positions_);
  A.leftCols(num_positions_) =
      weight * MatrixXd::Identity(num_positions_, num_positions_);
  A.rightCols(num_positions_) =
      -weight * MatrixXd::Identity(num_positions_, num_positions_);
  const VectorXd b = VectorXd::Zero(num_positions_);
  VectorXDecisionVariable vars(2 * num_positions_);
  std::vector<Binding<Cost>> binding;
  for (int i = 1; i < num_control_points(); ++i) {
    vars.head(num_positions_) = control_points_.col(i);
    vars.tail(num_positions_) = control_points_.col(i - 1);
    if (use_conic_constraint) {
      auto slack_cost_and_constraint_tuple =
          prog_.AddL2NormCostUsingConicConstraint(A, b, vars);
      binding.emplace_back(std::get<1>(slack_cost_and_constraint_tuple));
      std::get<2>(slack_cost_and_constraint_tuple)
          .evaluator()
          ->set_description(
              fmt::format("path length cost {} conic constraint", i));
    } else {
      binding.emplace_back(prog_.AddL2NormCost(A, b, vars));
    }
    binding[i - 1].evaluator()->set_description(
        fmt::format("path length cost {}", i));
  }
  return binding;
}

std::vector<solvers::Binding<solvers::Cost>>
KinematicTrajectoryOptimization::AddPathEnergyCost(double weight) {
  // For successive control points x, y, impose the cost (x-y)ᵀ(x-y) as
  // [x;y]ᵀ [I, -I; -I, I] [x;y]. This matrix is positive semidefinite, so the
  // resulting quadratic constraint is convex. We actually double the values in
  // the matrix, since AddQuadraticCost multiplies by a factor of 0.5.
  MatrixXd A(2 * num_positions_, 2 * num_positions_);
  A.topLeftCorner(num_positions_, num_positions_) =
      2.0 * weight * MatrixXd::Identity(num_positions_, num_positions_);
  A.bottomRightCorner(num_positions_, num_positions_) =
      2.0 * weight * MatrixXd::Identity(num_positions_, num_positions_);
  A.topRightCorner(num_positions_, num_positions_) =
      -2.0 * weight * MatrixXd::Identity(num_positions_, num_positions_);
  A.bottomLeftCorner(num_positions_, num_positions_) =
      -2.0 * weight * MatrixXd::Identity(num_positions_, num_positions_);
  const VectorXd b = VectorXd::Zero(2 * num_positions_);
  VectorXDecisionVariable vars(2 * num_positions_);
  std::vector<Binding<Cost>> binding;
  for (int i = 1; i < num_control_points(); ++i) {
    vars.head(num_positions_) = control_points_.col(i);
    vars.tail(num_positions_) = control_points_.col(i - 1);
    binding.emplace_back(prog_.AddQuadraticCost(A, b, vars, true));
    binding[i - 1].evaluator()->set_description(
        fmt::format("path energy cost {}", i));
  }
  return binding;
}

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
