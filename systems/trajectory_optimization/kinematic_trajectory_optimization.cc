#include "drake/systems/trajectory_optimization/kinematic_trajectory_optimization.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "drake/common/pointer_cast.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/matrix_util.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::nullopt;
using std::optional;

namespace drake {
namespace systems {
namespace trajectory_optimization {

using math::BsplineBasis;
using math::EigenToStdVector;
using math::ExtractValue;
using math::InitializeAutoDiff;
using math::StdVectorToEigen;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using trajectories::BsplineTrajectory;

namespace {

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
  // basis_ = trajectory.basis() normalized to s∈[0,1].
  const double duration = trajectory.end_time() - trajectory.start_time();
  std::vector<double> normalized_knots = trajectory.basis().knots();
  for (auto& knot : normalized_knots) {
    knot /= duration;
  }
  basis_ = BsplineBasis<double>(trajectory.basis().order(), normalized_knots);

  control_points_ = prog_.NewContinuousVariables(
      num_positions_, num_control_points_, "control_point");
  duration_ = prog_.NewContinuousVariables(1, "duration")[0];
  // duration >= 0.
  prog_.AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(),
                                 duration_);

  SetInitialGuess(trajectory);

  // Create symbolic curves to enable creating linear constraints on the
  // positions and its derivatives.
  sym_r_ = std::make_unique<BsplineTrajectory<symbolic::Expression>>(
      basis_, EigenToStdVector<Expression>(control_points_.cast<Expression>()));
  sym_rdot_ =
      dynamic_pointer_cast_or_throw<BsplineTrajectory<symbolic::Expression>>(
          sym_r_->MakeDerivative());
  sym_rddot_ =
      dynamic_pointer_cast_or_throw<BsplineTrajectory<symbolic::Expression>>(
          sym_rdot_->MakeDerivative());
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
  std::vector<double> scaled_knots = basis_.knots();
  for (auto& knot : scaled_knots) {
    knot *= duration;
  }

  return BsplineTrajectory<double>(
      BsplineBasis<double>(basis_.order(), scaled_knots),
      EigenToStdVector<double>(result.GetSolution(control_points_)));
}

void KinematicTrajectoryOptimization::AddPathPositionConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<symbolic::Expression> sym_r_value = sym_r_->value(s);
  prog_.AddLinearConstraint(lb <= sym_r_value && sym_r_value <= ub);
}

void KinematicTrajectoryOptimization::AddPathPositionConstraint(
    const std::shared_ptr<Constraint>& constraint, double s) {
  DRAKE_DEMAND(constraint->num_vars() == num_positions_);
  DRAKE_DEMAND(0 <= s && s <= 1);
  std::vector<double> basis_function_values;
  basis_function_values.reserve(basis_.order());
  std::vector<int> active_control_point_indices =
      basis_.ComputeActiveBasisFunctionIndices(s);
  const int num_active_control_points =
      static_cast<int>(active_control_point_indices.size());
  VectorXDecisionVariable var_vector(num_active_control_points *
                                     num_positions());
  for (int i = 0; i < num_active_control_points; ++i) {
    const int control_point_index = active_control_point_indices[i];
    basis_function_values.push_back(
        basis_.EvaluateBasisFunctionI(control_point_index, s));
    var_vector.segment(i * num_positions(), num_positions()) =
        control_points_.col(control_point_index);
  }
  prog_.AddConstraint(
      std::make_shared<PathConstraint>(constraint, basis_function_values),
      var_vector);
}

void KinematicTrajectoryOptimization::AddPathVelocityConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<symbolic::Expression> sym_rdot_value = sym_rdot_->value(s);
  prog_.AddLinearConstraint(lb <= sym_rdot_value && sym_rdot_value <= ub);
}

void KinematicTrajectoryOptimization::AddPathAccelerationConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<symbolic::Expression> sym_rddot_value = sym_rddot_->value(s);
  prog_.AddLinearConstraint(lb <= sym_rddot_value && sym_rddot_value <= ub);
}

void KinematicTrajectoryOptimization::AddDurationConstraint(
    optional<double> lb, optional<double> ub) {
  prog_.AddBoundingBoxConstraint(
      lb.value_or(0), ub.value_or(std::numeric_limits<double>::infinity()),
      duration_);
}

void KinematicTrajectoryOptimization::AddPositionBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  // This leverages the convex hull property of the B-splines: if all of the
  // control points satisfy these convex constraints and the curve is inside
  // the convex hull of these constraints, then the curve satisfies the
  // constraints for all s (and therefore all t).
  for (int i = 0; i < num_control_points(); ++i) {
    prog_.AddBoundingBoxConstraint(lb, ub, control_points_.col(i));
  }
}

void KinematicTrajectoryOptimization::AddVelocityBounds(
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
  for (int i = 0; i < sym_rdot_->num_control_points(); ++i) {
    prog_.AddLinearConstraint(sym_rdot_->control_points()[i] >=
                                  duration_ * lb &&
                              sym_rdot_->control_points()[i] <= duration_ * ub);
  }
}

void KinematicTrajectoryOptimization::AddDurationCost(double weight) {
  prog_.AddLinearCost(weight * duration_);
}

void KinematicTrajectoryOptimization::AddPathLengthCost(
    double weight, bool use_conic_constraint) {
  MatrixXd A(num_positions_, 2 * num_positions_);
  A.leftCols(num_positions_) =
      weight * MatrixXd::Identity(num_positions_, num_positions_);
  A.rightCols(num_positions_) =
      -weight * MatrixXd::Identity(num_positions_, num_positions_);
  const VectorXd b = VectorXd::Zero(num_positions_);
  VectorXDecisionVariable vars(2 * num_positions_);
  for (int i = 1; i < num_control_points(); ++i) {
    vars.head(num_positions_) = control_points_.col(i);
    vars.tail(num_positions_) = control_points_.col(i-1);
    if (use_conic_constraint) {
      prog_.AddL2NormCostUsingConicConstraint(A, b, vars);
    } else {
      prog_.AddL2NormCost(A, b, vars);
    }
  }
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
