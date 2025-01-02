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
using Eigen::VectorXd;
using std::nullopt;
using std::optional;

namespace drake {
namespace planning {
namespace trajectory_optimization {

const double kInf = std::numeric_limits<double>::infinity();

using math::BsplineBasis;
using math::EigenToStdVector;
using math::ExtractValue;
using math::InitializeAutoDiff;
using math::StdVectorToEigen;
using solvers::Binding;
using solvers::BoundingBoxConstraint;
using solvers::Constraint;
using solvers::Cost;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Variables;
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

/* Implements a constraint of the form
  wrapped_constraint([q, v]), where
  duration = x[0]
  q = M_pos * x[1:num_pos_vars+1]
  v = M_vel * x[-num_vel_vars:] / duration

  TODO(russt): M_pos and M_vel are predictably sparse, and we could handle that
  here if performance demands it.
*/
class WrappedVelocityConstraint : public Constraint {
 public:
  WrappedVelocityConstraint(std::shared_ptr<Constraint> wrapped_constraint,
                            Eigen::MatrixXd M_pos, Eigen::MatrixXd M_vel)
      : Constraint(wrapped_constraint->num_outputs(),
                   M_pos.cols() + M_vel.cols() + 1,
                   wrapped_constraint->lower_bound(),
                   wrapped_constraint->upper_bound()),
        wrapped_constraint_(wrapped_constraint),
        M_pos_{std::move(M_pos)},
        M_vel_{std::move(M_vel)} {
    DRAKE_DEMAND(M_pos_.rows() + M_vel_.rows() ==
                 wrapped_constraint_->num_vars());
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
    VectorX<AutoDiffXd> qv(wrapped_constraint_->num_vars());
    qv << M_pos_ * x.segment(1, M_pos_.cols()),
        M_vel_ * x.tail(M_vel_.cols()) / duration;
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
  const Eigen::MatrixXd M_pos_;
  const Eigen::MatrixXd M_vel_;
};

/* Implements a constraint of the form:
    duration = x[0]
    lb <= M * x[1:] / duration^order <= ub
*/
class DerivativeConstraint : public Constraint {
 public:
  DerivativeConstraint(const Eigen::MatrixXd& M, int derivative_order,
                       const Eigen::Ref<const VectorXd>& lb,
                       const Eigen::Ref<const VectorXd>& ub)
      : Constraint(M.rows(), M.cols() + 1, lb, ub),
        M_{M},
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
    *y = M_ * x.tail(M_.cols()) / pow(duration, derivative_order_);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::runtime_error(
        "DerivativeConstraint does not support evaluation with Expression.");
  }

 private:
  const Eigen::MatrixXd M_;
  const int derivative_order_;
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
  auto duration_bbox_binding =
      prog_.AddBoundingBoxConstraint(0, kInf, duration_);
  duration_bbox_binding.evaluator()->set_description(
      "positive duration constraint");

  SetInitialGuess(trajectory);

  // Create symbolic curves to enable creating linear constraints on the
  // positions and its derivatives.
  // TODO(russt): Consider computing these only the first time they are used.
  sym_r_ = std::make_unique<BsplineTrajectory<symbolic::Expression>>(
      basis_, EigenToStdVector<Expression>(control_points_.cast<Expression>()));
  sym_rdot_ =
      dynamic_pointer_cast_or_throw<BsplineTrajectory<symbolic::Expression>>(
          sym_r_->MakeDerivative());
  sym_rddot_ =
      dynamic_pointer_cast_or_throw<BsplineTrajectory<symbolic::Expression>>(
          sym_rdot_->MakeDerivative());
  sym_rdddot_ =
      dynamic_pointer_cast_or_throw<BsplineTrajectory<symbolic::Expression>>(
          sym_rddot_->MakeDerivative());
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

Binding<LinearConstraint>
KinematicTrajectoryOptimization::AddPathPositionConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<symbolic::Expression> sym_r_value = sym_r_->value(s);
  auto binding =
      prog_.AddLinearConstraint(lb <= sym_r_value && sym_r_value <= ub);
  binding.evaluator()->set_description("path position constraint");
  return binding;
}

Binding<Constraint> KinematicTrajectoryOptimization::AddPathPositionConstraint(
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
  auto binding = prog_.AddConstraint(
      std::make_shared<PathConstraint>(constraint, basis_function_values),
      var_vector);
  binding.evaluator()->set_description("path position constraint");
  return binding;
}

Binding<LinearConstraint>
KinematicTrajectoryOptimization::AddPathVelocityConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<symbolic::Expression> sym_rdot_value = sym_rdot_->value(s);
  auto binding =
      prog_.AddLinearConstraint(lb <= sym_rdot_value && sym_rdot_value <= ub);
  binding.evaluator()->set_description("path velocity constraint");
  return binding;
}

Binding<Constraint>
KinematicTrajectoryOptimization::AddVelocityConstraintAtNormalizedTime(
    const std::shared_ptr<solvers::Constraint>& constraint, double s) {
  DRAKE_DEMAND(constraint->num_vars() == 2 * num_positions_);
  DRAKE_DEMAND(0 <= s && s <= 1);

  VectorX<Expression> r = sym_r_->value(s);
  VectorX<Expression> rdot = sym_rdot_->value(s);
  VectorXDecisionVariable vars_pos, vars_vel;
  std::unordered_map<symbolic::Variable::Id, int> unused_map;
  std::tie(vars_pos, unused_map) = symbolic::ExtractVariablesFromExpression(r);
  std::tie(vars_vel, unused_map) =
      symbolic::ExtractVariablesFromExpression(rdot);
  Eigen::MatrixXd M_pos(num_positions(), vars_pos.size());
  Eigen::MatrixXd M_vel(num_positions(), vars_vel.size());
  symbolic::DecomposeLinearExpressions(r, vars_pos, &M_pos);
  symbolic::DecomposeLinearExpressions(rdot, vars_vel, &M_vel);
  VectorXDecisionVariable duration_and_vars(1 + vars_pos.size() +
                                            vars_vel.size());
  duration_and_vars << duration_, vars_pos, vars_vel;
  auto wrapped_constraint = std::make_shared<WrappedVelocityConstraint>(
      constraint, std::move(M_pos), std::move(M_vel));
  auto binding = prog_.AddConstraint(wrapped_constraint, duration_and_vars);
  binding.evaluator()->set_description("velocity constraint");
  return binding;
}

Binding<LinearConstraint>
KinematicTrajectoryOptimization::AddVelocityLinearConstraintAtNormalizedTime(
    const std::shared_ptr<solvers::LinearConstraint>& constraint, double s) {
  DRAKE_DEMAND(constraint->num_vars() == num_positions_);
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<Expression> rdot = sym_rdot_->value(s);
  // `constraint` is
  // lb <= A * qdot <= ub
  // we have qdot = rdot / T
  // So the linear constraint we impose is
  // lb <= A * rdot/T <= ub
  // Namely
  // A * rdot - lb * T >= 0
  // A * rdot - ub * T <= 0
  auto [vars_vel, _] = symbolic::ExtractVariablesFromExpression(rdot);
  Eigen::MatrixXd M_vel(num_positions(), vars_vel.size());
  symbolic::DecomposeLinearExpressions(rdot, vars_vel, &M_vel);
  // The constraint is
  // A * M_vel * vars_vel - lb * T >= 0
  // A * M_vel * vars_vel - ub * T <= 0
  // We call this new constraint as
  // lb_new <= A_new * vars_all <= ub_new
  // where vars_all = [vars_vel, T]
  Eigen::MatrixXd A_new(2 * constraint->num_constraints(), vars_vel.rows() + 1);
  Eigen::VectorXd lb_new(A_new.rows());
  Eigen::VectorXd ub_new(A_new.rows());
  int A_new_row_count = 0;
  const auto& A = constraint->GetDenseA();
  for (int i = 0; i < constraint->num_constraints(); ++i) {
    // If lb(i) == ub(i), then we only add a linear equality constraint
    // A.row(i) * M_vel * vars_vel - lb(i) * T = 0.
    // Otherwise, we add the constraint
    // A.row(i) * M_vel * vars_vel - lb(i) * T >= 0
    // and
    // A.row(i) * M_vel * vars_vel - ub(i) * T <= 0
    if (constraint->lower_bound()(i) == constraint->upper_bound()(i)) {
      DRAKE_DEMAND(!std::isinf(constraint->lower_bound()(i)));
      A_new.row(A_new_row_count) << A.row(i) * M_vel,
          -constraint->lower_bound()(i);
      lb_new(A_new_row_count) = 0;
      ub_new(A_new_row_count) = 0;
      ++A_new_row_count;
    } else {
      if (!std::isinf(constraint->lower_bound()(i))) {
        A_new.row(A_new_row_count) << constraint->GetDenseA().row(i) * M_vel,
            -constraint->lower_bound()(i);
        lb_new(A_new_row_count) = 0;
        ub_new(A_new_row_count) = kInf;
        ++A_new_row_count;
      }
      if (!std::isinf(constraint->upper_bound()(i))) {
        A_new.row(A_new_row_count) << constraint->GetDenseA().row(i) * M_vel,
            -constraint->upper_bound()(i);
        lb_new(A_new_row_count) = -kInf;
        ub_new(A_new_row_count) = 0;
        ++A_new_row_count;
      }
    }
  }
  auto binding = prog_.AddLinearConstraint(
      A_new.topRows(A_new_row_count), lb_new.topRows(A_new_row_count),
      ub_new.topRows(A_new_row_count),
      {vars_vel, Vector1<symbolic::Variable>(duration_)});
  binding.evaluator()->set_description("velocity linear constraint");
  return binding;
}

Binding<LinearConstraint>
KinematicTrajectoryOptimization::AddPathAccelerationConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<symbolic::Expression> sym_rddot_value = sym_rddot_->value(s);
  auto binding =
      prog_.AddLinearConstraint(lb <= sym_rddot_value && sym_rddot_value <= ub);
  binding.evaluator()->set_description("path acceleration constraint");
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
  std::vector<Binding<LinearConstraint>> binding;
  for (int i = 0; i < sym_rdot_->num_control_points(); ++i) {
    binding.emplace_back(prog_.AddLinearConstraint(
        sym_rdot_->control_points()[i] >= duration_ * lb &&
        sym_rdot_->control_points()[i] <= duration_ * ub));
    binding[i].evaluator()->set_description(
        fmt::format("velocity bound {}", i));
  }
  return binding;
}

std::vector<std::vector<Binding<Constraint>>>
KinematicTrajectoryOptimization::AddAccelerationBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());

  // We have t = duration * s. So dsdt = 1/duration, d²sdt² = 0. Then q̈(t) =
  // r̈(s) * dsdt^2.

  // This again leverages the convex hull property to enforce the guarantee ∀t
  // by only constraining the values at the control points.
  Eigen::RowVectorXd M;
  VectorXDecisionVariable vars, duration_and_vars;
  std::unordered_map<symbolic::Variable::Id, int> unused_map;
  std::vector<std::vector<Binding<Constraint>>> binding(
      sym_rddot_->num_control_points());
  for (int i = 0; i < sym_rddot_->num_control_points(); ++i) {
    for (int j = 0; j < num_positions(); ++j) {
      std::tie(vars, unused_map) = symbolic::ExtractVariablesFromExpression(
          sym_rddot_->control_points()[i](j));
      M.resize(vars.size());
      // TODO(russt): Avoid symbolic here and throughout.
      symbolic::DecomposeLinearExpressions(
          Vector1<Expression>(sym_rddot_->control_points()[i](j)), vars, &M);
      auto con = std::make_shared<DerivativeConstraint>(M, 2, lb.segment<1>(j),
                                                        ub.segment<1>(j));
      duration_and_vars.resize(vars.size() + 1);
      duration_and_vars << duration_, vars;
      binding[i].emplace_back(prog_.AddConstraint(con, duration_and_vars));
      binding[i][j].evaluator()->set_description(
          fmt::format("acceleration bound {}, {}", i, j));
    }
  }
  return binding;
}

std::vector<std::vector<Binding<Constraint>>>
KinematicTrajectoryOptimization::AddJerkBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());

  // Following the derivations above, we have d³qdt³(t) = d³rds³(s) * dsdt*3.

  // This again leverages the convex hull property to enforce the guarantee ∀t
  // by only constraining the values at the control points.
  Eigen::RowVectorXd M;
  VectorXDecisionVariable vars, duration_and_vars;
  std::unordered_map<symbolic::Variable::Id, int> map_var_to_index;
  std::vector<std::vector<Binding<Constraint>>> binding(
      sym_rddot_->num_control_points());
  for (int i = 0; i < sym_rdddot_->num_control_points(); ++i) {
    for (int j = 0; j < num_positions(); ++j) {
      std::tie(vars, map_var_to_index) =
          symbolic::ExtractVariablesFromExpression(
              sym_rdddot_->control_points()[i](j));
      M.resize(vars.size());
      symbolic::DecomposeLinearExpressions(
          Vector1<Expression>(sym_rdddot_->control_points()[i](j)), vars, &M);
      auto con = std::make_shared<DerivativeConstraint>(M, 3, lb.segment<1>(j),
                                                        ub.segment<1>(j));
      duration_and_vars.resize(vars.size() + 1);
      duration_and_vars << duration_, vars;
      binding[i].emplace_back(prog_.AddConstraint(con, duration_and_vars));
      binding[i][j].evaluator()->set_description(
          fmt::format("jerk bound {}, {}", i, j));
    }
  }
  return binding;
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
