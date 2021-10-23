#pragma once

#include <memory>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace multibody {

using solvers::Binding;
using solvers::BoundingBoxConstraint;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::SolverInterface;
using trajectories::PiecewisePolynomial;
using trajectories::Trajectory;

/**
 * Selects how linear constraints are enforced for TOPPRA's optimization.
 * kCollocation - enforces constraints only at each gridpoint.
 * kInterpolation - enforces constrants at each gridpoint and at the following
 * gridpoint using forward integration. Yields higher accuracy at minor
 * computational cost.
 */
enum class ToppraDiscretization { kCollocation, kInterpolation };

struct CalcGridPointsOptions {
  double max_err{1e-3};
  int max_iter{100};
  double max_seg_length{0.05};
  int min_points{100};
};

/**
 * Solves a Time Optimal Path Parameterization based on Reachability Analysis
 * (TOPPRA) to find the fastest traversal of a given path, satisfying the given
 * constraints.
 * The approach is described in "A new approach to Time-Optimal Path
 * Parameterization based on Reachability Analysis" by Hung Pham and Quang Cuong
 * Pham, IEEE Transactions on Robotics, 2018.
 *
 * @ingroup planning
 */
class Toppra {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Toppra)

  ~Toppra() {}

  /**
   * Constructs an inverse kinematics problem for a MultibodyPlant.
   * This constructor will create and own a context for @param plant.
   * @param path The trajectory on which the TOPPRA problem will be solved.
   * @param plant The robot that will follow the solved trajectory.  Used for
   *              enforcing torque and frame specific constraints.
   * @param gridpoints The points along the path to discretize the problem and
   *                   enforce constraints at.  The first and last gridpoint
   *                   must equal the path start and end time respectively.
   *                   Gridpoints must also be monotonically increasing.
   * @note Toppra does not currently support plants that contain bodies with
   * quaternion degrees of freedom. In addition, any plant where q̇ ≠ v will have
   * undefined behavior.
   * @note The path velocity, ṡ(t), is limited to be between 0 and 1e8 to ensure
   * the reachable set calculated in the backward pass is always bounded.
   */
  Toppra(const Trajectory<double>& path, const MultibodyPlant<double>& plant,
         const Eigen::Ref<const Eigen::VectorXd>& gridpoints);

  /**
   * Takes a path and generates a sequence of gridpoints selected to control
   * the interpolation error of the optimization. The gridpoints are selected
   * such that the distance between them is below `max_seg_length`, there are at
   * least `min_points` number of gridpoints and the interpolation error,
   * estimated with the equation
   * <pre>
   *   errₑₛₜ = max(|q̈ Δₛ²|) / 2
   * </pre>
   * where Δₛ is the distance between sequential gridpoints, is less than
   * `max_err`. Gridpoints are selected by adding the midpoint between two
   * gridpoints whenever the distance between them is too large or the estimated
   * error is too high. This results in more points in parts of the path with
   * higher curvature. All grid points will lie between path.start_time() and
   * path.end_time().
   */
  static Eigen::VectorXd CalcGridPoints(const Trajectory<double>& path,
                                        const CalcGridPointsOptions& options);

  // TODO(mpetersen94): Consider adding optional<Solver> argument.
  /**
   * Solves the TOPPRA optimization and returns the time optimized path
   * parameterization s(t). This can be used with the original path q(s) to
   * generate a time parameterized trajectory.
   * The path parameterization has the same start time as the original path's
   * starting break.
   */
  std::optional<PiecewisePolynomial<double>> SolvePathParameterization();

  /**
   * Adds a velocity limit to all the degrees of freedom in the plant. The
   * limits must be arranged in the same order as the entries in the path.
   * @param lower_limit The lower velocity limit for each degree of freedom.
   * @param upper_limit The upper velocity limit for each degree of freedom.
   */
  Binding<BoundingBoxConstraint> AddJointVelocityLimit(
      const Eigen::Ref<const Eigen::VectorXd>& lower_limit,
      const Eigen::Ref<const Eigen::VectorXd>& upper_limit);

  /**
   * Adds an acceleration limit to all the degrees of freedom in the plant. The
   * limits must be arranged in the same order as the entries in the path.
   * @param lower_limit The lower acceleration limit for each degree of freedom.
   * @param upper_limit The upper acceleration limit for each degree of freedom.
   * @param discretization The discretization scheme to use for this linear
   *                       constraint. See ToppraDiscretization for details.
   * @return A pair containing the linear constraints that will enforce the
   *         acceleration limit on the backward pass and forward pass
   *         respectively.
   */
  std::pair<Binding<LinearConstraint>, Binding<LinearConstraint>>
  AddJointAccelerationLimit(
      const Eigen::Ref<const Eigen::VectorXd>& lower_limit,
      const Eigen::Ref<const Eigen::VectorXd>& upper_limit,
      ToppraDiscretization discretization =
          ToppraDiscretization::kInterpolation);

  /**
   * Adds a torque limit to all the degrees of freedom in the plant. The limits
   * must be arranged in the same order as the entries in the path. This
   * constrains the generalized torques applied to the plant and does not reason
   * about contact forces.
   * @param lower_limit The lower torque limit for each degree of freedom.
   * @param upper_limit The upper torque limit for each degree of freedom.
   * @param discretization The discretization scheme to use for this linear
   *                       constraint. See ToppraDiscretization for details.
   * @return A pair containing the linear constraints that will enforce the
   *         torque limit on the backward pass and forward pass respectively.
   */
  std::pair<Binding<LinearConstraint>, Binding<LinearConstraint>>
  AddJointTorqueLimit(const Eigen::Ref<const Eigen::VectorXd>& lower_limit,
                      const Eigen::Ref<const Eigen::VectorXd>& upper_limit,
                      ToppraDiscretization discretization =
                          ToppraDiscretization::kInterpolation);

  /**
   * Adds a limit on the elements of the spatial velocity of the given frame,
   * measured and and expressed in the world frame.  The limits should be given
   * as [ω_WF, v_WF], where ω_WF is the frame's angular velocity and v_WF is the
   * frame's translational velocity.
   * @param constraint_frame The frame to limit the velocity of.
   * @param lower_limit The lower velocity limit for constraint_frame.
   * @param upper_limit The upper velocity limit for constraint_frame.
   * @return The bounding box constraint that will enforce the frame velocity
   *         limit during the backward pass.
   */
  Binding<BoundingBoxConstraint> AddFrameVelocityLimit(
      const Frame<double>& constraint_frame,
      const Eigen::Ref<const Vector6d>& lower_limit,
      const Eigen::Ref<const Vector6d>& upper_limit);

  /**
   * Adds a limit on the magnitude of the translational velocity of the given
   * frame, measured and expressed in the world frame.
   * @param constraint_frame The frame to limit the translational speed of.
   * @param upper_limit The upper translational speed limit for
   *                    constraint_frame.
   * @return The bounding box constraint that will enforce the frame
   *         translational speed limit during the backward pass.
   */
  Binding<BoundingBoxConstraint> AddFrameTranslationalSpeedLimit(
      const Frame<double>& constraint_frame, const double& upper_limit);

  /**
   * Adds a limit on the elements of the spatial acceleration of the given
   * frame, measured and and expressed in the world frame.  The limits should be
   * given as [α_WF, a_WF], where α_WF is the frame's angular acceleration and
   * v_WF is the frame's translational acceleration.
   * @param constraint_frame The frame to limit the acceleration of.
   * @param lower_limit The lower acceleration limit for constraint_frame.
   * @param upper_limit The upper acceleration limit for constraint_frame.
   * @param discretization The discretization scheme to use for this linear
   *                       constraint. See ToppraDiscretization for details.
   * @return A pair containing the linear constraints that will enforce the
   * frame acceleration limit on the backward pass and forward pass
   * respectively.
   */
  std::pair<Binding<LinearConstraint>, Binding<LinearConstraint>>
  AddFrameAccelerationLimit(
      const Frame<double>& constraint_frame,
      const Eigen::Ref<const Vector6d>& lower_limit,
      const Eigen::Ref<const Vector6d>& upper_limit,
      ToppraDiscretization discretization =
          ToppraDiscretization::kInterpolation);

 private:
  /*
   * Performs the backward pass step of TOPPRA, returning the controllable set,
   * K, at each gridpoint. K(0, i) and K(1, i) contain respectively the lower
   * and upper bound of the path velocity at grid point i.
   * @param s_dot_0 The path velocity at the beginning of the path.
   * @param s_dot_N The path velocity at the end of the path.
   * @param solver The solver to use for the optimizations at each gridpoint.
   */
  std::optional<Eigen::Matrix2Xd> ComputeBackwardPass(
      double s_dot_0, double s_dot_N, const SolverInterface& solver);

  /*
   * Performs the forward pass step of TOPPRA, computing the greediest
   * acceleration at each gridpoint that remains within the controllable set.
   * @param s_dot_0 The path velocity at the beginning of the path.
   * @param K The controllable set that the path velocity must stay within at
   *          each gridpoint. K(0, i) and K(1, i) contain respectively the lower
   *          and upper bound of the path velocity at grid point i.
   * @param solver The solver to use for the optimizations at each gridpoint.
   */
  std::optional<std::pair<Eigen::VectorXd, Eigen::VectorXd>> ComputeForwardPass(
      double s_dot_0, const Eigen::Ref<const Eigen::Matrix2Xd>& K,
      const SolverInterface& solver);

  /*
   * Calculates the interpolation constraint coefficients for the forward
   * integrated gridpoints based on the constraint coefficients already
   * calculated for each gridpoint.
   */
  void CalcInterpolationConstraint(Eigen::MatrixXd* A,
                                   Eigen::MatrixXd* lower_bound,
                                   Eigen::MatrixXd* upper_bound);

  /*
   * Contains the lower and upper bound for the bounding box constraint imposed
   * on x at each gridpoint. The bounding box constraint at the i'th grid point
   * is lb(i) <= x <= ub(i).
   */
  struct ToppraBoundingBoxConstraint {
    ToppraBoundingBoxConstraint(const Eigen::VectorXd& lower_limit,
                                const Eigen::VectorXd& upper_limit)
        : lb(lower_limit), ub(upper_limit) {}

    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
  };

  /*
   * Contains the coefficients, lower bound and upper bound for the linear
   * constraint at each gridpoint. The linear constraint at the i'th grid point
   * is lb.col(i) <= ceoffs.middleCols<2>(2*i) * [x;u] <= ub.col(i).
   */
  struct ToppraLinearConstraint {
    ToppraLinearConstraint(const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& lower_limit,
                           const Eigen::MatrixXd& upper_limit)
        : coeffs(A), lb(lower_limit), ub(upper_limit) {}

    Eigen::MatrixXd coeffs;
    Eigen::MatrixXd lb;
    Eigen::MatrixXd ub;
  };

  std::unique_ptr<solvers::MathematicalProgram> backward_prog_;
  solvers::VectorXDecisionVariable backward_x_;
  solvers::VectorXDecisionVariable backward_u_;
  Binding<LinearCost> backward_cost_;
  Binding<LinearConstraint> backward_continuity_con_;
  std::unique_ptr<solvers::MathematicalProgram> forward_prog_;
  solvers::VectorXDecisionVariable forward_u_;
  Binding<LinearCost> forward_cost_;
  Binding<LinearConstraint> forward_continuity_con_;
  const Trajectory<double>& path_;
  const MultibodyPlant<double>& plant_;
  const std::unique_ptr<systems::Context<double>> plant_context_;
  Eigen::VectorXd gridpoints_;
  // x_bounds_ maps a Binding<BoundingBoxConstraint> to its bounds for the
  // backward pass. At the i'th grid point the linear constraint
  // x_bounds_.at(constraint).lb.col(i) <= x
  // <= x_bounds_.at(constraint.ub.col(i) will be imposed for each constraint in
  // x_bounds_.
  std::unordered_map<Binding<BoundingBoxConstraint>,
                     ToppraBoundingBoxConstraint>
      x_bounds_;
  // Note: The coefficients/bounds in backward_lin_constraint_ and
  // forward_lin_constraint_ must be kept in sync to ensure the correct
  // solution.
  // backward_lin_constraint_ maps a Binding<LinearConstraint> to its
  // coefficients/bounds for the backward pass. At the i'th grid point the
  // linear constraint backward_lin_constraint_.at(constraint).lb.col(i) <=
  // backward_lin_constraint_.at(constraint).coeffs.middleCols<2>(2*i) * [x;u]
  // <= backward_lin_constraint_.at(constraint.ub.col(i) will be imposed for
  // each constraint in backward_lin_constraint_.
  std::unordered_map<Binding<LinearConstraint>, ToppraLinearConstraint>
      backward_lin_constraint_;
  // forward_lin_constraint_ maps a Binding<LinearConstraint> to its
  // coefficients/bounds for the forward pass in the same manner as
  // backward_lin_constraint_.
  std::unordered_map<Binding<LinearConstraint>, ToppraLinearConstraint>
      forward_lin_constraint_;
};
}  // namespace multibody
}  // namespace drake
