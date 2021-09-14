#pragma once

#include <memory>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {

using solvers::Binding;
using solvers::BoundingBoxConstraint;
using solvers::LinearConstraint;
using solvers::LinearCost;
using trajectories::PiecewisePolynomial;

/**
 * Selects how linear constraints are enforced for Toppra's optimization.
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

  /**
   * Contains the lower and upper bound for the bounding box constraint imposed
   * on x at each gridpoint. The length of each vector is equal to the number of
   * gridpoints.
   */
  typedef std::tuple<Eigen::VectorXd, Eigen::VectorXd>
      ToppraBoundingBoxConstraint;
  /**
   * Contains the coefficients, lower bound and upper bound for the linear
   * constraint at each gridpoint. The number of columns in the coefficent
   * matrix is twice the number of gridpoints and each pair of columns
   * correspond to the coefficients on [x, u] for the linear constraint at the
   * respective gridpoint. The number of columns in the lower and upper bound
   * matrices is equal to the number of gridpoints and each column contains the
   * bounds for the constraint at the respective gridpoint.
   */
  typedef std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
      ToppraLinearConstraint;

  ~Toppra() {}

  /**
   * Constructs an inverse kinematics problem for a MultibodyPlant.
   * This constructor will create and own a context for @param plant.
   * @param path The trajectory on which the toppra problem will be solved.
   * @param plant The robot that will follow the solved trajectory.  Used for
   *              enforcing torque and frame specific constraints.
   * @param gridpoints The points along the path to discretize the problem and
   *                   enforce constraints at.  The first and last gridpoint
   *                   must equal the path start and end time respectively.
   *                   Gridpoints must also be monotonically increasing.
   */
  explicit Toppra(const PiecewisePolynomial<double>& path,
                  const MultibodyPlant<double>& plant,
                  const Eigen::Ref<const Eigen::VectorXd>& gridpoints);

  /**
   * Takes a path and generates a sequence of gridpoints selected to control
   * the interpolation error of the optimization. The distance between
   * gridpoints must be below `max_seg_length`, there must be at least
   * `min_points` number of gridpoints and the interpolation error, estimated
   * with the equation
   * <pre>
   *   err_{est} = 0.5 * \max(|q̈ * Δ_{segment}^2|),
   * </pre>
   * must be less than `max_err`.  Any segments that are too long or have too
   * much error are split in half.  This results in more points in parts of the
   * path with higher curvature. All grid points will lie between
   * path.start_time() and path.end_time().
   */
  static Eigen::VectorXd CalcGridpts(const PiecewisePolynomial<double>& path,
                                     const CalcGridPointsOptions& options);

  /**
   * Solves the toppra optimization and returns the time optimized trajectory.
   * Resulting trajectory has the same start time as the original path.
   * @note The trajectory may not perfectly follow the original path due to
   * refitting a piecewise cubic but will pass through the points defined by the
   * gridpoints.
   */
  std::optional<PiecewisePolynomial<double>> Solve();

  /**
   * Adds a constant velocity limit to all the degrees of freedom in the plant.
   * The limits must be arranged in the same order as the entries in the path.
   * @param lower_limit The lower velocity limit for each degree of freedom.
   * @param upper_limit The upper velocity limit for each degree of freedom.
   */
  ToppraBoundingBoxConstraint& AddJointVelocityLimit(
      const Eigen::Ref<const Eigen::VectorXd>& lower_limit,
      const Eigen::Ref<const Eigen::VectorXd>& upper_limit);

  /**
   * Adds a constant acceleration limit to all the degrees of freedom in the
   * plant. The limits must be arranged in the same order as the entries in the
   * path.
   * @param lower_limit The lower acceleration limit for each degree of freedom.
   * @param upper_limit The upper acceleration limit for each degree of freedom.
   * @param discretization The discretization scheme to use for this linear
   *                       constraint. See ToppraDiscretization for details.
   */
  std::pair<Binding<LinearConstraint>, Binding<LinearConstraint>>
  AddJointAccelerationLimit(
      const Eigen::Ref<const Eigen::VectorXd>& lower_limit,
      const Eigen::Ref<const Eigen::VectorXd>& upper_limit,
      ToppraDiscretization discretization =
          ToppraDiscretization::kInterpolation);

 private:
  /**
   * Performs the backward pass step of TOPPRA, returning the controllable set
   * at each gridpoint. The rows are respectively the lower and upper bound for
   * the path velocity at each knot point.
   * @param s_dot_0 The path velocity at the beginning of the path.
   * @param s_dot_N The path velocity at the end of the path.
   */
  std::optional<Eigen::Matrix2Xd> ComputeBackwardPass(double s_dot_0,
                                                      double s_dot_N);

  /**
   * Performs the forward pass step of TOPPRA, computing the greediest
   * acceleration at each gridpoint that remains within the controllable set.
   * @param s_dot_0 The path velocity at the beginning of the path.
   * @param K The controllable set that the path velocity must stay within at
   *          each gridpoint. Each column consists of the lower and upper bound
   *          for the path velocity and there must be gridpoint.size() number of
   *          columns.
   */
  Eigen::VectorXd ComputeForwardPass(double s_dot_0, const Eigen::Matrix2Xd& K);

  std::unique_ptr<solvers::MathematicalProgram> backward_prog_;
  solvers::VectorXDecisionVariable backward_x_;
  solvers::VectorXDecisionVariable backward_u_;
  Binding<LinearCost> backward_cost_;
  Binding<LinearConstraint> backward_continuity_con_;
  std::unique_ptr<solvers::MathematicalProgram> forward_prog_;
  solvers::VectorXDecisionVariable forward_u_;
  Binding<LinearCost> forward_cost_;
  Binding<LinearConstraint> forward_continuity_con_;
  const PiecewisePolynomial<double>& path_;
  const MultibodyPlant<double>& plant_;
  Eigen::VectorXd gridpoints_;
  Binding<BoundingBoxConstraint> x_bounding_box_con_;
  std::vector<ToppraBoundingBoxConstraint> x_bounds_;
  std::unordered_map<Binding<LinearConstraint>, ToppraLinearConstraint>
      backward_lin_constraint_;
  std::unordered_map<Binding<LinearConstraint>, ToppraLinearConstraint>
      forward_lin_constraint_;
};
}  // namespace multibody
}  // namespace drake
