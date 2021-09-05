#pragma once

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

enum class ToppraDiscretization { kCollocation, kInterpolation };

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

  typedef std::tuple<Eigen::VectorXd, Eigen::VectorXd>
      ToppraBoundingBoxConstraint;
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
   *                   enforce constraints at.
   */
  explicit Toppra(const PiecewisePolynomial<double>& path,
                  const MultibodyPlant<double>& plant,
                  const Eigen::VectorXd& gridpoints);

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
                                     double max_err = 1e-3, int max_iter = 100,
                                     double max_seg_length = 0.05,
                                     int min_points = 100);

  PiecewisePolynomial<double> Solve();

  ToppraBoundingBoxConstraint& AddJointVelocityLimit(
      const Eigen::VectorXd& lower_limit, const Eigen::VectorXd upper_limit);

  ToppraLinearConstraint& AddJointAccelerationLimit(
      const Eigen::VectorXd& lower_limit, const Eigen::VectorXd upper_limit,
      ToppraDiscretization discretization =
          ToppraDiscretization::kInterpolation);

  // ToppraLinearConstraint& AddJointTorqueLimit(const Eigen::VectorXd&
  // lower_limit,
  //                                            const Eigen::VectorXd
  //                                            upper_limit);

  // ToppraBoundingBoxConstraint& AddFrameVelocityLimit(
  //     const Frame<double>& constraint_frame, const Eigen::VectorXd&
  //     lower_limit, const Eigen::VectorXd upper_limit);

  // ToppraLinearConstraint& AddFrameAccelerationLimit(
  //     const Frame<double>& constraint_frame, const Eigen::VectorXd&
  //     lower_limit, const Eigen::VectorXd upper_limit);

 private:
  /**
   * Performs the backward pass step of TOPPRA, computing the controllable set
   * at each gridpoint.
   */
  Eigen::MatrixXd ComputeBackwardPass(double s_dot_0, double s_dot_N);

  /**
   * Performs the forward pass step of TOPPRA, computing the greediest
   * acceleration at each gridpoint that remains within the controllable set.
   */
  Eigen::VectorXd ComputeForwardPass(double s_dot_0, Eigen::MatrixXd& K);

  std::unique_ptr<solvers::MathematicalProgram> backward_prog_;
  solvers::VectorXDecisionVariable backward_vars_;
  Binding<LinearCost> backward_cost_;
  Binding<LinearConstraint> backward_continuity_con_;
  std::unique_ptr<solvers::MathematicalProgram> forward_prog_;
  solvers::VectorXDecisionVariable forward_vars_;
  Binding<LinearCost> forward_cost_;
  Binding<LinearConstraint> forward_continuity_con_;
  const PiecewisePolynomial<double>& path_;
  const MultibodyPlant<double>& plant_;
  Eigen::VectorXd gridpoints_;
  std::vector<Binding<BoundingBoxConstraint>> bb_constraint_;
  std::vector<ToppraBoundingBoxConstraint> bb_constraint_coeff_;
  std::vector<Binding<LinearConstraint>> backward_lin_constraint_;
  std::vector<Binding<LinearConstraint>> forward_lin_constraint_;
  std::vector<ToppraLinearConstraint> lin_constraint_coeff_;
};
}  // namespace multibody
}  // namespace drake
