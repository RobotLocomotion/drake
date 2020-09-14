#pragma once
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/inverse_kinematics/global_inverse_kinematics.h"

namespace drake {
namespace multibody {
std::unique_ptr<MultibodyPlant<double>> ConstructKuka();

std::unique_ptr<MultibodyPlant<double>> ConstructSingleBody();

class KukaTest : public ::testing::Test {
 public:
  KukaTest();

  ~KukaTest() override{};

  /**
   * Given the solution computed from global IK, reconstruct the posture, then
   * compute the body pose using forward kinematics with the reconstructed
   * posture. Compare that forward kinematics body pose, with the body pose
   * in the global IK.
   */
  void CheckGlobalIKSolution(const solvers::MathematicalProgramResult& result,
                             double pos_tol, double orient_tol) const;

  /**
   * Solve the inverse kinematics problem from the nonlinear solver.
   * @param ee_pos_lb_W Lower bound of the end effector position.
   * @param ee_pos_ub_W Upper bound of the end effector position.
   * @param ee_orient Desired end effector orientation.
   * @param angle_tol Tolerance on end effector orientation error.
   * @param q_guess Initial guess for nonlinear IK solution.
   * @param q_nom Desired posture.
   * @param info_expected Expected return status from nonlinear IK
   * @return The nonlinear IK solution.
   */
  Eigen::VectorXd CheckNonlinearIK(const Eigen::Vector3d& ee_pos_lb_W,
                                   const Eigen::Vector3d& ee_pos_ub_W,
                                   const Eigen::Quaterniond& ee_orient,
                                   double angle_tol,
                                   const Eigen::Matrix<double, 7, 1>& q_guess,
                                   const Eigen::Matrix<double, 7, 1>& q_nom,
                                   bool ik_success_expected) const;

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  GlobalInverseKinematics global_ik_;
  BodyIndex ee_idx_;  // end effector's body index.
};
}  // namespace multibody
}  // namespace drake
