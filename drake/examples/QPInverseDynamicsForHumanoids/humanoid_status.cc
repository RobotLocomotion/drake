#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

#include <iostream>

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// TODO(siyuan.feng@tri.global): These are hard coded for Valkyrie, and they
// should be included in the model file or loaded from a separate config file.
const Eigen::Vector3d HumanoidStatus::kFootToSoleOffset =
    Eigen::Vector3d(0, 0, -0.09);
const Eigen::Vector3d HumanoidStatus::kFootToSensorPositionOffset =
    Eigen::Vector3d(0.0215646, 0.0, -0.051054);
const Eigen::Matrix3d HumanoidStatus::kFootToSensorRotationOffset =
    Eigen::Matrix3d(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()));

void HumanoidStatus::Update(
    double t, const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& joint_torque,
    const Eigen::Ref<const Eigen::Vector6d>& l_wrench,
    const Eigen::Ref<const Eigen::Vector6d>& r_wrench) {
  if (q.size() != position_.size() || v.size() != velocity_.size() ||
      joint_torque.size() != joint_torque_.size()) {
    throw std::runtime_error("robot state update dimension mismatch.");
  }

  time_ = t;
  position_ = q;
  velocity_ = v;
  joint_torque_ = joint_torque;

  cache_.initialize(position_, velocity_);
  robot_.doKinematics(cache_, true);

  M_ = robot_.massMatrix(cache_);
  drake::eigen_aligned_std_unordered_map<RigidBody const*,
                                         drake::TwistVector<double>> f_ext;
  bias_term_ = robot_.dynamicsBiasTerm(cache_, f_ext);

  // com
  com_ = robot_.centerOfMass(cache_);
  J_com_ = robot_.centerOfMassJacobian(cache_);
  Jdot_times_v_com_ = robot_.centerOfMassJacobianDotTimesV(cache_);
  comd_ = J_com_ * velocity_;
  centroidal_momentum_matrix_ = robot_.centroidalMomentumMatrix(cache_);
  centroidal_momentum_matrix_dot_times_v_ =
      robot_.centroidalMomentumMatrixDotTimesV(cache_);
  centroidal_momentum_ = centroidal_momentum_matrix_ * velocity_;

  // body parts
  for (BodyOfInterest& body_of_interest : bodies_of_interest_)
    body_of_interest.Update(robot_, cache_);

  // ft sensor
  foot_wrench_raw_[Side::LEFT] = l_wrench;
  foot_wrench_raw_[Side::RIGHT] = r_wrench;
  for (int i = 0; i < 2; i++) {
    // Make H1 = H_sensor_to_sole.
    // Assuming the sole frame has the same orientation as the foot frame.
    Eigen::Isometry3d H1;
    H1.linear() = kFootToSensorRotationOffset.transpose();
    H1.translation() = -kFootToSensorPositionOffset + kFootToSoleOffset;

    foot_wrench_in_sole_frame_[i] =
        transformSpatialForce(H1, foot_wrench_raw_[i]);

    // H2 = transformation from sensor frame to a frame that is aligned with the
    // world frame, and is located at the origin of the foot frame.
    Eigen::Isometry3d H2;
    H2.linear() =
        foot(i).pose().linear() * kFootToSensorRotationOffset.transpose();
    H2.translation() =
        foot(i).pose().translation() - foot_sensor(i).pose().translation();

    foot_wrench_in_world_frame_[i] =
        transformSpatialForce(H2, foot_wrench_raw_[i]);
  }

  // Compute center of pressure (CoP)
  Eigen::Vector2d cop_w[2];
  double Fz[2] = {foot_wrench_in_world_frame_[Side::LEFT][5],
                  foot_wrench_in_world_frame_[Side::RIGHT][5]};
  for (int i = 0; i < 2; i++) {
    // Ignore CoP computation if normal force is small
    if (fabs(foot_wrench_raw_[i][5]) < 1) {
      cop_in_sole_frame_[i][0] = 0;
      cop_in_sole_frame_[i][1] = 0;
      cop_w[i][0] = foot(i).pose().translation()[0];
      cop_w[i][1] = foot(i).pose().translation()[1];
    } else {
      // CoP relative to the ft sensor
      cop_in_sole_frame_[i][0] =
          -foot_wrench_in_sole_frame_[i][1] / foot_wrench_in_sole_frame_[i][5];
      cop_in_sole_frame_[i][1] =
          foot_wrench_in_sole_frame_[i][0] / foot_wrench_in_sole_frame_[i][5];

      // CoP in the world frame
      cop_w[i][0] = -foot_wrench_in_world_frame_[i][1] / Fz[i] +
                    foot(i).pose().translation()[0];
      cop_w[i][1] = foot_wrench_in_world_frame_[i][0] / Fz[i] +
                    foot(i).pose().translation()[1];
    }
  }

  // This is assuming that both feet are on the same horizontal surface.
  cop_ = (cop_w[Side::LEFT] * Fz[Side::LEFT] +
          cop_w[Side::RIGHT] * Fz[Side::RIGHT]) /
         (Fz[Side::LEFT] + Fz[Side::RIGHT]);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
