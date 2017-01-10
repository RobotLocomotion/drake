#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

#include <iostream>

#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// TODO(siyuan.feng): These are hard coded for Valkyrie, and they should be
// included in the model file or loaded from a separate config file.
const Vector3<double> HumanoidStatus::kFootToSoleOffset =
    Vector3<double>(0, 0, -0.09);
const Vector3<double> HumanoidStatus::kFootToSensorPositionOffset =
    Vector3<double>(0.0215646, 0.0, -0.051054);
const Matrix3<double> HumanoidStatus::kFootToSensorRotationOffset =
    Matrix3<double>(AngleAxis<double>(-M_PI, Vector3<double>::UnitX()));

void HumanoidStatus::Update() {
  cache_.initialize(position_, velocity_);
  robot_->doKinematics(cache_, true);

  M_ = robot_->massMatrix(cache_);
  drake::eigen_aligned_std_unordered_map<RigidBody<double> const*,
                                         drake::TwistVector<double>> f_ext;
  bias_term_ = robot_->dynamicsBiasTerm(cache_, f_ext);

  // com
  com_ = robot_->centerOfMass(cache_);
  J_com_ = robot_->centerOfMassJacobian(cache_);
  Jdot_times_v_com_ = robot_->centerOfMassJacobianDotTimesV(cache_);
  comd_ = J_com_ * velocity_;
  centroidal_momentum_matrix_ = robot_->centroidalMomentumMatrix(cache_);
  centroidal_momentum_matrix_dot_times_v_ =
      robot_->centroidalMomentumMatrixDotTimesV(cache_);
  centroidal_momentum_ = centroidal_momentum_matrix_ * velocity_;

  // body parts
  for (BodyOfInterest& body_of_interest : bodies_of_interest_)
    body_of_interest.Update(*robot_, cache_);

  // ft sensor
  for (int i = 0; i < 2; ++i) {
    // Make H1 = H_sensor_to_sole.
    // Assuming the sole frame has the same orientation as the foot frame.
    Isometry3<double> H1;
    H1.linear() = kFootToSensorRotationOffset.transpose();
    H1.translation() = -kFootToSensorPositionOffset + kFootToSoleOffset;

    foot_wrench_in_sole_frame_[i] =
        transformSpatialForce(H1, foot_wrench_raw_[i]);

    // H2 = transformation from sensor frame to a frame that is aligned with the
    // world frame, and is located at the origin of the foot frame.
    Isometry3<double> H2;
    H2.linear() =
        foot(i).pose().linear() * kFootToSensorRotationOffset.transpose();
    H2.translation() =
        foot(i).pose().translation() - foot_sensor(i).pose().translation();

    foot_wrench_in_world_frame_[i] =
        transformSpatialForce(H2, foot_wrench_raw_[i]);
  }

  // Compute center of pressure (CoP)
  Vector2<double> cop_w[2];
  double Fz[2] = {foot_wrench_in_world_frame_[Side::LEFT][5],
                  foot_wrench_in_world_frame_[Side::RIGHT][5]};
  for (int i = 0; i < 2; ++i) {
    // Ignore CoP computation if normal force is small
    if (std::abs(foot_wrench_raw_[i][5]) < 1) {
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

std::ostream& operator<<(std::ostream& out,
                         const HumanoidStatus& robot_status) {
  out << "Time: " << robot_status.time() << std::endl;
  for (int i = 0; i < robot_status.position().size(); ++i) {
    out << robot_status.robot().get_position_name(i) << ": "
        << robot_status.position(i) << ", " << robot_status.velocity(i)
        << std::endl;
  }
  out << "left foot vel: " << robot_status.foot(Side::LEFT).velocity()
      << std::endl;
  out << "right foot vel: " << robot_status.foot(Side::RIGHT).velocity()
      << std::endl;
  return out;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
