#include "humanoid_status.h"
#include <iostream>

const Vector3d HumanoidStatus::kFootToContactOffset = Vector3d(0, 0, -0.09);
const Vector3d HumanoidStatus::kFootToSensorOffset =
    Vector3d(0.0215646, 0.0, -0.051054);

void HumanoidStatus::FillKinematics(const RigidBody& body, Isometry3d* pose,
                                    Vector6d* vel, MatrixXd* J,
                                    Vector6d* Jdot_times_v,
                                    const Vector3d& local_offset) const {
  *pose = Isometry3d::Identity();
  pose->translation() = local_offset;
  *pose = robot_->relativeTransform(cache_, 0, body.get_body_index()) * (*pose);

  *vel = GetTaskSpaceVel(*(robot_), cache_, body, local_offset);
  *J = GetTaskSpaceJacobian(*(robot_), cache_, body, local_offset);
  *Jdot_times_v =
      GetTaskSpaceJacobianDotTimesV(*(robot_), cache_, body, local_offset);
}

void HumanoidStatus::Update(double t, const VectorXd& q, const VectorXd& v,
                            const VectorXd& trq, const Vector6d& l_ft,
                            const Vector6d& r_ft, const Matrix3d& rot) {
  if (q.size() != position_.size() || v.size() != velocity_.size() ||
      trq.size() != joint_torque_.size()) {
    throw std::runtime_error("robot_ state update dimension mismatch");
  }

  time_ = t;
  position_ = q;
  velocity_ = v;
  joint_torque_ = trq;

  cache_.initialize(position_, velocity_);
  robot_->doKinematics(cache_, true);

  M_ = robot_->massMatrix(cache_);
  const RigidBodyTree::BodyToWrenchMap<double> no_external_wrenches;
  bias_term_ = robot_->dynamicsBiasTerm(cache_, no_external_wrenches);

  // com
  com_ = robot_->centerOfMass(cache_);
  J_com_ = robot_->centerOfMassJacobian(cache_);
  Jdot_times_v_com_ = robot_->centerOfMassJacobianDotTimesV(cache_);
  comd_ = J_com_ * v;

  // body parts
  FillKinematics(*pelv_.body, &pelv_.pose, &pelv_.vel, &pelv_.J,
                 &pelv_.Jdot_times_v);
  FillKinematics(*torso_.body, &torso_.pose, &torso_.vel, &torso_.J,
                 &torso_.Jdot_times_v);
  for (int s = 0; s < 2; s++) {
    FillKinematics(*foot_[s].body, &foot_[s].pose, &foot_[s].vel, &foot_[s].J,
                   &foot_[s].Jdot_times_v, kFootToContactOffset);
    FillKinematics(*foot_sensor_[s].body, &foot_sensor_[s].pose,
                   &foot_sensor_[s].vel, &foot_sensor_[s].J,
                   &foot_sensor_[s].Jdot_times_v, kFootToSensorOffset);
  }

  // ft sensor
  foot_wrench_in_body_frame_[Side::LEFT] = l_ft;
  foot_wrench_in_body_frame_[Side::RIGHT] = r_ft;
  for (int i = 0; i < 2; i++) {
    foot_wrench_in_body_frame_[i].head(3) =
        rot * foot_wrench_in_body_frame_[i].head(3);
    foot_wrench_in_body_frame_[i].tail(3) =
        rot * foot_wrench_in_body_frame_[i].tail(3);
    foot_wrench_in_world_frame_[i].head(3) =
        foot_sensor_[i].pose.linear() * foot_wrench_in_body_frame_[i].head(3);
    foot_wrench_in_world_frame_[i].tail(3) =
        foot_sensor_[i].pose.linear() * foot_wrench_in_body_frame_[i].tail(3);
  }

  // cop
  Vector2d cop_w[2];
  double Fz[2];
  for (int i = 0; i < 2; i++) {
    Fz[i] = foot_wrench_in_world_frame_[i][5];
    if (fabs(Fz[i]) < 1e-3) {
      cop_in_body_frame_[i][0] = 0;
      cop_in_body_frame_[i][1] = 0;
      cop_w[i][0] = foot_sensor_[i].pose.translation()[0];
      cop_w[i][1] = foot_sensor_[i].pose.translation()[1];
    } else {
      // cop relative to the ft sensor
      cop_in_body_frame_[i][0] =
          -foot_wrench_in_body_frame_[i][1] / foot_wrench_in_body_frame_[i][5];
      cop_in_body_frame_[i][1] =
          foot_wrench_in_body_frame_[i][0] / foot_wrench_in_body_frame_[i][5];

      cop_w[i][0] = -foot_wrench_in_world_frame_[i][1] / Fz[i] +
                    foot_sensor_[i].pose.translation()[0];
      cop_w[i][1] = foot_wrench_in_world_frame_[i][0] / Fz[i] +
                    foot_sensor_[i].pose.translation()[1];
    }
  }

  // This is assuming that both feet are on the same horizontal surface.
  cop_ = (cop_w[Side::LEFT] * Fz[Side::LEFT] +
          cop_w[Side::RIGHT] * Fz[Side::RIGHT]) /
         (Fz[Side::LEFT] + Fz[Side::RIGHT]);
}
