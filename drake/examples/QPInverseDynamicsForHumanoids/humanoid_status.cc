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

HumanoidStatus::HumanoidStatus(
    const RigidBodyTree<double>& robot,
    const param_parsers::RigidBodyTreeAliasGroups<double>& alias_group)
    : robot_(&robot), cache_(robot_->CreateKinematicsCache()) {
  time_ = 0;

  // These are humanoid specific special group names, and they do not exsit
  // for manupulators such as the iiwa arm.
  const std::vector<std::string> body_names =
      {"pelvis", "torso", "left_foot", "right_foot"};

  for (const auto& name : body_names) {
    if (alias_group.has_body_group(name)) {
      const RigidBody<double>* body = alias_group.get_body(name);
      bodies_of_interest_.emplace(
          name, BodyOfInterest(name, *body, Vector3<double>::Zero()));
    }
  }

  // Only attach foot sensors if this robot has feet.
  if (alias_group.has_body_group("left_foot")) {
    bodies_of_interest_.emplace(
        "left_foot_sensor",
        BodyOfInterest("left_foot_sensor",
                       *alias_group.get_body("left_foot"),
                       kFootToSensorPositionOffset));
  }

  if (alias_group.has_body_group("right_foot")) {
    bodies_of_interest_.emplace(
        "right_foot_sensor",
        BodyOfInterest("right_foot_sensor",
                       *alias_group.get_body("right_foot"),
                       kFootToSensorPositionOffset));
  }

  position_.resize(robot_->get_num_positions());
  velocity_.resize(robot_->get_num_velocities());
  joint_torque_.resize(robot_->get_num_actuators());

  // Build various lookup maps.
  body_name_to_id_ = std::unordered_map<std::string, int>();
  for (auto it = robot_->bodies.begin(); it != robot_->bodies.end(); ++it) {
    body_name_to_id_[(*it)->get_name()] = it - robot_->bodies.begin();
  }

  name_to_position_index_ = std::unordered_map<std::string, int>();
  for (int i = 0; i < robot_->get_num_positions(); ++i) {
    name_to_position_index_[robot_->get_position_name(i)] = i;
  }
  name_to_velocity_index_ = std::unordered_map<std::string, int>();
  for (int i = 0; i < robot_->get_num_velocities(); ++i) {
    name_to_velocity_index_[robot_->get_velocity_name(i)] = i;
  }
  for (int i = 0; i < robot_->get_num_actuators(); ++i) {
    actuator_name_to_actuator_index_[robot_->actuators.at(i).name_] = i;
  }
}

void HumanoidStatus::Update(
    double t, const Eigen::Ref<const VectorX<double>>& q,
    const Eigen::Ref<const VectorX<double>>& v,
    const Eigen::Ref<const VectorX<double>>& joint_torque,
    const Eigen::Ref<const Vector6<double>>& l_wrench,
    const Eigen::Ref<const Vector6<double>>& r_wrench) {
  if (q.size() != position_.size() || v.size() != velocity_.size() ||
      joint_torque.size() != joint_torque_.size()) {
    throw std::runtime_error("robot state update dimension mismatch.");
  }
  time_ = t;
  position_ = q;
  velocity_ = v;
  joint_torque_ = joint_torque;
  foot_wrench_raw_[Side::LEFT] = l_wrench;
  foot_wrench_raw_[Side::RIGHT] = r_wrench;
  Update();
}

void HumanoidStatus::UpdateKinematics(
    double t, const Eigen::Ref<const VectorX<double>>& q,
    const Eigen::Ref<const VectorX<double>>& v) {
  Update(t, q, v, VectorX<double>::Zero(joint_torque_.size()),
      Vector6<double>::Zero(), Vector6<double>::Zero());
}

void HumanoidStatus::Update() {
  cache_.initialize(position_, velocity_);
  robot_->doKinematics(cache_, true);

  M_ = robot_->massMatrix(cache_);
  drake::eigen_aligned_std_unordered_map<RigidBody<double> const*,
                                         drake::TwistVector<double>>
      f_ext;
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
  for (auto& body_of_interest_pair : bodies_of_interest_)
    body_of_interest_pair.second.Update(*robot_, cache_);

  // Computes CoP only when we have foot sensors.
  if (bodies_of_interest_.find("left_foot_sensor") !=
          bodies_of_interest_.end() &&
      bodies_of_interest_.find("right_foot_sensor") !=
          bodies_of_interest_.end()) {
    // ft sensor
    for (int i = 0; i < 2; ++i) {
      // Make H1 = H_sensor_to_sole.
      // Assuming the sole frame has the same orientation as the foot frame.
      Isometry3<double> H1;
      H1.fromPositionOrientationScale(
          -kFootToSensorPositionOffset + kFootToSoleOffset,
          kFootToSensorRotationOffset.transpose(),
          Vector3<double>::Ones());

      foot_wrench_in_sole_frame_[i] =
          transformSpatialForce(H1, foot_wrench_raw_[i]);

      // H2 = transformation from sensor frame to a frame that is aligned with
      // the world frame, and is located at the origin of the foot frame.
      Isometry3<double> H2;
      H2.fromPositionOrientationScale(
          foot(i).pose().translation() - foot_sensor(i).pose().translation(),
          foot(i).pose().linear() * kFootToSensorRotationOffset.transpose(),
          Vector3<double>::Ones());

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
        cop_in_sole_frame_[i][0] = -foot_wrench_in_sole_frame_[i][1] /
                                   foot_wrench_in_sole_frame_[i][5];
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
}

std::ostream& operator<<(std::ostream& out,
                         const HumanoidStatus& robot_status) {
  out << "Time: " << robot_status.time() << "\n";
  for (int i = 0; i < robot_status.position().size(); ++i) {
    out << robot_status.robot().get_position_name(i) << ": "
        << robot_status.position(i) << ", " << robot_status.velocity(i) << "\n";
  }
  out << "left foot vel: " << robot_status.foot(Side::LEFT).velocity() << "\n";
  out << "right foot vel: " << robot_status.foot(Side::RIGHT).velocity()
      << std::endl;
  return out;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
