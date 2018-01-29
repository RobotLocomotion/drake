#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace manipulation {
namespace planner {

enum class DifferentialInverseKinematicsStatus {
  kSolutionFound,
  kNoSolutionFound,
  kStuck
};

std::ostream& operator<<(std::ostream& os,
                         const DifferentialInverseKinematicsStatus value);

struct DifferentialInverseKinematicsResult {
  optional<VectorX<double>> joint_velocities{};
  DifferentialInverseKinematicsStatus status{
      DifferentialInverseKinematicsStatus::kNoSolutionFound};
};

class DifferentialInverseKinematicsParameters {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      DifferentialInverseKinematicsParameters);

  DifferentialInverseKinematicsParameters(int num_positions = 0,
                                          int num_velocities = 0);

  double get_unconstrained_degrees_of_freedom_velocity_limit() const {
    return unconstrained_degrees_of_freedom_velocity_limit_;
  }

  void set_unconstrained_degrees_of_freedom_velocity_limit(double limit) {
    unconstrained_degrees_of_freedom_velocity_limit_ = limit;
  }

  const VectorX<double>& get_nominal_joint_position() const {
    return nominal_joint_position_;
  }

  void set_nominal_joint_position(
      const VectorX<double>& nominal_joint_position) {
    DRAKE_THROW_UNLESS(nominal_joint_position.size() == get_num_positions());
    nominal_joint_position_ = nominal_joint_position;
  }

  const Vector6<double>& get_end_effector_velocity_gain() const {
    return gain_E_;
  }

  void set_end_effector_velocity_gain(const Vector6<double>& gain_E) {
    DRAKE_THROW_UNLESS((gain_E.array() >= 0).all() &&
                       (gain_E.array() <= 1).all());
    gain_E_ = gain_E;
  }

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  get_joint_position_limits() const {
    return q_bounds_;
  }

  void set_joint_position_limits(
      const std::pair<VectorX<double>, VectorX<double>>& q_bounds) {
    DRAKE_DEMAND(q_bounds.first.size() == get_num_positions());
    DRAKE_DEMAND(q_bounds.second.size() == get_num_positions());
    DRAKE_DEMAND((q_bounds.second.array() >= q_bounds.first.array()).all());
    q_bounds_ = q_bounds;
  }

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  get_joint_velocity_limits() const {
    return v_bounds_;
  }

  void set_joint_velocity_limits(
      const std::pair<VectorX<double>, VectorX<double>>& v_bounds) {
    DRAKE_DEMAND(v_bounds.first.size() == get_num_velocities());
    DRAKE_DEMAND(v_bounds.second.size() == get_num_velocities());
    DRAKE_DEMAND((v_bounds.second.array() >= v_bounds.first.array()).all());
    v_bounds_ = v_bounds;
  }

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  get_joint_acceleration_limits() const {
    return vd_bounds_;
  }

  void set_joint_acceleration_limits(
      const std::pair<VectorX<double>, VectorX<double>>& vd_bounds) {
    DRAKE_DEMAND(vd_bounds.first.size() == get_num_velocities());
    DRAKE_DEMAND(vd_bounds.second.size() == get_num_velocities());
    DRAKE_DEMAND((vd_bounds.second.array() >= vd_bounds.first.array()).all());
    vd_bounds_ = vd_bounds;
  }

  double get_timestep() const { return dt_; }

  void set_timestep(double dt) {
    DRAKE_ASSERT(dt > 0);
    dt_ = dt;
  }

  int get_num_positions() const { return num_positions_; }

  int get_num_velocities() const { return num_velocities_; }

 private:
  int num_positions_{0};
  int num_velocities_{0};
  VectorX<double> nominal_joint_position_;
  optional<std::pair<VectorX<double>, VectorX<double>>> q_bounds_{};
  optional<std::pair<VectorX<double>, VectorX<double>>> v_bounds_{};
  optional<std::pair<VectorX<double>, VectorX<double>>> vd_bounds_{};
  double unconstrained_degrees_of_freedom_velocity_limit_{
      std::numeric_limits<double>::infinity()};
  Vector6<double> gain_E_{Vector6<double>::Ones()};
  double dt_{1};
};

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const VectorX<double> q_current, const VectorX<double>& v_current,
    const VectorX<double>& V, const MatrixX<double>& J,
    const DifferentialInverseKinematicsParameters& parameters);

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const RigidBodyFrame<double>& frame_E, const Vector6<double>& V_WE,
    const DifferentialInverseKinematicsParameters& parameters);

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const VectorX<double>& q_current,
    const VectorX<double>& v_current, const RigidBodyFrame<double>& frame_E,
    const Isometry3<double>& X_WE_desired,
    const DifferentialInverseKinematicsParameters& parameters);

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const VectorX<double>& q_current,
    const VectorX<double>& v_current, const RigidBodyFrame<double>& frame_E,
    const Vector6<double>& V_WE,
    const DifferentialInverseKinematicsParameters& parameters);

class DifferentialInverseKinematics {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DifferentialInverseKinematics);

  DifferentialInverseKinematics(std::unique_ptr<RigidBodyTree<double>> robot,
                                const std::string& end_effector_frame_name);

  void set_unconstrained_degrees_of_freedom_velocity_limit(double limit) {
    parameters_.set_unconstrained_degrees_of_freedom_velocity_limit(limit);
  }

  const Vector6<double>& get_end_effector_velocity_gain() const {
    return parameters_.get_end_effector_velocity_gain();
  }

  void set_end_effector_velocity_gain(const Vector6<double>& gain_E) {
    parameters_.set_end_effector_velocity_gain(gain_E);
  }

  const optional<std::pair<VectorX<double>, VectorX<double>>>
  get_joint_position_limits() const {
    return parameters_.get_joint_position_limits();
  }

  void set_joint_position_limits(
      const std::pair<VectorX<double>, VectorX<double>>& q_bounds) {
    parameters_.set_joint_position_limits(q_bounds);
  }

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  get_joint_velocity_limits() const {
    return parameters_.get_joint_position_limits();
  }

  void set_joint_velocity_limits(
      const std::pair<VectorX<double>, VectorX<double>>& v_bounds) {
    parameters_.set_joint_velocity_limits(v_bounds);
  }

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  get_joint_acceleration_limits() const {
    return parameters_.get_joint_position_limits();
  }

  void set_joint_acceleration_limits(
      const std::pair<VectorX<double>, VectorX<double>>& vd_bounds) {
    parameters_.set_joint_acceleration_limits(vd_bounds);
  }

  void set_current_joint_position(const VectorX<double>& q_current) {
    DRAKE_ASSERT(q_current.size() == robot_->get_num_positions());
    q_current_ = q_current;
  }

  void set_current_joint_velocity(const VectorX<double>& v_current) {
    DRAKE_ASSERT(v_current.size() == robot_->get_num_velocities());
    v_current_ = v_current;
  }

  const Vector6<double>& get_desired_end_effector_velocity() {
    return V_WE_desired_;
  }

  void set_desired_end_effector_velocity(
      const Vector6<double>& desired_end_effector_velocity) {
    V_WE_desired_ = desired_end_effector_velocity;
  }

  double get_timestep() const { return parameters_.get_timestep(); }

  void set_timestep(double dt) { parameters_.set_timestep(dt); }

  DifferentialInverseKinematicsResult Solve() const {
    return DoDifferentialInverseKinematics(
        *robot_, q_current_, v_current_, *frame_E_, V_WE_desired_, parameters_);
  }

  DifferentialInverseKinematicsResult ComputeJointVelocities(
      const VectorX<double>& q, const VectorX<double>& v_last,
      const Vector6<double>& V_WE, double dt);

 private:
  copyable_unique_ptr<RigidBodyTree<double>> robot_{};
  std::shared_ptr<RigidBodyFrame<double>> frame_E_{};
  VectorX<double> q_current_;
  VectorX<double> v_current_;
  Vector6<double> V_WE_desired_;
  DifferentialInverseKinematicsParameters parameters_{};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
