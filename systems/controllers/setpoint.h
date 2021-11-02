#pragma once

#include <iostream>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace systems {
namespace controllers {

/**
 * This is used to compute target spatial acceleration, which is the input
 * to the inverse dynamics controller.
 * The target acceleration is computed by:
 * acceleration_d = Kp*(x* - x) + Kd*(xd* - xd) + xdd*,
 * where x is pose, xd is velocity, and xdd is acceleration.
 * Variables with superscript * are the set points, and Kp and Kd are the
 * position and velocity gains.
 *
 * Pose "difference" is computed as:
 * H^w_d = E * H^w_m, E = H^w_d * H^w_m.inverse(), where
 * H^w_d = desired orientation in the world frame,
 * H^w_m = measured orientation in the world frame,
 * E = a small rotation in the world frame from measured to desired.
 *
 * The first terms 3 are angular accelerations, and the last 3 are linear
 * accelerations.
 */
template <typename Scalar>
class DRAKE_DEPRECATED("2022-02-01", "This (unused) class is being removed.")
CartesianSetpoint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CartesianSetpoint)

  CartesianSetpoint() {
    pose_d_.SetIdentity();
    vel_d_.setZero();
    acc_d_.setZero();
    Kp_.setZero();
    Kd_.setZero();
  }

  /**
   * @param pose_d Desired pose
   * @param vel_d Desired velocity
   * @param acc_d Desired feedforward acceleration
   * @param Kp Position gain
   * @param Kd Velocity gain
   */
  CartesianSetpoint(const Isometry3<Scalar>& pose_d,
                    const Vector6<Scalar>& vel_d, const Vector6<Scalar>& acc_d,
                    const Vector6<Scalar>& Kp, const Vector6<Scalar>& Kd) {
    pose_d_ = math::RigidTransform<Scalar>(pose_d);
    vel_d_ = vel_d;
    acc_d_ = acc_d;
    Kp_ = Kp;
    Kd_ = Kd;
  }

  /**
   * Computes target acceleration using PD feedback + feedfoward acceleration.
   * @param pose Measured pose
   * @param vel Measured velocity
   * @return Computed spatial acceleration.
   */
  Vector6<Scalar> ComputeTargetAcceleration(const Isometry3<Scalar>& pose,
                                            const Vector6<Scalar>& vel) const {
    // feedforward acc + velocity feedback
    Vector6<Scalar> acc = acc_d_;
    acc += (Kd_.array() * (vel_d_ - vel).array()).matrix();

    // pose feedback
    // H^w_d = desired orientation in the world frame,
    // H^w_m = measured orientation in the world frame,
    // E = a small rotation in the world frame from measured to desired.
    // H^w_d = E * H^w_m, E = H^w_d * H^w_m.inverse()
    Quaternion<Scalar> quat_d = pose_d_.rotation().ToQuaternion();
    Quaternion<Scalar> quat(pose.linear());
    // Make sure the relative rotation between the desired and the measured
    // rotation goes the "shortest" way.
    if (quat_d.dot(quat) < 0) {
      quat.w() *= -1;
      quat.x() *= -1;
      quat.y() *= -1;
      quat.z() *= -1;
    }
    AngleAxis<Scalar> angle_axis_err(quat_d * quat.inverse().normalized());

    Vector3<Scalar> pos_err = pose_d_.translation() - pose.translation();
    Vector3<Scalar> rot_err = angle_axis_err.axis() * angle_axis_err.angle();

    // orientation
    acc.template head<3>() +=
        (Kp_.template head<3>().array() * rot_err.array()).matrix();

    // position
    acc.template tail<3>() +=
        (Kp_.template tail<3>().array() * pos_err.array()).matrix();

    return acc;
  }

  bool is_valid() const {
    bool ret = pose_d_.translation().allFinite();
    ret &= Matrix3<Scalar>::Identity().isApprox(
        (pose_d_.rotation() * pose_d_.rotation().transpose()).matrix(),
        Eigen::NumTraits<Scalar>::epsilon());
    ret &= vel_d_.allFinite();
    ret &= acc_d_.allFinite();
    ret &= Kp_.allFinite();
    ret &= Kd_.allFinite();
    return ret;
  }

  // Getters
  const math::RigidTransform<Scalar>& desired_pose() const {
    return pose_d_;
  }
  const Vector6<Scalar>& desired_velocity() const { return vel_d_; }
  const Vector6<Scalar>& desired_acceleration() const { return acc_d_; }
  const Vector6<Scalar>& Kp() const { return Kp_; }
  const Vector6<Scalar>& Kd() const { return Kd_; }

  // Setters
  math::RigidTransform<Scalar>& mutable_desired_pose() {
    return pose_d_;
  }
  Vector6<Scalar>& mutable_desired_velocity() { return vel_d_; }
  Vector6<Scalar>& mutable_desired_acceleration() { return acc_d_; }
  Vector6<Scalar>& mutable_Kp() { return Kp_; }
  Vector6<Scalar>& mutable_Kd() { return Kd_; }

 private:
  // Desired pose
  math::RigidTransform<Scalar> pose_d_;
  // Desired velocity
  Vector6<Scalar> vel_d_;
  // Desired acceleration
  Vector6<Scalar> acc_d_;

  // Position gains
  Vector6<Scalar> Kp_;
  // Velocity gains
  Vector6<Scalar> Kd_;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template <typename Scalar>
std::ostream& operator<<(std::ostream& out,
                         const CartesianSetpoint<Scalar>& setpoint) {
#pragma GCC diagnostic pop
  const math::RigidTransform<Scalar> X(setpoint.desired_pose());
  const math::RollPitchYaw<Scalar> rpy(X.rotation());
  out << "pose: (" << X.translation().transpose()
      << "), (" << rpy.vector().transpose() << ")"
      << "\n";
  out << "velocity: " << setpoint.desired_velocity().transpose() << "\n";
  out << "acceleration: " << setpoint.desired_acceleration().transpose()
      << "\n";
  out << "Kp: " << setpoint.Kp().transpose() << "\n";
  out << "Kd: " << setpoint.Kd().transpose() << std::endl;
  return out;
}

template <typename Scalar>
class DRAKE_DEPRECATED("2022-02-01", "This (unused) class is being removed.")
VectorSetpoint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VectorSetpoint)

  VectorSetpoint() {}

  explicit VectorSetpoint(int dim) {
    pos_d_ = VectorX<Scalar>::Zero(dim);
    vel_d_ = VectorX<Scalar>::Zero(dim);
    acc_d_ = VectorX<Scalar>::Zero(dim);
    Kp_ = VectorX<Scalar>::Zero(dim);
    Kd_ = VectorX<Scalar>::Zero(dim);
  }

  /**
   * @param pos_d Desired position
   * @param vel_d Desired velocity
   * @param acc_d Desired feedforward acceleration
   * @param Kp Position gain
   * @param Kd Velocity gain
   */
  VectorSetpoint(const VectorX<Scalar>& pos_d, const VectorX<Scalar>& vel_d,
                 const VectorX<Scalar>& acc_d, const VectorX<Scalar>& Kp,
                 const VectorX<Scalar>& Kd) {
    if (pos_d.size() != vel_d.size() || pos_d.size() != acc_d.size() ||
        pos_d.size() != Kp.size() || pos_d.size() != Kd.size()) {
      throw std::runtime_error(
          "Setpoints and gains have different dimensions.");
    }
    pos_d_ = pos_d;
    vel_d_ = vel_d;
    acc_d_ = acc_d;
    Kp_ = Kp;
    Kd_ = Kd;
  }

  /**
   * Computes target acceleration using PD feedback + feedforward acceleration
   * @param idx Index
   * @param pos Measured position
   * @param vel Measured velocity
   * @return Computed acceleration
   */
  Scalar ComputeTargetAcceleration(int idx, Scalar pos, Scalar vel) const {
    if (idx < 0 || idx >= pos_d_.size())
      throw std::runtime_error("Index out of bound.");
    Scalar acc = acc_d_[idx];
    acc += Kp_[idx] * (pos_d_[idx] - pos);
    acc += Kd_[idx] * (vel_d_[idx] - vel);
    return acc;
  }

  /**
   * Computes target acceleration using PD feedback + feedforward acceleration
   * @param idx Index
   * @param pos Measured position
   * @param vel Measured velocity
   * @return Computed acceleration
   */
  VectorX<Scalar> ComputeTargetAcceleration(const VectorX<Scalar>& pos,
                                            const VectorX<Scalar>& vel) const {
    if (pos.size() != vel.size() || pos.size() != pos_d_.size()) {
      throw std::runtime_error(
          "Setpoints and states have different dimensions.");
    }
    VectorX<Scalar> acc(pos_d_.size());
    for (int i = 0; i < size(); ++i)
      acc[i] = ComputeTargetAcceleration(i, pos[i], vel[i]);
    return acc;
  }

  bool is_valid(int dim) const {
    bool ret = pos_d_.size() == vel_d_.size() &&
               pos_d_.size() == acc_d_.size() && pos_d_.size() == Kp_.size() &&
               pos_d_.size() == Kd_.size() && pos_d_.size() == dim;
    ret &= pos_d_.allFinite();
    ret &= vel_d_.allFinite();
    ret &= acc_d_.allFinite();
    ret &= Kp_.allFinite();
    ret &= Kd_.allFinite();
    return ret;
  }

  // Getters
  const VectorX<Scalar>& desired_position() const { return pos_d_; }
  const VectorX<Scalar>& desired_velocity() const { return vel_d_; }
  const VectorX<Scalar>& desired_acceleration() const { return acc_d_; }
  const VectorX<Scalar>& Kp() const { return Kp_; }
  const VectorX<Scalar>& Kd() const { return Kd_; }
  int size() const { return pos_d_.size(); }

  // Setters
  VectorX<Scalar>& mutable_desired_position() { return pos_d_; }
  VectorX<Scalar>& mutable_desired_velocity() { return vel_d_; }
  VectorX<Scalar>& mutable_desired_acceleration() { return acc_d_; }
  VectorX<Scalar>& mutable_Kp() { return Kp_; }
  VectorX<Scalar>& mutable_Kd() { return Kd_; }

 private:
  // Desired position
  VectorX<Scalar> pos_d_;
  // Desired velocity
  VectorX<Scalar> vel_d_;
  // Desired acceleration
  VectorX<Scalar> acc_d_;

  // Position gains
  VectorX<Scalar> Kp_;
  // Velocity gains
  VectorX<Scalar> Kd_;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template <typename Scalar>
std::ostream& operator<<(std::ostream& out,
                         const VectorSetpoint<Scalar>& setpoint) {
#pragma GCC diagnostic pop
  out << "pos: " << setpoint.desired_position().transpose() << "\n";
  out << "vel: " << setpoint.desired_velocity().transpose() << "\n";
  out << "acc: " << setpoint.desired_acceleration().transpose() << "\n";
  out << "Kp: " << setpoint.Kp().transpose() << "\n";
  out << "Kd: " << setpoint.Kd().transpose() << std::endl;
  return out;
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake
