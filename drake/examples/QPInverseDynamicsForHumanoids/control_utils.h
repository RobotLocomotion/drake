#pragma once

#include <iostream>
#include <Eigen/Dense>

#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This is used to compute target task space acceleration, which is the input
 * to the inverse dynamics controller.
 * The target acceleration is computed by:
 * acceleration_d = Kp*(x* - x) + Kd*(xd* - xd) + xdd*,
 * where x is pose, xd is velocity, and xdd is acceleration.
 * Variables with superscript * are the set points, and Kp and Kd are the
 * position and velocity gains.
 * The first terms 3 are angular accelerations, and the last 3 are linear
 * accelerations.
 */
template <typename Scalar>
class CartesianSetpoint {
 public:
  CartesianSetpoint() {
    pose_d_.setIdentity();
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
  CartesianSetpoint(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& pose_d,
                    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vel_d,
                    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& acc_d,
                    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Kp,
                    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Kd) {
    pose_d_ = pose_d;
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
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ComputeTargetAcceleration(
      const Eigen::Transform<Scalar, 3, Eigen::Isometry>& pose,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vel) const {
    // feedforward acc + velocity feedback
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> acc = acc_d_;
    acc += (Kd_.array() * (vel_d_ - vel).array()).matrix();

    // pose feedback
    // H^w_d = desired orientation in the world frame,
    // H^w_m = measured orientation in the world frame,
    // E = a small rotation in the world frame from measured to desired.
    // H^w_d = E * H^w_m, E = H^w_d * H^w_m.transpose()
    Eigen::Matrix<Scalar, 3, 3> R_err =
        pose_d_.linear() * pose.linear().transpose();
    Eigen::AngleAxis<Scalar> angle_axis_err(R_err);

    Eigen::Matrix<Scalar, 3, 1> pos_err =
        pose_d_.translation() - pose.translation();
    Eigen::Matrix<Scalar, 3, 1> rot_err =
        angle_axis_err.axis() * angle_axis_err.angle();

    // orientation
    acc.segment(0, 3) += (Kp_.segment(0, 3).array() * rot_err.array()).matrix();

    // position
    acc.segment(3, 3) += (Kp_.segment(3, 3).array() * pos_err.array()).matrix();

    return acc;
  }

  bool is_valid() const {
    bool ret = pose_d_.translation().allFinite();
    ret &= Eigen::Matrix<Scalar, 3, 3>::Identity().isApprox(
        pose_d_.linear() * pose_d_.linear().transpose(),
        Eigen::NumTraits<Scalar>::epsilon());
    ret &= vel_d_.allFinite();
    ret &= acc_d_.allFinite();
    ret &= Kp_.allFinite();
    ret &= Kd_.allFinite();
    return ret;
  }

  // Getters
  inline const Eigen::Transform<Scalar, 3, Eigen::Isometry>& desired_pose()
      const {
    return pose_d_;
  }
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& desired_velocity()
      const {
    return vel_d_;
  }
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& desired_acceleration()
      const {
    return acc_d_;
  }
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Kp() const {
    return Kp_;
  }
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Kd() const {
    return Kd_;
  }

  // Setters
  inline Eigen::Transform<Scalar, 3, Eigen::Isometry>& mutable_desired_pose() {
    return pose_d_;
  }
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mutable_desired_velocity() {
    return vel_d_;
  }
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>&
  mutable_desired_acceleration() {
    return acc_d_;
  }
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mutable_Kp() { return Kp_; }
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mutable_Kd() { return Kd_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Desired pose
  Eigen::Transform<Scalar, 3, Eigen::Isometry> pose_d_;
  // Desired velocity
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> vel_d_;
  // Desired acceleration
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> acc_d_;

  // Position gains
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Kp_;
  // Velocity gains
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Kd_;
};

template <typename Scalar>
inline std::ostream& operator<<(std::ostream& out,
                                const CartesianSetpoint<Scalar>& setpoint) {
  Eigen::Matrix<Scalar, 3, 1> rpy =
      math::rotmat2rpy(setpoint.desired_pose().linear());
  out << "pose: (" << setpoint.desired_pose().translation().transpose()
      << "), (" << rpy.transpose() << ")" << std::endl;
  out << "velocity: " << setpoint.desired_velocity().transpose() << std::endl;
  out << "acceleration: " << setpoint.desired_acceleration().transpose()
      << std::endl;
  out << "Kp: " << setpoint.Kp().transpose() << std::endl;
  out << "Kd: " << setpoint.Kd().transpose() << std::endl;
  return out;
}

template <typename Scalar>
class VectorSetpoint {
 public:
  VectorSetpoint() {}

  explicit VectorSetpoint(int dim) {
    pos_d_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(dim);
    vel_d_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(dim);
    acc_d_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(dim);
    Kp_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(dim);
    Kd_ = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(dim);
  }

  /**
   * @param pos_d Desired position
   * @param vel_d Desired velocity
   * @param acc_d Desired feedforward acceleration
   * @param Kp Position gain
   * @param Kd Velocity gain
   */
  VectorSetpoint(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& pos_d,
                 const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vel_d,
                 const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& acc_d,
                 const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Kp,
                 const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Kd) {
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
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ComputeTargetAcceleration(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& pos,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vel) const {
    if (pos.size() != vel.size() || pos.size() != pos_d_.size()) {
      throw std::runtime_error(
          "Setpoints and states have different dimensions.");
    }
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> acc(pos_d_.size());
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
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& desired_position()
      const {
    return pos_d_;
  }
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& desired_velocity()
      const {
    return vel_d_;
  }
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& desired_acceleration()
      const {
    return acc_d_;
  }
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Kp() const {
    return Kp_;
  }
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Kd() const {
    return Kd_;
  }
  inline int size() const { return pos_d_.size(); }

  // Setters
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mutable_desired_position() {
    return pos_d_;
  }
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mutable_desired_velocity() {
    return vel_d_;
  }
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>&
  mutable_desired_acceleration() {
    return acc_d_;
  }
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mutable_Kp() { return Kp_; }
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mutable_Kd() { return Kd_; }

 private:
  // Desired position
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> pos_d_;
  // Desired velocity
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> vel_d_;
  // Desired acceleration
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> acc_d_;

  // Position gains
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Kp_;
  // Velocity gains
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Kd_;
};

template <typename Scalar>
std::ostream& operator<<(std::ostream& out,
                         const VectorSetpoint<Scalar>& setpoint) {
  out << "pos: " << setpoint.desired_position().transpose() << std::endl;
  out << "vel: " << setpoint.desired_velocity().transpose() << std::endl;
  out << "acc: " << setpoint.desired_acceleration().transpose() << std::endl;
  out << "Kp: " << setpoint.Kp().transpose() << std::endl;
  out << "Kd: " << setpoint.Kd().transpose() << std::endl;
  return out;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
