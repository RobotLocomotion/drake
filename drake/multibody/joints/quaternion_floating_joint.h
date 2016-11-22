#pragma once

#include <string>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/math/normalize_vector.h"
#include "drake/math/rotation_conversion_gradient.h"
#include "drake/multibody/joints/drake_joint_impl.h"
#include "drake/util/drakeGeometryUtil.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
class QuaternionFloatingJoint : public DrakeJointImpl<QuaternionFloatingJoint> {
 public:
  QuaternionFloatingJoint(const std::string& name,
                          const Eigen::Isometry3d& transform_to_parent_body)
      : DrakeJointImpl(*this, name, transform_to_parent_body, 7, 6) {}

  virtual ~QuaternionFloatingJoint() {}

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry> ret;
    ret.linear() = drake::math::quat2rotmat(q.template bottomRows<4>());
    ret.translation() << q[0], q[1], q[2];
    ret.makeAffine();
    return ret;
  }

  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(
      const Eigen::MatrixBase<DerivedQ>& q,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::MatrixBase<DerivedMS>& motion_subspace,
      typename drake::math::Gradient<DerivedMS, Eigen::Dynamic>::type*
          dmotion_subspace = nullptr) const {
    motion_subspace.setIdentity(drake::kTwistSize, get_num_velocities());
    if (dmotion_subspace) {
      dmotion_subspace->setZero(motion_subspace.size(), get_num_positions());
    }
  }

  template <typename DerivedQ, typename DerivedV>
  void motionSubspaceDotTimesV(
      const Eigen::MatrixBase<DerivedQ>& q,
      const Eigen::MatrixBase<DerivedV>& v,
      Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>&
          motion_subspace_dot_times_v,
      typename drake::math::Gradient<
          Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type*
          dmotion_subspace_dot_times_vdq = nullptr,
      typename drake::math::Gradient<
          Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type*
          dmotion_subspace_dot_times_vdv = nullptr) const {
    motion_subspace_dot_times_v.setZero();
    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(
          motion_subspace_dot_times_v.size(), get_num_positions());
    }
    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(
          motion_subspace_dot_times_v.size(), get_num_velocities());
    }
  }

  /**
   * Computes a matrix that transforms the time derivative of generalized 
   * configuration to generalized velocity _for given configuration @p q_.
   * @param q the generalized configuration 
   * @param qdot_to_v a nv × nq sized matrix, where nv is the dimension of
   *        generalized velocities and nq is the dimension of generalized
   *        coordinates, that converts time derivatives of generalized
   *        coordinates to generalized velocities _for given configuration 
   *        @p q_.
   */
  template <typename DerivedQ>
  void qdot2v(const Eigen::MatrixBase<DerivedQ>& q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_VELOCITIES,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                            DrakeJoint::MAX_NUM_POSITIONS>& qdot_to_v,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic>* dqdot_to_v) const {
    if (dqdot_to_v) {
      throw std::runtime_error("no longer supported");
    }

    qdot_to_v.resize(get_num_velocities(), get_num_positions());

    // Get the quaternion values.
    auto quat =
        q.template middleRows<drake::kQuaternionSize>(drake::kSpaceDimension);
    const auto qw = quat[0];
    const auto qx = quat[1];
    const auto qy = quat[2];
    const auto qz = quat[3];

    // First three rows correspond to rigid body translation. Transformation
    // from time derivative of generalized position to generalized velocity
    // is just the identity matrix.
    qdot_to_v.block(0, 0, 3, 3).setIdentity();
    qdot_to_v.block(0, 3, 3, 4).setZero();

    // The next four rows correspond to the "G" matrix in the equation:
    // 2 G de/dt = ω, where e = [ qw qx qy qz ] are the values of the
    // unit quaternion and ω is the angular velocity vector defined in the
    // world frame. This equation was taken from:
    // - P. Nikravesh, Computer-Aided Analysis of Mechanical Systems. Prentice
    //     Hall, New Jersey, 1988. Equation 105.
    qdot_to_v.block(3, 0, 3, 3).setZero();
    qdot_to_v.block(3, 3, 3, 4) <<  -qx,  qw, -qz,  qy,
                                    -qy,  qz,  qw, -qx,
                                    -qz, -qy,  qx,  qw;
    qdot_to_v.block(3, 3, 3, 4) *= 2.;
  }

  /**
   * Computes a matrix that transforms the generalized velocity to the time 
   * derivative of generalized configuration to generalized velocity _for given 
   * configuration @p q_.
   * @param q the generalized configuration 
   * @param v_to_qdoc a nq × nv sized matrix, where nv is the dimension of
   *        generalized velocities and nq is the dimension of generalized
   *        coordinates, that converts generalized velocities to time 
   *        derivatives of generalized coordinates _for given configuration 
   *        @p q_.
   */
  template <typename DerivedQ>
  void v2qdot(const Eigen::MatrixBase<DerivedQ>& q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_POSITIONS,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                            DrakeJoint::MAX_NUM_VELOCITIES>& v_to_qdot,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic>* dv_to_qdot) const {
    v_to_qdot.resize(get_num_positions(), get_num_velocities());

    // First three columns correspond to rigid body translation. Transformation
    // from generalized velocity to time derivative of generalized coordinates
    // is just the identity matrix.
    v_to_qdot.block(0, 0, 3, 3).setIdentity();
    v_to_qdot.block(3, 0, 3, 3).setZero();

    // Get the quaternion values.
    auto quat =
        q.template middleRows<drake::kQuaternionSize>(drake::kSpaceDimension);
    const auto qw = quat[0];
    const auto qx = quat[1];
    const auto qy = quat[2];
    const auto qz = quat[3];

    // The next four columns correspond to the transpose of the "G" matrix
    // used in qdot2v(). Specifically, this matrix serves the function:
    // de/dt = 1/2 G' ω, where e = [ qw qx qy qz ] are the values of the
    // unit quaternion and ω is the angular velocity vector defined in the
    // world frame. This matrix was taken from:
    // - P. Nikravesh, Computer-Aided Analysis of Mechanical Systems. Prentice
    //     Hall, New Jersey, 1988. Equation 106.
    v_to_qdot.block(0, 3, 4, 3).setZero();
    v_to_qdot.block(3, 3, 4, 3) <<  -qx, -qy, -qz,
                                     qw,  qz, -qy,
                                    -qz,  qw,  qx,
                                     qy, -qx,  qw;
    v_to_qdot.block(3, 3, 4, 3) *= 0.5;
  }

  template <typename DerivedV>
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorque(
      const Eigen::MatrixBase<DerivedV>& v) const {
    return Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1>::Zero(
        get_num_velocities(), 1);
  }

  bool is_floating() const override { return true; };

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use is_floating().")
#endif
  bool isFloating() const override { return is_floating(); }

  std::string get_position_name(int index) const override;
  std::string get_velocity_name(int index) const override;
  Eigen::VectorXd zeroConfiguration() const override;
  Eigen::VectorXd randomConfiguration(
      std::default_random_engine& generator) const override;

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_position_name().")
#endif
  std::string getPositionName(int index) const override;

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_velocity_name().")
#endif
  std::string getVelocityName(int index) const override;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual
