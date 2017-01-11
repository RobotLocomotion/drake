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
   * @param q the 7-dimensional generalized configuration (see warning below)
   * @warning The first three values of generalized configuration are positional
   *          and the next four values are a quaternion orientation (ew, ex, ey,
   *          ez). The first three values of generalized velocity are angular
   *          velocity (ωx, ωy, ωz) and the second three values are linear
   *          velocity (dx/dt, dy/dt, dz/dt). The particular frames associated
   *          with generalized coordinates and generalized velocities are
   *          described further immediately below.
   * @param[out] qdot_to_v a nv × nq sized matrix, where nv is the dimension of
   *        generalized velocities (6) and nq is the dimension of generalized
   *        coordinates (7), that converts time derivatives of generalized
   *        coordinates to generalized velocities _for given configuration
   *        @p q_. The linear components of velocity will be rotated from some
   *        frame N to a frame B, while the time derivatives of the quaternion
   *        values (the quaternion values themselves are used to generate
   *        matrices that rotate vectors from Frame B to Frame N) will be
   *        transformed to angular velocities in Frame B.
   * @warning If the norm of the quaternion values used for orientation in @p q
   *          is equal to `s ≠ 1`, applying @p qdot_to_v to unscaled time
   *          derivatives of the quaternion values will yield an angular
   *          velocity vector scaled by `s` as well. This method neither
   *          performs a normalization check nor normalizes the quaternion
   *          orientation parameters, as implications for integration
   *          techniques must be carefully considered.
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
    const auto& ew = quat[0];
    const auto& ex = quat[1];
    const auto& ey = quat[2];
    const auto& ez = quat[3];

    // Assume that the quaternion orientation (e) gives a transformation matrix
    // that rotates vectors from some frame B to some frame N. The upper right
    // hand block of the transformation matrix represents
    // the 2 L part of the relationship ω = 2 L de/dt, where
    // e = [ ew ex ey ez ] are the quaternion values and ω is an angular
    // velocity given in frame B. The relationship ω = 2 L de/dt and the
    // matrix L was taken from:
    // - P. Nikravesh, Computer-Aided Analysis of Mechanical Systems. Prentice
    //     Hall, New Jersey, 1988. Equation 6.108.
    // NOTE: the torque-free, cylindrical solid unit test successfully detects
    //       when this matrix (incorrectly) is set to that which transforms
    //       unit quaternion time derivatives to angular velocities in the
    //       global frame.
    qdot_to_v.template block<3, 3>(0, 0).setZero();
    qdot_to_v.template block<3, 4>(0, 3) <<  -ex,  ew,  ez, -ey,
                                             -ey, -ez,  ew,  ex,
                                             -ez,  ey, -ex,  ew;
    qdot_to_v.template block<3, 4>(0, 3) *= 2.;

    // Given the arbitrary frames B and N described above, the next three
    // columns rotate linear velocities from frame N to frame B.
    auto R = drake::math::quat2rotmat(quat);
    qdot_to_v.template block<3, 3>(3, 0) = R.transpose();
    qdot_to_v.template block<3, 4>(3, 3).setZero();
  }

  /**
   * Computes a matrix that transforms a generalized velocity to the time
   * derivative of generalized configuration to generalized velocity _for given 
   * configuration @p q_.
   * @param q the 7-dimensional generalized configuration (see warning below).
   * @warning The first three values of generalized configuration are positional
   *          and the next four values are a quaternion orientation (ew, ex, ey,
   *          ez). The first three values of generalized velocity are angular
   *          velocity (ωx, ωy, ωz) and the second three values are linear
   *          velocity (dx/dt, dy/dt, dz/dt). The particular frames associated
   *          with generalized coordinates and generalized velocities are
   *          described further immediately below.
   * @param[out] v_to_qdot a nq × nv sized matrix, where nv is the dimension of
   *        generalized velocities (6) and nq is the dimension of generalized
   *        coordinates (7), that converts generalized velocities to time
   *        derivatives of generalized coordinates _for given configuration 
   *        @p q_. The linear components of velocity will be rotated
   *        from some frame B to some frame N (i.e., linear velocities will
   *        not change in length), while angular velocities in Frame
   *        B will be transformed to time derivatives of the quaternion values
   *        (the quaternion values themselves are used to generate matrices that
   *        rotate vectors from Frame B to Frame N).
   * @warning If the norm of the quaternion values used for orientation in @p q
   *          is equal to `s ≠ 1`, applying @p v_to_qdot to unscaled angular
   *          velocity vector will yeild time derivatives of the quaternion
   *          values scaled by `s` as well. This method neither
   *          performs a normalization check nor normalizes the quaternion
   *          orientation parameters, as implications for integration
   *          techniques must be carefully considered.
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

    if (dv_to_qdot) {
      throw std::runtime_error("no longer supported");
    }

    // Get the quaternion values.
    auto quat =
        q.template middleRows<drake::kQuaternionSize>(drake::kSpaceDimension);
    const auto& ew = quat[0];
    const auto& ex = quat[1];
    const auto& ey = quat[2];
    const auto& ez = quat[3];

    // Assume that the quaternion orientation (e) gives a transformation matrix
    // that rotates vectors from some frame B to some frame N. The upper right
    // hand block of the tranfsormation matrix corresponds to the transpose of
    // the "L" matrix used in qdot2v(). Specifically, this matrix represents
    // the 1/2 Lᵀ part of the relationship de/dt = 1/2 Lᵀω, where
    // e = [ ew ex ey ez ] are the quaternion values and ω is an angular
    // velocity given in frame B. The relationship de/dt = 1/2 Lᵀω and the
    // matrix L was taken from:
    // - P. Nikravesh, Computer-Aided Analysis of Mechanical Systems. Prentice
    //     Hall, New Jersey, 1988. Equation 6.109.
    v_to_qdot.template block<4, 3>(0, 0).setZero();
    v_to_qdot.template block<4, 3>(3, 0) <<  -ex, -ey, -ez,
                                              ew, -ez,  ey,
                                              ez,  ew, -ex,
                                             -ey,  ex,  ew;
    v_to_qdot.template block<4, 3>(3, 0) *= 0.5;

    // Given the arbitrary frames B and N described above, the next three
    // columns rotate linear velocities from frame B to frame N.
    v_to_qdot.template block<3, 3>(0, 3) = drake::math::quat2rotmat(quat);
    v_to_qdot.template block<4, 3>(3, 3).setZero();
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
