#pragma once

#include <string>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/math/normalize_vector.h"
#include "drake/math/rotation_conversion_gradient.h"
#include "drake/systems/plants/joints/DrakeJointImpl.h"
#include "drake/util/drakeGeometryUtil.h"

class DRAKE_EXPORT QuaternionFloatingJoint
    : public DrakeJointImpl<QuaternionFloatingJoint> {
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
    typedef typename DerivedQ::Scalar Scalar;
    auto quat =
        q.template middleRows<drake::kQuaternionSize>(drake::kSpaceDimension);
    auto R = drake::math::quat2rotmat(quat);

    Eigen::Matrix<Scalar, 4, 1> quattilde;
    typename drake::math::Gradient<Eigen::Matrix<Scalar, 4, 1>,
                                   drake::kQuaternionSize,
                                   1>::type dquattildedquat;
    drake::math::NormalizeVector(quat, quattilde, &dquattildedquat);
    auto RTransposeM = (R.transpose() * quatdot2angularvelMatrix(quat)).eval();
    qdot_to_v.template block<3, 3>(0, 0).setZero();
    qdot_to_v.template block<3, 4>(0, 3).noalias() =
        RTransposeM * dquattildedquat;
    qdot_to_v.template block<3, 3>(3, 0) = R.transpose();
    qdot_to_v.template block<3, 4>(3, 3).setZero();
  }

  template <typename DerivedQ>
  void v2qdot(const Eigen::MatrixBase<DerivedQ>& q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_POSITIONS,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                            DrakeJoint::MAX_NUM_VELOCITIES>& v_to_qdot,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic>* dv_to_qdot) const {
    typedef typename DerivedQ::Scalar Scalar;
    v_to_qdot.resize(get_num_positions(), get_num_velocities());

    auto quat =
        q.template middleRows<drake::kQuaternionSize>(drake::kSpaceDimension);
    auto R = drake::math::quat2rotmat(quat);

    Eigen::Matrix<Scalar, drake::kQuaternionSize, drake::kSpaceDimension> M;
    if (dv_to_qdot) {
      auto dR = drake::math::dquat2rotmat(quat);
      typename drake::math::Gradient<decltype(M), drake::kQuaternionSize,
                                     1>::type dM;
      angularvel2quatdotMatrix(quat, M, &dM);

      dv_to_qdot->setZero(v_to_qdot.size(), get_num_positions());

      setSubMatrixGradient<4>(*dv_to_qdot, dR, intRange<3>(0), intRange<3>(3),
                              v_to_qdot.rows(), 3);
      auto dMR = matGradMultMat(M, R, dM, dR);
      setSubMatrixGradient<4>(*dv_to_qdot, dMR, intRange<4>(3), intRange<3>(0),
                              v_to_qdot.rows(), 3);
    } else {
      angularvel2quatdotMatrix(
          quat, M,
          (typename drake::math::Gradient<decltype(M), drake::kQuaternionSize,
                                          1>::type*)nullptr);
    }

    v_to_qdot.template block<3, 3>(0, 0).setZero();
    v_to_qdot.template block<3, 3>(0, 3) = R;
    v_to_qdot.template block<4, 3>(3, 0).noalias() = M * R;
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
