#pragma once

#include "DrakeJointImpl.h"

#include "drake/common/eigen_types.h"

class DRAKEJOINTS_EXPORT FixedJoint : public DrakeJointImpl<FixedJoint> {
 public:
  FixedJoint(const std::string& name,
             const Eigen::Isometry3d& transform_to_parent_body)
      : DrakeJointImpl(*this, name, transform_to_parent_body, 0, 0) {}

  virtual ~FixedJoint() {}

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    return Eigen::Transform<typename DerivedQ::Scalar, 3,
                            Eigen::Isometry>::Identity();
  }

  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(
      const Eigen::MatrixBase<DerivedQ>& q,
      Eigen::MatrixBase<DerivedMS>& motion_subspace,
      typename drake::math::Gradient<DerivedMS, Eigen::Dynamic>::type*
          dmotion_subspace = nullptr) const {
    motion_subspace.resize(drake::kTwistSize, getNumVelocities());
    if (dmotion_subspace) {
      dmotion_subspace->resize(motion_subspace.size(), getNumPositions());
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
      dmotion_subspace_dot_times_vdq->setZero(drake::kTwistSize, 1);
    }

    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(drake::kTwistSize, 1);
    }
  }

  template <typename DerivedQ>
  void qdot2v(
      const Eigen::MatrixBase<DerivedQ>& q,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic,
                    0, MAX_NUM_VELOCITIES, MAX_NUM_POSITIONS>& qdot_to_v,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic>*
          dqdot_to_v) const {
    qdot_to_v.resize(getNumVelocities(), getNumPositions());
    if (dqdot_to_v) {
      dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
    }
  }

  template <typename DerivedQ>
  void v2qdot(
      const Eigen::MatrixBase<DerivedQ>& q,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic,
                    0, MAX_NUM_POSITIONS, MAX_NUM_VELOCITIES>& v_to_qdot,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic>*
          dv_to_qdot) const {
    v_to_qdot.resize(getNumPositions(), getNumVelocities());
    if (dv_to_qdot) {
      dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
    }
  }

  template <typename DerivedV>
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorque(
      const Eigen::MatrixBase<DerivedV>& v) const {
    return Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1>(
        getNumVelocities(), 1);
  }

  std::string getPositionName(int index) const override;
  Eigen::VectorXd zeroConfiguration() const override;
  Eigen::VectorXd randomConfiguration(
      std::default_random_engine& generator) const override;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
