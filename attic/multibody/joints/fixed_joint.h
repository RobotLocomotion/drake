#pragma once

#include <memory>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/multibody/joints/drake_joint_impl.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
class FixedJoint : public DrakeJointImpl<FixedJoint> {
 public:
  FixedJoint(const std::string& name,
             const Eigen::Isometry3d& transform_to_parent_body)
      : DrakeJointImpl(*this, name, transform_to_parent_body, 0, 0) {}

  virtual ~FixedJoint() {}

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ>& q) const {
    drake::unused(q);
    return Eigen::Transform<typename DerivedQ::Scalar, 3,
                            Eigen::Isometry>::Identity();
  }

  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(
      const Eigen::MatrixBase<DerivedQ>& q,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::MatrixBase<DerivedMS>& motion_subspace,
      typename drake::math::Gradient<DerivedMS, Eigen::Dynamic>::type*
          dmotion_subspace = nullptr) const {
    drake::unused(q);
    motion_subspace.resize(drake::kTwistSize, get_num_velocities());
    if (dmotion_subspace) {
      dmotion_subspace->resize(motion_subspace.size(), get_num_positions());
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
    drake::unused(q, v);

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
                    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                    0, MAX_NUM_VELOCITIES, MAX_NUM_POSITIONS>& qdot_to_v,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic>*
          dqdot_to_v) const {
    drake::unused(q);
    qdot_to_v.resize(get_num_velocities(), get_num_positions());
    if (dqdot_to_v) {
      dqdot_to_v->setZero(qdot_to_v.size(), get_num_positions());
    }
  }

  template <typename DerivedQ>
  void v2qdot(
      const Eigen::MatrixBase<DerivedQ>& q,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic,
                    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                    0, MAX_NUM_POSITIONS, MAX_NUM_VELOCITIES>& v_to_qdot,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic>*
          dv_to_qdot) const {
    drake::unused(q);
    v_to_qdot.resize(get_num_positions(), get_num_velocities());
    if (dv_to_qdot) {
      dv_to_qdot->setZero(v_to_qdot.size(), get_num_positions());
    }
  }

  template <typename DerivedV>
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorque(
      const Eigen::MatrixBase<DerivedV>& v) const {
    drake::unused(v);
    return Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1>(
        get_num_velocities(), 1);
  }

  template <typename DerivedQ>
  Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, 1> SpringTorque(
      const Eigen::MatrixBase<DerivedQ>& q) const {
    drake::unused(q);
    // Fixed joints have zero degrees of freedom.
    return drake::VectorX<typename DerivedQ::Scalar>::Zero(0, 1);
  }

  std::string get_position_name(int index) const override;
  Eigen::VectorXd zeroConfiguration() const override;
  Eigen::VectorXd randomConfiguration(
      std::default_random_engine& generator) const override;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  std::unique_ptr<DrakeJoint> DoClone() const final;
  void DoInitializeClone(DrakeJoint*) const final {}
};
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual
