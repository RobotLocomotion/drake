/*
 * FixedJoint.h
 *
 *  Created on: Mar 26, 2015
 *      Author: twan
 */

#ifndef DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_
#define DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_

#include "DrakeJointImpl.h"

class DRAKEJOINTS_EXPORT FixedJoint : public DrakeJointImpl<FixedJoint> {
 public:
  FixedJoint(const std::string &name,
             const Eigen::Isometry3d &transform_to_parent_body)
      : DrakeJointImpl(*this, name, transform_to_parent_body, 0, 0){};

  virtual ~FixedJoint(){};

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>
  jointTransform(const Eigen::MatrixBase<DerivedQ> &q) const {
    return Eigen::Transform<typename DerivedQ::Scalar, 3,
                            Eigen::Isometry>::Identity();
  };

  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(const Eigen::MatrixBase<DerivedQ> &q,
                      Eigen::MatrixBase<DerivedMS> &motion_subspace,
                      typename Gradient<DerivedMS, Eigen::Dynamic>::type *
                          dmotion_subspace = nullptr) const {
    motion_subspace.resize(TWIST_SIZE, getNumVelocities());
    if (dmotion_subspace) {
      dmotion_subspace->resize(motion_subspace.size(), getNumPositions());
    }
  };

  template <typename DerivedQ, typename DerivedV>
  void motionSubspaceDotTimesV(
      const Eigen::MatrixBase<DerivedQ> &q,
      const Eigen::MatrixBase<DerivedV> &v,
      Eigen::Matrix<typename DerivedQ::Scalar, 6, 1> &
          motion_subspace_dot_times_v,
      typename Gradient<Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>,
                        Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdq =
          nullptr,
      typename Gradient<Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>,
                        Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdv =
          nullptr) const {
    motion_subspace_dot_times_v.setZero();

    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(TWIST_SIZE, 1);
    }

    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(TWIST_SIZE, 1);
    }
  };

  template <typename DerivedQ>
  void qdot2v(
      const Eigen::MatrixBase<DerivedQ> &q,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic,
                    0, MAX_NUM_VELOCITIES, MAX_NUM_POSITIONS> &qdot_to_v,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> *
          dqdot_to_v) const {
    qdot_to_v.resize(getNumVelocities(), getNumPositions());
    if (dqdot_to_v) {
      dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
    }
  };

  template <typename DerivedQ>
  void v2qdot(
      const Eigen::MatrixBase<DerivedQ> &q,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic,
                    0, MAX_NUM_POSITIONS, MAX_NUM_VELOCITIES> &v_to_qdot,
      Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> *
          dv_to_qdot) const {
    v_to_qdot.resize(getNumPositions(), getNumVelocities());
    if (dv_to_qdot) {
      dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
    }
  };

  template <typename DerivedV>
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorque(
      const Eigen::MatrixBase<DerivedV> &v) const {
    return Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1>(
        getNumVelocities(), 1);
  }

  virtual std::string getPositionName(int index) const override;
  virtual Eigen::VectorXd zeroConfiguration() const override;
  virtual Eigen::VectorXd randomConfiguration(
      std::default_random_engine &generator) const override;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_ */
