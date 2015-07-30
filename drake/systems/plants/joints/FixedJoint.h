/*
 * FixedJoint.h
 *
 *  Created on: Mar 26, 2015
 *      Author: twan
 */

#ifndef DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_
#define DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_

#include "DrakeJointImpl.h"

class DLLEXPORT_DRAKEJOINT FixedJoint: public DrakeJointImpl<FixedJoint> {
public:
  FixedJoint(const std::string &name, const Eigen::Isometry3d &transform_to_parent_body)
          : DrakeJointImpl(*this, name, transform_to_parent_body, 0, 0) { };

  virtual ~FixedJoint() { };

  template<typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::MatrixBase<DerivedQ> &q) const {
    return Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry>::Identity();
  };

  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(const Eigen::MatrixBase<DerivedQ> & q,
                      Eigen::MatrixBase<DerivedMS>& motion_subspace,
                      typename Gradient<DerivedMS, Eigen::Dynamic>::type* dmotion_subspace = nullptr) const {
    motion_subspace.resize(TWIST_SIZE, getNumVelocities());
    if (dmotion_subspace) {
      dmotion_subspace->resize(motion_subspace.size(), getNumPositions());
    }
  };

  template<typename Scalar>
  void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &q, const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &v,
                               Eigen::Matrix<Scalar, 6, 1> &motion_subspace_dot_times_v,
                               typename Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdq = nullptr,
                               typename Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdv = nullptr) const {
    motion_subspace_dot_times_v.setZero();

    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(TWIST_SIZE, 1);
    }

    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(TWIST_SIZE, 1);
    }
  };

  template<typename Scalar>
  void qdot2v(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q,
              Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &qdot_to_v,
              Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dqdot_to_v) const {
    qdot_to_v.resize(getNumVelocities(), getNumPositions());
    if (dqdot_to_v) {
      dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
    }
  };

  template<typename Scalar>
  void v2qdot(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &q,
              Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &v_to_qdot,
              Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dv_to_qdot) const {
    v_to_qdot.resize(getNumPositions(), getNumVelocities());
    if (dv_to_qdot) {
      dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
    }
  };

    virtual std::string getPositionName(int index) const;
    virtual Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const;
};

#endif /* DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_ */
