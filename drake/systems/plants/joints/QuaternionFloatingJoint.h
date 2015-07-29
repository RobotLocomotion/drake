#ifndef QUATERNIONFLOATINGJOINT_H_
#define QUATERNIONFLOATINGJOINT_H_

#include "DrakeJointImpl.h"
#include "drakeGeometryUtil.h"

class DLLEXPORT_DRAKEJOINT QuaternionFloatingJoint: public DrakeJointImpl<QuaternionFloatingJoint>
{
  // disable copy construction and assignment
  // not available in MSVC2010...
  // QuaternionFloatingJoint(const QuaternionFloatingJoint&) = delete;
  // QuaternionFloatingJoint& operator=(const QuaternionFloatingJoint&) = delete;

public:
  QuaternionFloatingJoint(const std::string & name, const Eigen::Isometry3d & transform_to_parent_body) :
          DrakeJointImpl(*this, name, transform_to_parent_body, 7, 6) { };

  virtual ~QuaternionFloatingJoint() { };

  template <typename Scalar>
  Eigen::Transform<Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::Ref< const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > & q) const {
    Eigen::Transform<Scalar, 3, Eigen::Isometry> ret(Eigen::Quaternion<Scalar>(q[3], q[4], q[5], q[6]));
    ret.translation() << q[0], q[1], q[2];
    ret.makeAffine();
    return ret;
  };

  template <typename Scalar>
  void motionSubspace(const Eigen::Ref< const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > & q,
    MotionSubspaceType& motion_subspace, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* dmotion_subspace = nullptr) const {
    motion_subspace.setIdentity(TWIST_SIZE, getNumVelocities());
    if (dmotion_subspace) {
      dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
    }
  };

  template <typename Scalar>
  void motionSubspaceDotTimesV(const Eigen::Ref< const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > & q,
    const Eigen::Ref< const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > & v, Eigen::Matrix<Scalar, 6, 1> & motion_subspace_dot_times_v,
      typename Gradient< Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq = nullptr,
      typename Gradient< Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr) const {
    motion_subspace_dot_times_v.setZero();
    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(motion_subspace_dot_times_v.size(), getNumPositions());
    }
    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(motion_subspace_dot_times_v.size(), getNumVelocities());
    }
  };

  template <typename Scalar>
  void qdot2v(const Eigen::Ref< const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > & q,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & qdot_to_v,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> * dqdot_to_v) const {
    qdot_to_v.resize(getNumVelocities(), getNumPositions());

    auto quat = q.template middleRows<QUAT_SIZE>(SPACE_DIMENSION);
    auto R = quat2rotmat(quat);

    Eigen::Matrix<Scalar, 4, 1> quattilde;
    Eigen::Matrix<Scalar, SPACE_DIMENSION, QUAT_SIZE> M;
    Eigen::Matrix<Scalar, SPACE_DIMENSION, QUAT_SIZE> RTransposeM;

    typename Gradient<Eigen::Matrix<Scalar, 4, 1>, QUAT_SIZE, 1>::type dquattildedquat;

    if (dqdot_to_v) {
      typename Gradient<Eigen::Matrix<Scalar, 4, 1>, QUAT_SIZE, 2>::type ddquattildedquat;
      normalizeVec(quat, quattilde, &dquattildedquat, &ddquattildedquat);
      auto dR = dquat2rotmat(quat);
      typename Gradient<Eigen::Matrix<Scalar, SPACE_DIMENSION, QUAT_SIZE>, QUAT_SIZE, 1>::type dM;
      quatdot2angularvelMatrix(quat, M, &dM);

      RTransposeM.noalias() = R.transpose() * M;
      auto dRTranspose = transposeGrad(dR, R.rows());
      auto dRTransposeM = matGradMultMat(R.transpose(), M, dRTranspose, dM);
      auto dRTransposeMdquattildedquat = matGradMultMat(RTransposeM, dquattildedquat, dRTransposeM, ddquattildedquat);
      dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
      setSubMatrixGradient<4>(*dqdot_to_v, dRTranspose, intRange<3>(3), intRange<3>(0), qdot_to_v.rows(), 3);
      setSubMatrixGradient<4>(*dqdot_to_v, dRTransposeMdquattildedquat, intRange<3>(0), intRange<4>(3), qdot_to_v.rows(), 3);
    }
    else {
      normalizeVec(quat, quattilde, &dquattildedquat);
      quatdot2angularvelMatrix(quat, M);
      RTransposeM.noalias() = R.transpose() * M;
    }
    qdot_to_v.template block<3, 3>(0, 0).setZero();
    qdot_to_v.template block<3, 4>(0, 3).noalias() = RTransposeM * dquattildedquat;
    qdot_to_v.template block<3, 3>(3, 0) = R.transpose();
    qdot_to_v.template block<3, 4>(3, 3).setZero();
  };

  template<typename Scalar>
  void v2qdot(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > & q,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & v_to_qdot, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> * dv_to_qdot) const {
    v_to_qdot.resize(getNumPositions(), getNumVelocities());

    auto quat = q.template middleRows<QUAT_SIZE>(SPACE_DIMENSION);
    auto R = quat2rotmat(quat);

    Eigen::Matrix<Scalar, QUAT_SIZE, SPACE_DIMENSION> M;
    if (dv_to_qdot) {
      auto dR = dquat2rotmat(quat);
      typename Gradient<decltype(M), QUAT_SIZE, 1>::type dM;
      angularvel2quatdotMatrix(quat, M, &dM);

      dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());

      setSubMatrixGradient<4>(*dv_to_qdot, dR, intRange<3>(0), intRange<3>(3), v_to_qdot.rows(), 3);
      auto dMR = matGradMultMat(M, R, dM, dR);
      setSubMatrixGradient<4>(*dv_to_qdot, dMR, intRange<4>(3), intRange<3>(0), v_to_qdot.rows(), 3);
    }
    else {
      angularvel2quatdotMatrix(quat, M, (typename Gradient<decltype(M), QUAT_SIZE, 1>::type*) nullptr);
    }

    v_to_qdot.template block<3, 3>(0, 0).setZero();
    v_to_qdot.template block<3, 3>(0, 3) = R;
    v_to_qdot.template block<4, 3>(3, 0).noalias() = M * R;
    v_to_qdot.template block<4, 3>(3, 3).setZero();
  };

  virtual bool isFloating() const { return true; };
  virtual std::string getPositionName(int index) const;
  virtual std::string getVelocityName(int index) const;
  virtual Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const; //override;
};

#endif /* QUATERNIONFLOATINGJOINT_H_ */
