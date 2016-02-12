#ifndef QUATERNIONFLOATINGJOINT_H_
#define QUATERNIONFLOATINGJOINT_H_

#include "DrakeJointImpl.h"
#include "drake/util/drakeGeometryUtil.h"

class DRAKEJOINTS_EXPORT QuaternionFloatingJoint: public DrakeJointImpl<QuaternionFloatingJoint>
{
  // disable copy construction and assignment
  //QuaternionFloatingJoint(const QuaternionFloatingJoint&) = delete;
  //QuaternionFloatingJoint& operator=(const QuaternionFloatingJoint&) = delete;

public:
  QuaternionFloatingJoint(const std::string & name, const Eigen::Isometry3d & transform_to_parent_body) :
          DrakeJointImpl(*this, name, transform_to_parent_body, 7, 6) { };

  virtual ~QuaternionFloatingJoint() { };

  template <typename DerivedQ>
  Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::MatrixBase<DerivedQ> & q) const {
    Eigen::Transform<typename DerivedQ::Scalar, 3, Eigen::Isometry> ret;
    ret.linear() = quat2rotmat(q.template bottomRows<4>());
    ret.translation() << q[0], q[1], q[2];
    ret.makeAffine();
    return ret;
  };

  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(const Eigen::MatrixBase<DerivedQ> & q,
    Eigen::MatrixBase<DerivedMS>& motion_subspace,
    typename Gradient<DerivedMS, Eigen::Dynamic>::type* dmotion_subspace = nullptr) const {
    motion_subspace.setIdentity(TWIST_SIZE, getNumVelocities());
    if (dmotion_subspace) {
      dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
    }
  };

  template<typename DerivedQ, typename DerivedV>
  void motionSubspaceDotTimesV(const Eigen::MatrixBase<DerivedQ> &q, const Eigen::MatrixBase<DerivedV> &v,
                               Eigen::Matrix<typename DerivedQ::Scalar, 6, 1> &motion_subspace_dot_times_v,
                               typename Gradient<Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdq = nullptr,
                               typename Gradient<Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdv = nullptr) const {
    motion_subspace_dot_times_v.setZero();
    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(motion_subspace_dot_times_v.size(), getNumPositions());
    }
    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(motion_subspace_dot_times_v.size(), getNumVelocities());
    }
  };

  template<typename DerivedQ>
  void qdot2v(const Eigen::MatrixBase<DerivedQ> & q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_VELOCITIES, DrakeJoint::MAX_NUM_POSITIONS> &qdot_to_v,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> *dqdot_to_v) const {
    if (dqdot_to_v) {
      throw std::runtime_error("no longer supported");
    }

    qdot_to_v.resize(getNumVelocities(), getNumPositions());
    typedef typename DerivedQ::Scalar Scalar;
    auto quat = q.template middleRows<QUAT_SIZE>(SPACE_DIMENSION);
    auto R = quat2rotmat(quat);

    Eigen::Matrix<Scalar, 4, 1> quattilde;
    typename Gradient<Eigen::Matrix<Scalar, 4, 1>, QUAT_SIZE, 1>::type dquattildedquat;
    normalizeVec(quat, quattilde, &dquattildedquat);
    auto RTransposeM = (R.transpose() * quatdot2angularvelMatrix(quat)).eval();
    qdot_to_v.template block<3, 3>(0, 0).setZero();
    qdot_to_v.template block<3, 4>(0, 3).noalias() = RTransposeM * dquattildedquat;
    qdot_to_v.template block<3, 3>(3, 0) = R.transpose();
    qdot_to_v.template block<3, 4>(3, 3).setZero();
  };

  template<typename DerivedQ>
  void v2qdot(const Eigen::MatrixBase<DerivedQ> & q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_POSITIONS, DrakeJoint::MAX_NUM_VELOCITIES> &v_to_qdot,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> *dv_to_qdot) const {
    typedef typename DerivedQ::Scalar Scalar;
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

  template <typename DerivedV>
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorque(const Eigen::MatrixBase<DerivedV> & v) const
  {
    return Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1>::Zero(getNumVelocities(), 1);
  }

  virtual bool isFloating() const override { return true; };
  virtual std::string getPositionName(int index) const override;
  virtual std::string getVelocityName(int index) const override;
  virtual Eigen::VectorXd zeroConfiguration() const override;
  virtual Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const override;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* QUATERNIONFLOATINGJOINT_H_ */
