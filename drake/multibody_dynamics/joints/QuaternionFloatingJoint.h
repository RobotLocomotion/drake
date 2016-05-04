#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINT_QUATERNIONFLOATINGJOINT_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINT_QUATERNIONFLOATINGJOINT_H

#include "drake/multibody_dynamics/joints/Joint.h"

namespace drake {

template <typename Scalar>
class QuaternionFloatingJoint : public Joint<Scalar> {
 public:
  using Joint<Scalar>::getNumPositions;
  using Joint<Scalar>::getNumVelocities;

  QuaternionFloatingJoint(const std::string &name, const Transform3D<Scalar> &transform_to_parent_body) :
      Joint<Scalar>(name, transform_to_parent_body, 7, 6, true) {
    // empty
  }

  virtual std::string getPositionNamePostfix(int index) const override {
    switch (index) {
      case 0:
        return "_x";
      case 1:
        return "_y";
      case 2:
        return "_z";
      case 3:
        return "_qw";
      case 4:
        return "_qx";
      case 5:
        return "_qy";
      case 6:
        return "_qz";
      default:
        throw std::runtime_error("bad index");
    }
  }

  virtual std::string getVelocityNamePostfix(int index) const override {
    switch (index) {
      case 0:
        return "_wx";
      case 1:
        return "_wy";
      case 2:
        return "_wz";
      case 3:
        return "_vx";
      case 4:
        return "_vy";
      case 5:
        return "_vz";
      default:
        throw std::runtime_error("bad index");
    }
  }

  virtual VectorX<double> zeroConfiguration() const override {
    VectorX<double> ret(getNumPositions());
    ret << 0, 0, 0, 1, 0, 0, 0;
    return ret;
  }

  virtual VectorX<double> randomConfiguration(std::default_random_engine &generator) const override {
    VectorX<double> q(getNumPositions());
    std::normal_distribution<double> normal;

    // position
    q[0] = normal(generator);
    q[1] = normal(generator);
    q[2] = normal(generator);

    // orientation
    Eigen::Vector4d quat = uniformlyRandomQuat(generator);
    q[3] = quat(0);
    q[4] = quat(1);
    q[5] = quat(2);
    q[6] = quat(3);
    return q;
  }

  virtual Transform3D<Scalar> jointTransform(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    Transform3D<Scalar> ret;
    ret.linear() = quat2rotmat(q.template bottomRows<4>());
    ret.translation() << q[0], q[1], q[2];
    ret.makeAffine();
    return ret;
  }

  virtual MotionSubspace<Scalar> motionSubspace(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    return MotionSubspace<Scalar>::Identity(TWIST_SIZE, getNumVelocities());
  }

  virtual SpatialVector<Scalar> motionSubspaceDotTimesV(const Eigen::Ref<VectorX<Scalar>> &q, const Eigen::Ref<VectorX<Scalar>> &v) const override {
    return SpatialVector<Scalar>::Zero();
  }

  virtual ConfigurationDerivativeToVelocity<Scalar> configurationDerivativeToVelocity(const Eigen::Ref<VectorX<Scalar>> &q) const override {

    ConfigurationDerivativeToVelocity<Scalar> ret(getNumVelocities(), getNumPositions());
    auto quat = q.template middleRows<QUAT_SIZE>(SPACE_DIMENSION);
    auto R = quat2rotmat(quat);

    Eigen::Matrix<Scalar, 4, 1> quattilde;
    typename Gradient<Eigen::Matrix<Scalar, 4, 1>, QUAT_SIZE, 1>::type dquattildedquat;
    normalizeVec(quat, quattilde, &dquattildedquat);
    auto RTransposeM = (R.transpose() * quatdot2angularvelMatrix(quat)).eval();
    ret.template block<3, 3>(0, 0).setZero();
    ret.template block<3, 4>(0, 3).noalias() = RTransposeM * dquattildedquat;
    ret.template block<3, 3>(3, 0) = R.transpose();
    ret.template block<3, 4>(3, 3).setZero();
    return ret;
  }

  virtual VelocityToConfigurationDerivative<Scalar> velocityToConfigurationDerivative(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    VelocityToConfigurationDerivative<Scalar> ret(getNumPositions(), getNumVelocities());
    auto quat = q.template middleRows<QUAT_SIZE>(SPACE_DIMENSION);
    auto R = quat2rotmat(quat);

    Eigen::Matrix<Scalar, QUAT_SIZE, SPACE_DIMENSION> M;
    typename Gradient<decltype(M), QUAT_SIZE, 1>::type* dM = nullptr;
    angularvel2quatdotMatrix(quat, M, dM);

    ret.template block<3, 3>(0, 0).setZero();
    ret.template block<3, 3>(0, 3) = R;
    ret.template block<4, 3>(3, 0).noalias() = M * R;
    ret.template block<4, 3>(3, 3).setZero();
    return ret;
  }

  virtual VectorX<Scalar> frictionTorque(const Eigen::Ref<VectorX<Scalar>> &v) const override {
    return Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(getNumVelocities(), 1);
  }
};

}



#endif //DRAKE_MULTIBODY_DYNAMICS_JOINT_QUATERNIONFLOATINGJOINT_H
