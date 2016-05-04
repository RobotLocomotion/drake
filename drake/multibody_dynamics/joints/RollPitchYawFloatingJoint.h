#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINT_ROLLPITCHYAWFLOATINGJOINT_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINT_ROLLPITCHYAWFLOATINGJOINT_H

#include "drake/multibody_dynamics/joints/Joint.h"

namespace drake {

template <typename Scalar>
class RollPitchYawFloatingJoint : public Joint<Scalar> {
 public:
  using Joint<Scalar>::GetNumPositions;
  using Joint<Scalar>::GetNumVelocities;


  RollPitchYawFloatingJoint(const std::string &name, const Transform3D<Scalar> &transform_to_parent_body) :
      Joint<Scalar>(name, transform_to_parent_body, 6, 6, true) {
    // empty
  }

  virtual std::string GetPositionNamePostfix(int index) const override {
    switch (index) {
      case 0:
        return "_x";
      case 1:
        return "_y";
      case 2:
        return "_z";
      case 3:
        return "_roll";
      case 4:
        return "_pitch";
      case 5:
        return "_yaw";
      default:
        throw std::runtime_error("bad index");
    }
  }

  virtual Eigen::VectorXd RandomConfiguration(std::default_random_engine &generator) const override {
    using namespace Eigen;

    VectorXd q(6);
    std::normal_distribution<double> normal;

    Map<Vector3d> pos(&q[0]);
    for (int i = 0; i < SPACE_DIMENSION; i++) {
      pos(i) = normal(generator);
    }

    Map<Vector3d> rpy(&q[3]);
    rpy = uniformlyRandomRPY(generator);
    return q;
  }

  virtual Transform3D<Scalar> JointTransform(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    Transform3D<Scalar> ret;
    ret.linear() = rpy2rotmat(q.template middleRows<RPY_SIZE>(SPACE_DIMENSION));
    ret.translation() = q.template middleRows<SPACE_DIMENSION>(0);
    ret.makeAffine();
    return ret;
  }

  virtual MotionSubspaceType<Scalar> MotionSubspace(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    MotionSubspaceType<Scalar> ret(TWIST_SIZE, GetNumVelocities());
    auto rpy = q.template middleRows<RPY_SIZE>(SPACE_DIMENSION);
    Eigen::Matrix<Scalar, SPACE_DIMENSION, RPY_SIZE> E;
    rpydot2angularvelMatrix(rpy, E);
    Eigen::Matrix<Scalar, 3, 3> R = rpy2rotmat(rpy);
    ret.template block<3, 3>(0, 0).setZero();
    ret.template block<3, 3>(0, 3) = R.transpose() * E;
    ret.template block<3, 3>(3, 0) = R.transpose();
    ret.template block<3, 3>(3, 3).setZero();
    return ret;
  }

  virtual SpatialVector<Scalar> MotionSubspaceDotTimesV(const Eigen::Ref<const VectorX<Scalar>> &q, const Eigen::Ref<const VectorX<Scalar>> &v) const override {
    SpatialVector<Scalar> ret;

    auto rpy = q.template middleRows<RPY_SIZE>(SPACE_DIMENSION);
    const Scalar& roll = rpy(0);
    const Scalar& pitch = rpy(1);
    const Scalar& yaw = rpy(2);

    auto pd = v.template middleRows<SPACE_DIMENSION>(0);
    const Scalar& xd = pd(0);
    const Scalar& yd = pd(1);
    const Scalar& zd = pd(2);

    auto rpyd = v.template middleRows<RPY_SIZE>(SPACE_DIMENSION);
    const Scalar& rolld = rpyd(0);
    const Scalar& pitchd = rpyd(1);
    const Scalar& yawd = rpyd(2);

    Scalar cr = cos(roll);
    Scalar sr = sin(roll);
    Scalar cp = cos(pitch);
    Scalar sp = sin(pitch);
    Scalar cy = cos(yaw);
    Scalar sy = sin(yaw);

    ret.transpose() << -pitchd * yawd * cp,
        rolld * yawd * cp * cr - pitchd * yawd * sp * sr - pitchd * rolld * sr,
        -pitchd * rolld * cr - pitchd * yawd * cr * sp - rolld * yawd * cp * sr,
        yd * (yawd * cp * cy - pitchd * sp * sy) -
            xd * (pitchd * cy * sp + yawd * cp * sy) - pitchd * zd * cp,
        zd * (rolld * cp * cr - pitchd * sp * sr) +
            xd * (rolld * (sr * sy + cr * cy * sp) -
                yawd * (cr * cy + sp * sr * sy) + pitchd * cp * cy * sr) -
            yd * (rolld * (cy * sr - cr * sp * sy) +
                yawd * (cr * sy - cy * sp * sr) - pitchd * cp * sr * sy),
        xd * (rolld * (cr * sy - cy * sp * sr) +
            yawd * (cy * sr - cr * sp * sy) + pitchd * cp * cr * cy) -
            zd * (pitchd * cr * sp + rolld * cp * sr) +
            yd * (yawd * (sr * sy + cr * cy * sp) -
                rolld * (cr * cy + sp * sr * sy) + pitchd * cp * cr * sy);
    return ret;
  }

  virtual ConfigurationDerivativeToVelocityType<Scalar> ConfigurationDerivativeToVelocity(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    return ConfigurationDerivativeToVelocityType<Scalar>::Identity(GetNumVelocities(), GetNumPositions());
  }

  virtual VelocityToConfigurationDerivativeType<Scalar> VelocityToConfigurationDerivative(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    return VelocityToConfigurationDerivativeType<Scalar>::Identity(GetNumPositions(), GetNumVelocities());
  }

  virtual VectorX<Scalar> FrictionTorque(const Eigen::Ref<const VectorX<Scalar>> &v) const override {
    return VectorX<Scalar>::Zero(GetNumVelocities(), 1);
  }
};

}


#endif //DRAKE_MULTIBODY_DYNAMICS_JOINT_ROLLPITCHYAWFLOATINGJOINT_H
