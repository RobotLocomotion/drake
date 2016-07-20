#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDJOINT_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDJOINT_H

#include <stdexcept>
#include "drake/multibody_dynamics/joints/Joint.h"

namespace drake {

template <typename Scalar>
class FixedJoint : public Joint<Scalar> {
 public:
  using Joint<Scalar>::GetNumPositions;
  using Joint<Scalar>::GetNumVelocities;

  FixedJoint(const std::string &name, const Transform3D<Scalar> &transform_to_parent_body) :
      Joint<Scalar>(name, transform_to_parent_body, 0, 0, false) {
    // empty
  }

  virtual std::string GetPositionNamePostfix(int index) const override {
    throw std::runtime_error("bad index");
  }

  virtual VectorX<double> RandomConfiguration(std::default_random_engine &generator) const override {
    return VectorX<double>::Zero(GetNumPositions());
  }

  virtual Transform3D<Scalar> JointTransform(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    return Transform3D<Scalar>::Identity();
  }

  virtual MotionSubspaceType<Scalar> MotionSubspace(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    return MotionSubspaceType<Scalar>::Zero(TWIST_SIZE, GetNumVelocities());
  }

  virtual SpatialVector<Scalar> MotionSubspaceDotTimesV(const Eigen::Ref<const VectorX<Scalar>> &q, const Eigen::Ref<const VectorX<Scalar>> &v) const override {
    return SpatialVector<Scalar>::Zero();
  }

  virtual ConfigurationDerivativeToVelocityType<Scalar> ConfigurationDerivativeToVelocity(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    return ConfigurationDerivativeToVelocityType<Scalar>::Zero(GetNumVelocities(), GetNumPositions());
  }

  virtual VelocityToConfigurationDerivativeType<Scalar> VelocityToConfigurationDerivative(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    return VelocityToConfigurationDerivativeType<Scalar>::Zero(GetNumPositions(), GetNumVelocities());
  }

  virtual VectorX<Scalar> FrictionTorque(const Eigen::Ref<const VectorX<Scalar>> &v) const override {
    return VectorX<Scalar>::Zero(GetNumVelocities(), 1);
  }

};

}

#endif //DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDJOINT_H
