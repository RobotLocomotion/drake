#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDJOINT_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDJOINT_H

#include <stdexcept>
#include "drake/multibody_dynamics/joints/Joint.h"

namespace drake {

template <typename Scalar>
class FixedJoint : public Joint<Scalar> {
 public:
  using Joint<Scalar>::getNumPositions;
  using Joint<Scalar>::getNumVelocities;

  FixedJoint(const std::string &name, const Transform3D<Scalar> &transform_to_parent_body) :
      Joint<Scalar>(name, transform_to_parent_body, 0, 0, false) {
    // empty
  }

  virtual std::string getPositionNamePostfix(int index) const override {
    throw std::runtime_error("bad index");
  }

  virtual VectorX<double> randomConfiguration(std::default_random_engine &generator) const override {
    return VectorX<double>::Zero(getNumPositions());
  }

  virtual Transform3D<Scalar> jointTransform(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    return Transform3D<Scalar>::Identity();
  }

  virtual MotionSubspace<Scalar> motionSubspace(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    return MotionSubspace<Scalar>::Zero(TWIST_SIZE, getNumVelocities());
  }

  virtual SpatialVector<Scalar> motionSubspaceDotTimesV(const Eigen::Ref<VectorX<Scalar>> &q, const Eigen::Ref<VectorX<Scalar>> &v) const override {
    return SpatialVector<Scalar>::Zero();
  }

  virtual ConfigurationDerivativeToVelocity<Scalar> configurationDerivativeToVelocity(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    return ConfigurationDerivativeToVelocity<Scalar>::Zero(getNumVelocities(), getNumPositions());
  }

  virtual VelocityToConfigurationDerivative<Scalar> velocityToConfigurationDerivative(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    return VelocityToConfigurationDerivative<Scalar>::Zero(getNumPositions(), getNumVelocities());
  }

  virtual VectorX<Scalar> frictionTorque(const Eigen::Ref<VectorX<Scalar>> &v) const override {
    return VectorX<Scalar>::Zero(getNumVelocities(), 1);
  }

};

}

#endif //DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDJOINT_H
