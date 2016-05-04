#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDAXISONEDOFJOINT_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDAXISONEDOFJOINT_H

#include "drake/multibody_dynamics/joints/Joint.h"

namespace drake {

template <typename Scalar>
class FixedAxisOneDoFJoint : public Joint<Scalar> {
 public:
  using Joint<Scalar>::getNumPositions;
  using Joint<Scalar>::getNumVelocities;

  FixedAxisOneDoFJoint(const std::string &name, const Transform3D<Scalar> &transform_to_parent_body, const SpatialVector<Scalar>& axis) :
      Joint<Scalar>(name, transform_to_parent_body, 1, 1, false), joint_axis(axis) {
    // empty
  }

  virtual std::string getPositionNamePostfix(int index) const override {
    if (index != 0) throw std::runtime_error("bad index");
    return "";
  }

  virtual VectorX<double> randomConfiguration(std::default_random_engine &generator) const override {
    return VectorX<double>::Random(getNumPositions());
  }

  virtual MotionSubspace<Scalar> motionSubspace(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    return joint_axis;
  }

  virtual SpatialVector<Scalar> motionSubspaceDotTimesV(const Eigen::Ref<VectorX<Scalar>> &q, const Eigen::Ref<VectorX<Scalar>> &v) const override {
    return SpatialVector<Scalar>::Zero();
  }

  virtual ConfigurationDerivativeToVelocity<Scalar> configurationDerivativeToVelocity(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    return ConfigurationDerivativeToVelocity<Scalar>::Identity(getNumPositions(), getNumVelocities());
  }

  virtual VelocityToConfigurationDerivative<Scalar> velocityToConfigurationDerivative(const Eigen::Ref<VectorX<Scalar>> &q) const override {
    return VelocityToConfigurationDerivative<Scalar>::Identity(getNumPositions(), getNumVelocities());
  }

  virtual VectorX<Scalar> frictionTorque(const Eigen::Ref<VectorX<Scalar>> &v) const override {
    VectorX<Scalar> ret(getNumVelocities(), 1);
    using std::abs;
    ret[0] = damping * v[0];
    Scalar coulomb_window_fraction = v[0] / coulomb_window;
    Scalar coulomb = std::min(Scalar(1), std::max(Scalar(-1), coulomb_window_fraction)) * coulomb_friction;
    ret[0] += coulomb;
    return ret;
  }

  void setDynamics(const Scalar& damping, const Scalar& coulomb_friction, const Scalar& coulomb_window) {
    this->damping = damping;
    this->coulomb_friction = coulomb_friction;
    this->coulomb_window = coulomb_window;
  }

  const SpatialVector <Scalar> &getJointAxis() const {
    return joint_axis;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF((sizeof(SpatialVector<Scalar>)%16)==0)

 private:
  SpatialVector<Scalar> joint_axis;
  Scalar damping;
  Scalar coulomb_friction;
  Scalar coulomb_window;
};

}

#endif //DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDAXISONEDOFJOINT_H
