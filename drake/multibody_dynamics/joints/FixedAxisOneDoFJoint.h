#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDAXISONEDOFJOINT_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDAXISONEDOFJOINT_H

#include "drake/multibody_dynamics/joints/Joint.h"

namespace drake {

template <typename Scalar>
class FixedAxisOneDoFJoint : public Joint<Scalar> {
 public:
  using Joint<Scalar>::GetNumPositions;
  using Joint<Scalar>::GetNumVelocities;

  FixedAxisOneDoFJoint(const std::string &name, const Transform3D<Scalar> &transform_to_parent_body, const SpatialVector<Scalar>& axis) :
      Joint<Scalar>(name, transform_to_parent_body, 1, 1, false), joint_axis_(axis) {
    // empty
  }

  virtual std::string GetPositionNamePostfix(int index) const override {
    if (index != 0) throw std::runtime_error("bad index");
    return "";
  }

  virtual VectorX<double> RandomConfiguration(std::default_random_engine &generator) const override {
    return VectorX<double>::Random(GetNumPositions());
  }

  virtual MotionSubspaceType<Scalar> MotionSubspace(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    return joint_axis_;
  }

  virtual SpatialVector<Scalar> MotionSubspaceDotTimesV(const Eigen::Ref<const VectorX<Scalar>> &q, const Eigen::Ref<const VectorX<Scalar>> &v) const override {
    return SpatialVector<Scalar>::Zero();
  }

  virtual ConfigurationDerivativeToVelocityType<Scalar> ConfigurationDerivativeToVelocity(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    return ConfigurationDerivativeToVelocityType<Scalar>::Identity(GetNumPositions(), GetNumVelocities());
  }

  virtual VelocityToConfigurationDerivativeType<Scalar> VelocityToConfigurationDerivative(const Eigen::Ref<const VectorX<Scalar>> &q) const override {
    return VelocityToConfigurationDerivativeType<Scalar>::Identity(GetNumPositions(), GetNumVelocities());
  }

  virtual VectorX<Scalar> FrictionTorque(const Eigen::Ref<const VectorX<Scalar>> &v) const override {
    VectorX<Scalar> ret(GetNumVelocities(), 1);
    ret[0] = damping_ * v[0];
    Scalar coulomb_window_fraction = v[0] / coulomb_window_;
    Scalar coulomb = std::min(Scalar(1), std::max(Scalar(-1), coulomb_window_fraction)) * coulomb_friction_;
    ret[0] += coulomb;
    return ret;
  }

  void SetDynamics(const Scalar& damping, const Scalar& coulomb_friction, const Scalar& coulomb_window) {
    this->damping_ = damping;
    this->coulomb_friction_ = coulomb_friction;
    this->coulomb_window_ = coulomb_window;
  }

  const SpatialVector <Scalar> &GetJointAxis() const {
    return joint_axis_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF((sizeof(SpatialVector<Scalar>)%16)==0)

 private:
  SpatialVector<Scalar> joint_axis_;
  Scalar damping_;
  Scalar coulomb_friction_;
  Scalar coulomb_window_;
};

}

#endif //DRAKE_MULTIBODY_DYNAMICS_JOINT_FIXEDAXISONEDOFJOINT_H
