#ifndef DRAKE_DRAKEJOINTIMPL_H
#define DRAKE_DRAKEJOINTIMPL_H

#include "DrakeJoint.h"

#if defined(WIN32) || defined(WIN64)
#define OVERRIDE
#else
#define OVERRIDE override
#endif

#define POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(Scalar) \
  virtual Eigen::Transform<Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q) const OVERRIDE {return derived.jointTransform(q); }; \
  virtual GradientVar<Scalar, 6, Eigen::Dynamic> motionSubspace(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q) const OVERRIDE {return derived.motionSubspace(q); }; \
  virtual GradientVar<Scalar, 6, 1> motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q, const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v, int gradient_order) const OVERRIDE {return derived.motionSubspaceDotTimesV(q, v, gradient_order); }; \
  virtual GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> qdot2v(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q, int gradient_order) const OVERRIDE {return derived.qdot2v(q, gradient_order); }; \
  virtual GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> v2qdot(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q, int gradient_order) const OVERRIDE {return derived.v2qdot(q, gradient_order); }; \
  virtual GradientVar<Scalar, Eigen::Dynamic, 1> frictionTorque(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v, int gradient_order) const OVERRIDE {return derived.frictionTorque(v, gradient_order); };

template <typename Derived>
class DrakeJointImpl : public DrakeJoint
{
private:
  Derived& derived;

public:
  DrakeJointImpl(Derived& derived, const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, int num_positions,
                 int num_velocities) : DrakeJoint(name, transform_to_parent_body, num_positions, num_velocities), derived(derived) { };

  POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(double)
  POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(Eigen::AutoDiffScalar<Eigen::VectorXd>)
};

#endif //DRAKE_DRAKEJOINTIMPL_H
