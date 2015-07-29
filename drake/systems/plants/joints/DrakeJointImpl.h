#ifndef DRAKE_DRAKEJOINTIMPL_H
#define DRAKE_DRAKEJOINTIMPL_H

#include "DrakeJoint.h"

#if defined(WIN32) || defined(WIN64)
#define OVERRIDE
#else
#define OVERRIDE override
#endif

#define POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(Scalar) \
  virtual Eigen::Transform<Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q) const OVERRIDE { return derived.jointTransform(q); }; \
  virtual void motionSubspace(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q, MotionSubspaceType &motion_subspace, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dmotion_subspace = nullptr) const OVERRIDE { derived.motionSubspace(q, motion_subspace, dmotion_subspace); }; \
  virtual void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &q, const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &v, Eigen::Matrix<Scalar, 6, 1> &motion_subspace_dot_times_v, typename Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdq = nullptr, typename Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdv = nullptr) const OVERRIDE { derived.motionSubspaceDotTimesV(q ,v, motion_subspace_dot_times_v, dmotion_subspace_dot_times_vdq, dmotion_subspace_dot_times_vdv); }; \
  virtual void qdot2v(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &qdot_to_v, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dqdot_to_v) const OVERRIDE { derived.qdot2v(q, qdot_to_v, dqdot_to_v); }; \
  virtual void v2qdot(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &q, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &v_to_qdot, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dv_to_qdot) const OVERRIDE { derived.v2qdot(q, v_to_qdot, dv_to_qdot); }; \
  virtual GradientVar<Scalar, Eigen::Dynamic, 1> frictionTorque(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v, int gradient_order) const OVERRIDE { return derived.frictionTorque(v, gradient_order); };

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
