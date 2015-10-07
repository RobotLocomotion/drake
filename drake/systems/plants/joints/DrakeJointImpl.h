#ifndef DRAKE_DRAKEJOINTIMPL_H
#define DRAKE_DRAKEJOINTIMPL_H

#include "DrakeJoint.h"

#define POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(Scalar) \
  virtual Eigen::Transform<Scalar, 3, Eigen::Isometry> jointTransform(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q) const override { return derived.jointTransform(q); }; \
  virtual void motionSubspace(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q, Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic, 0, TWIST_SIZE, MAX_NUM_VELOCITIES> &motion_subspace, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dmotion_subspace = nullptr) const override { derived.motionSubspace(q, motion_subspace, dmotion_subspace); }; \
  virtual void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &q, const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &v, Eigen::Matrix<Scalar, 6, 1> &motion_subspace_dot_times_v, typename Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdq = nullptr, typename Gradient<Eigen::Matrix<Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdv = nullptr) const override { derived.motionSubspaceDotTimesV(q ,v, motion_subspace_dot_times_v, dmotion_subspace_dot_times_vdq, dmotion_subspace_dot_times_vdv); }; \
  virtual void qdot2v(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> > &q, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_NUM_VELOCITIES, MAX_NUM_POSITIONS> &qdot_to_v, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dqdot_to_v) const override { derived.qdot2v(q, qdot_to_v, dqdot_to_v); }; \
  virtual void v2qdot(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> &q, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_NUM_POSITIONS, MAX_NUM_VELOCITIES> &v_to_qdot, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> *dv_to_qdot) const override { derived.v2qdot(q, v_to_qdot, dv_to_qdot); }; \
  virtual GradientVar<Scalar, Eigen::Dynamic, 1> frictionTorque(const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v, int gradient_order) const override { return derived.frictionTorque(v, gradient_order); };

template <typename Derived>
class DrakeJointImpl : public DrakeJoint
{
private:
  Derived& derived;

public:
	DrakeJointImpl(Derived& _derived, const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, int num_positions,
		int num_velocities) : DrakeJoint(name, transform_to_parent_body, num_positions, num_velocities), derived(_derived) {};
  virtual ~DrakeJointImpl() {};

  POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(double)
  POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(Eigen::AutoDiffScalar<Eigen::VectorXd>)

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

/*
 * from http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 * returns 0 when val is +0 or -0
 */
template <typename T> int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

#endif //DRAKE_DRAKEJOINTIMPL_H
