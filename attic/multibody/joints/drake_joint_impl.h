#pragma once

#include <string>

#include "drake/math/gradient.h"
#include "drake/multibody/joints/drake_joint.h"

/// @cond

#define POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(Scalar)                 \
  virtual Eigen::Transform<Scalar, 3, Eigen::Isometry> jointTransform(       \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q)   \
      const override {                                                       \
    return derived_.jointTransform(q);                                       \
  };                                                                         \
                                                                             \
  void motionSubspace(                                                       \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q,   \
      Eigen::Matrix<Scalar, drake::kTwistSize, Eigen::Dynamic, 0,            \
                    drake::kTwistSize, MAX_NUM_VELOCITIES>& motion_subspace, \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>*                 \
          dmotion_subspace = nullptr) const override {                       \
    derived_.motionSubspace(q, motion_subspace, dmotion_subspace);           \
  };                                                                         \
                                                                             \
  void motionSubspaceDotTimesV(                                              \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q,   \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v,   \
      Eigen::Matrix<Scalar, 6, 1>& motion_subspace_dot_times_v,              \
      typename drake::math::Gradient<Eigen::Matrix<Scalar, 6, 1>,            \
                                     Eigen::Dynamic>::type*                  \
          dmotion_subspace_dot_times_vdq = nullptr,                          \
      typename drake::math::Gradient<                                        \
          Eigen::Matrix<Scalar, 6, 1>,                                       \
          Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr)   \
      const override {                                                       \
    derived_.motionSubspaceDotTimesV(q, v, motion_subspace_dot_times_v,      \
                                    dmotion_subspace_dot_times_vdq,          \
                                    dmotion_subspace_dot_times_vdv);         \
  };                                                                         \
                                                                             \
  void qdot2v(                                                               \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q,   \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,               \
                    MAX_NUM_VELOCITIES, MAX_NUM_POSITIONS>& qdot_to_v,       \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* dqdot_to_v)     \
      const override {                                                       \
    derived_.qdot2v(q, qdot_to_v, dqdot_to_v);                               \
  };                                                                         \
                                                                             \
  void v2qdot(                                                               \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q,   \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,               \
                    MAX_NUM_POSITIONS, MAX_NUM_VELOCITIES>& v_to_qdot,       \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* dv_to_qdot)     \
      const override {                                                       \
    derived_.v2qdot(q, v_to_qdot, dv_to_qdot);                               \
  };                                                                         \
                                                                             \
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> frictionTorque(                   \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& v)   \
      const override {                                                       \
    return derived_.frictionTorque(v);                                       \
  };                                                                         \
                                                                             \
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> SpringTorque(                     \
      const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>& q)   \
      const override {                                                       \
    return derived_.SpringTorque(q);                                         \
  };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
template <typename Derived>
class DrakeJointImpl : public DrakeJoint {
 public:
  /* The constructor saves a reference to the concrete subclass instance and
   * passes the rest of the parameters up to the parent class's constructor.
   *
   * @param[in] derived A reference to a class derived from this one, using the
   * Curiously Recurring Template Pattern. For more information about this
   * pattern, see:
   * https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern.
   *
   * @param[in] name The name of this joint. This should be unique to a
   * model instance.
   *
   * @param[in] transform_to_parent_body The transform from this joint's frame
   * to this joint's parent link's frame.
   *
   * @param[in] num_positions The number of position states in the joint.
   *
   * @param[in] num_velocities The number of velocity states in this joint.
   */
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  DrakeJointImpl(Derived& derived, const std::string& name,
                 const Eigen::Isometry3d& transform_to_parent_body,
                 int num_positions, int num_velocities)
      : DrakeJoint(name, transform_to_parent_body, num_positions,
                   num_velocities),
        derived_(derived) {}

  virtual ~DrakeJointImpl() {}

  POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(double)
  POSITION_AND_VELOCITY_DEPENDENT_METHODS_IMPL(
      Eigen::AutoDiffScalar<Eigen::VectorXd>)

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  Derived& derived_;
};
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual

/*
 * from
 * http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 * returns 0 when val is +0 or -0
 */
template <typename T>
int sign(T val) {
  return (T(0) < val) - (val < T(0));
}

/// @endcond
