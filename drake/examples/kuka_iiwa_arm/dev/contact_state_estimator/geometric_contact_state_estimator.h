# pragma once

#include<memory>

#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace contact_state_estimator {

template <typename T>
class GeometricContactStateEstimator {
 public:
  GeometricContactStateEstimator(
      std::unique_ptr<RigidBodyTree<T>> tree);

  void ComputeContactResults(const VectorX<T>& x,
                             systems::ContactResults<T>* contact_results );

  const RigidBodyTreed& get_rigid_body_tree() const;
 private:
  T ComputeFrictionCoefficient(T v_tangent_BAc);
  Matrix3<T> ComputeBasisFromZ(const Vector3<T>& z_axis_W);
  std::unique_ptr<RigidBodyTree<T>> tree_;
  T step5(T x);

  // Some parameters defining the contact.
  // TODO(amcastro-tri): Implement contact materials for the RBT engine.
  // These default values are all semi-arbitrary.  They seem to produce,
  // generally, plausible results. They are in *no* way universally valid or
  // meaningful.
  T penetration_stiffness_{10000.0};
  T dissipation_{2};
  // Note: this is the *inverse* of the v_stiction_tolerance parameter to
  // optimize for the division.
  T inv_v_stiction_tolerance_{100};  // inverse of 1 cm/s.
  T static_friction_coef_{0.9};
  T dynamic_friction_ceof_{0.5};
};

} // namespace contact_state_estimator
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake