#include "drake/multibody/test_utilities/floating_body_plant.h"

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace test {

using Eigen::Vector3d;

template <typename T>
AxiallySymmetricFreeBodyPlant<T>::AxiallySymmetricFreeBodyPlant(
    double mass, double I, double J, double g, double time_step)
    : mass_(mass),
      I_(I),
      J_(J),
      g_(g),
      plant_(std::make_unique<MultibodyPlant<T>>(time_step)) {
  // Create the spatial inertia M_Bcm of body B, about Bcm, expressed in B.
  UnitInertia<double> G_Bcm =
      UnitInertia<double>::AxiallySymmetric(J_, I_, Vector3<double>::UnitZ());
  SpatialInertia<double> M_Bcm(mass_, Vector3<double>::Zero(), G_Bcm);
  body_ = &plant_->AddRigidBody("FreeBody", M_Bcm);
  plant_->mutable_gravity_field().set_gravity_vector(-g_ *
                                                     Vector3<double>::UnitZ());
  plant_->Finalize();

  // Some sanity checks. By default MultibodyPlant uses a quternion free
  // mobilizer for bodies that are not connected by any joint.
  DRAKE_DEMAND(plant_->num_positions() == 7);
  DRAKE_DEMAND(plant_->num_velocities() == 6);
  DRAKE_DEMAND(plant_->num_multibody_states() == 13);
}

template <typename T>
AxiallySymmetricFreeBodyPlant<T>::~AxiallySymmetricFreeBodyPlant() = default;

template <typename T>
Vector3<double>
AxiallySymmetricFreeBodyPlant<T>::get_default_initial_angular_velocity() {
  return Vector3d::UnitX() + Vector3d::UnitY() + Vector3d::UnitZ();
}

template <typename T>
Vector3<double>
AxiallySymmetricFreeBodyPlant<T>::get_default_initial_translational_velocity() {
  return Vector3d::UnitX();
}

template <typename T>
std::unique_ptr<systems::Context<T>>
AxiallySymmetricFreeBodyPlant<T>::CreatePlantContext() const {
  const SpatialVelocity<T> V_WB(
      get_default_initial_angular_velocity().template cast<T>(),
      get_default_initial_translational_velocity().template cast<T>());
  std::unique_ptr<systems::Context<T>> context = plant_->CreateDefaultContext();
  this->tree().SetFreeBodySpatialVelocityOrThrow(body(), V_WB, *context,
                                                 &context->get_mutable_state());
  return context;
}

template <typename T>
Vector3<T> AxiallySymmetricFreeBodyPlant<T>::get_angular_velocity(
    const systems::Context<T>& context) const {
  return CalcSpatialVelocityInWorldFrame(context).rotational();
}

template <typename T>
Vector3<T> AxiallySymmetricFreeBodyPlant<T>::get_translational_velocity(
    const systems::Context<T>& context) const {
  return CalcSpatialVelocityInWorldFrame(context).translational();
}

template <typename T>
math::RigidTransform<T> AxiallySymmetricFreeBodyPlant<T>::CalcPoseInWorldFrame(
    const systems::Context<T>& context) const {
  internal::PositionKinematicsCache<T> pc(this->tree().forest());
  this->tree().CalcPositionKinematicsCache(context, &pc);
  return math::RigidTransform<T>(pc.get_X_WB(body_->mobod_index()));
}

template <typename T>
SpatialVelocity<T>
AxiallySymmetricFreeBodyPlant<T>::CalcSpatialVelocityInWorldFrame(
    const systems::Context<T>& context) const {
  internal::PositionKinematicsCache<T> pc(this->tree().forest());
  this->tree().CalcPositionKinematicsCache(context, &pc);
  internal::VelocityKinematicsCache<T> vc(this->tree().forest());
  this->tree().CalcVelocityKinematicsCache(context, pc, &vc);
  return vc.get_V_WB(body_->mobod_index());
}

template class AxiallySymmetricFreeBodyPlant<double>;

}  // namespace test
}  // namespace multibody
}  // namespace drake
