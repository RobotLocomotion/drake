#include "drake/multibody/multibody_tree/test/floating_body_plant.h"

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {

using drake::multibody::multibody_plant::MultibodyPlant;
using Eigen::Vector3d;

template<typename T>
AxiallySymmetricFreeBodyPlant<T>::AxiallySymmetricFreeBodyPlant(
    double mass, double I, double J, double g)
    : MultibodyPlant<T>(), mass_(mass), I_(I), J_(J), g_(g) {

  // Create the MultibodyPlant model.
  // Create the spatial inertia M_Bcm of body B, about Bcm, expressed in B.
  UnitInertia<double> G_Bcm =
      UnitInertia<double>::AxiallySymmetric(J_, I_, Vector3<double>::UnitZ());
  SpatialInertia<double> M_Bcm(mass_, Vector3<double>::Zero(), G_Bcm);
  body_ = &this->AddRigidBody("FreeBody", M_Bcm);
  this->template AddForceElement<UniformGravityFieldElement>(
      -g_ * Vector3<double>::UnitZ());
  this->Finalize();

  // Some sanity checks. By default MultibodyPlant uses a quternion free
  // mobilizer for bodies that are not connected by any joint.
  DRAKE_DEMAND(this->num_positions() == 7);
  DRAKE_DEMAND(this->num_velocities() == 6);
  DRAKE_DEMAND(this->num_multibody_states() == 13);
}

template<typename T>
template<typename U>
AxiallySymmetricFreeBodyPlant<T>::AxiallySymmetricFreeBodyPlant(
    const AxiallySymmetricFreeBodyPlant<U> &other) :
    AxiallySymmetricFreeBodyPlant<T>(
    other.mass_, other.I_, other.J_, other.g_) {}

template<typename T>
Vector3<double>
AxiallySymmetricFreeBodyPlant<T>::get_default_initial_angular_velocity() {
  return Vector3d::UnitX() + Vector3d::UnitY() + Vector3d::UnitZ();
}

template<typename T>
Vector3<double>
AxiallySymmetricFreeBodyPlant<T>::get_default_initial_translational_velocity() {
  return Vector3d::UnitX();
}

template<typename T>
void AxiallySymmetricFreeBodyPlant<T>::SetDefaultState(
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  MultibodyPlant<T>::SetDefaultState(context, state);
  const SpatialVelocity<T> V_WB(
      get_default_initial_angular_velocity().template cast<T>(),
      get_default_initial_translational_velocity().template cast<T>());
  this->model().SetFreeBodySpatialVelocityOrThrow(body(), V_WB, context, state);
}

template<typename T>
Vector3<T> AxiallySymmetricFreeBodyPlant<T>::get_angular_velocity(
    const systems::Context<T>& context) const {
  return CalcSpatialVelocityInWorldFrame(context).rotational();
}

template<typename T>
Vector3<T> AxiallySymmetricFreeBodyPlant<T>::get_translational_velocity(
    const systems::Context<T>& context) const {
  return CalcSpatialVelocityInWorldFrame(context).translational();
}

template<typename T>
Isometry3<T> AxiallySymmetricFreeBodyPlant<T>::CalcPoseInWorldFrame(
    const systems::Context<T>& context) const {
  PositionKinematicsCache<T> pc(this->model().get_topology());
  this->model().CalcPositionKinematicsCache(context, &pc);
  return pc.get_X_WB(body_->node_index());
}

template<typename T>
SpatialVelocity<T>
AxiallySymmetricFreeBodyPlant<T>::CalcSpatialVelocityInWorldFrame(
    const systems::Context<T>& context) const {
  PositionKinematicsCache<T> pc(this->model().get_topology());
  this->model().CalcPositionKinematicsCache(context, &pc);
  VelocityKinematicsCache<T> vc(this->model().get_topology());
  this->model().CalcVelocityKinematicsCache(context, pc, &vc);
  return vc.get_V_WB(body_->node_index());
}

}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::multibody_tree::test::
    AxiallySymmetricFreeBodyPlant)
