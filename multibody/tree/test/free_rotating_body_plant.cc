#include "drake/multibody/tree/test/free_rotating_body_plant.h"

#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/multibody_tree-inl.h"

namespace drake {
namespace multibody {
namespace test {

using Eigen::Vector3d;

template<typename T>
FreeRotatingBodyPlant<T>::FreeRotatingBodyPlant(double I, double J) :
    internal::MultibodyTreeSystem<T>(), I_(I), J_(J) {
  BuildMultibodyTreeModel();
  DRAKE_DEMAND(tree().num_positions() == 3);
  DRAKE_DEMAND(tree().num_velocities() == 3);
  DRAKE_DEMAND(tree().num_states() == 6);
}

template<typename T>
template<typename U>
FreeRotatingBodyPlant<T>::FreeRotatingBodyPlant(
    const FreeRotatingBodyPlant<U> &other) : FreeRotatingBodyPlant<T>(I_, J_) {}

template <typename T>
void FreeRotatingBodyPlant<T>::BuildMultibodyTreeModel() {
  UnitInertia<double> G_Bcm =
      UnitInertia<double>::AxiallySymmetric(J_, I_, Vector3<double>::UnitZ());
  const double kMass = 1.0;
  SpatialInertia<double> M_Bcm(kMass, Vector3<double>::Zero(), G_Bcm);

  body_ = &this->mutable_tree().template AddBody<RigidBody>(M_Bcm);
  mobilizer_ = &this->mutable_tree().template AddMobilizer<
      internal::SpaceXYZMobilizer>(
          tree().world_frame(), body_->body_frame());

  internal::MultibodyTreeSystem<T>::Finalize();
}

template<typename T>
Vector3<double>
FreeRotatingBodyPlant<T>::get_default_initial_angular_velocity() const {
  return Vector3d::UnitX() + Vector3d::UnitY() + Vector3d::UnitZ();
}

template<typename T>
void FreeRotatingBodyPlant<T>::SetDefaultState(
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  internal::MultibodyTreeSystem<T>::SetDefaultState(context, state);

  mobilizer_->set_angular_velocity(
      context, get_default_initial_angular_velocity(), state);
}

template<typename T>
Vector3<T> FreeRotatingBodyPlant<T>::get_angular_velocity(
    const systems::Context<T>& context) const {
  return mobilizer_->get_angular_velocity(context);
}

template<typename T>
void FreeRotatingBodyPlant<T>::set_angular_velocity(
    systems::Context<T>* context, const Vector3<T>& w_WB) const {
  mobilizer_->set_angular_velocity(context, w_WB);
}

template<typename T>
math::RigidTransform<T> FreeRotatingBodyPlant<T>::CalcPoseInWorldFrame(
    const systems::Context<T>& context) const {
  auto& pc = this->EvalPositionKinematics(context);
  return math::RigidTransform<T>(pc.get_X_WB(body_->node_index()));
}

template<typename T>
SpatialVelocity<T> FreeRotatingBodyPlant<T>::CalcSpatialVelocityInWorldFrame(
    const systems::Context<T>& context) const {
  auto& vc = this->EvalVelocityKinematics(context);
  return vc.get_V_WB(body_->node_index());
}

}  // namespace test
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::test::FreeRotatingBodyPlant)
