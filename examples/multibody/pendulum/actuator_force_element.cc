#include "drake/examples/multibody/pendulum/actuator_force_element.h"

#include "drake/common/default_scalars.h"
#include "drake/examples/multibody/pendulum/pendulum_plant.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace examples {
namespace multibody {
namespace pendulum {

using multibody::ForceElement;
using multibody::Joint;
using multibody::MultibodyTree;
using multibody::MultibodyTreeContext;
using multibody::PositionKinematicsCache;
using multibody::VelocityKinematicsCache;

template <typename T>
ActuatorForceElement<T>::ActuatorForceElement(
    const PendulumPlant<T>* plant, const multibody::Joint<T>* joint) :
    plant_(plant),
    joint_(joint) {}

template <typename T>
void ActuatorForceElement<T>::DoCalcAndAddForceContribution(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&,
    std::vector<SpatialForce<T>>* F_Bo_W_array,
    EigenPtr<VectorX<T>> tau) const {
  //const T& actuator_torque = plant_->get_tau(context);

  Eigen::VectorBlock<const VectorX<T>> u = plant_->EvalEigenVectorInput(
      context, plant_->get_input_port().get_index());

  // Probably better having a:
  // Eigen::VectorBlock<const VectorX<T>> plant_->GetActuatorForcing(*actuator_);

  // In this case there is only one actuator and we do know u corresponds to
  // this joint.
  joint_->AccumulateForcing(context, u, forcing);
}

template <typename T>
T ActuatorForceElement<T>::CalcPotentialEnergy(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc) const {
  return 0.0;
}

template <typename T>
T ActuatorForceElement<T>::CalcConservativePower(
    const MultibodyTreeContext<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
  return 0.0;
}

template <typename T>
T ActuatorForceElement<T>::CalcNonConservativePower(
    const MultibodyTreeContext<T>&,
    const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&) const {
  return 0.0;
}

template <typename T>
std::unique_ptr<ForceElement<double>>
ActuatorForceElement<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree) const {
  const Joint<double>& joint = tree.get_variant(*joint_);
  const ForceElement<double>& force = tree.get_variant(*forcing_)
  return std::make_unique<ActuatorForceElement<double>>(
      plant_, joint_);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
ActuatorForceElement<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>&) const {
  return std::make_unique<ActuatorForceElement<AutoDiffXd>>(plant_, joint_);
}

}  // namespace pendulum
}  // namespace multibody
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::examples::multibody::pendulum::ActuatorForceElement)