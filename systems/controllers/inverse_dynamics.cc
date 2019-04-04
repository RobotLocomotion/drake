#include "drake/systems/controllers/inverse_dynamics.h"

#include <vector>

using drake::multibody::MultibodyPlant;

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
InverseDynamics<T>::InverseDynamics(const MultibodyPlant<T>* plant,
                                    const InverseDynamicsMode mode)
    : multibody_plant_(plant),
      mode_(mode),
      q_dim_(plant->num_positions()),
      v_dim_(plant->num_velocities()) {
  DRAKE_DEMAND(multibody_plant_);
  DRAKE_DEMAND(plant->is_finalized());

  input_port_index_state_ =
      this->DeclareInputPort(kVectorValued, q_dim_ + v_dim_).get_index();
  output_port_index_force_ =
      this->DeclareVectorOutputPort(BasicVector<T>(v_dim_),
                                    &InverseDynamics<T>::CalcOutputForce)
          .get_index();

  // Make context with default parameters.
  multibody_plant_context_ = plant->CreateDefaultContext();

  // Doesn't declare desired acceleration input port if we are only doing
  // gravity compensation.
  if (!this->is_pure_gravity_compensation()) {
    input_port_index_desired_acceleration_ =
        this->DeclareInputPort(kVectorValued, v_dim_).get_index();
  }
}

template <typename T>
InverseDynamics<T>::~InverseDynamics() = default;

template <typename T>
void InverseDynamics<T>::CalcOutputForce(const Context<T>& context,
                                          BasicVector<T>* output) const {
  // State input.
  VectorX<T> x = get_input_port_estimated_state().Eval(context);

  // Desired acceleration input.
  VectorX<T> desired_vd = VectorX<T>::Zero(v_dim_);

  if (!this->is_pure_gravity_compensation()) {
    // Only eval acceleration input port when we are not in pure gravity
    // compensation mode.
    desired_vd = get_input_port_desired_acceleration().Eval(context);
  } else {
    // Sets velocity to zero in pure gravity compensation.
    x.tail(v_dim_).setZero();
  }

  auto& plant = *multibody_plant_;
  // Set the position and velocity in the context.
  plant.SetPositionsAndVelocities(multibody_plant_context_.get(), x);

  if (this->is_pure_gravity_compensation()) {
    output->get_mutable_value() =
        -plant.CalcGravityGeneralizedForces(*multibody_plant_context_);
  } else {
    // Compute inverse dynamics.
    multibody::MultibodyForces<T> external_forces(plant);
    plant.CalcForceElementsContribution(
        *multibody_plant_context_, &external_forces);
    output->get_mutable_value() = plant.CalcInverseDynamics(
        *multibody_plant_context_, desired_vd, external_forces);
  }
}

template class InverseDynamics<double>;
// TODO(siyuan) template on autodiff.
// template class InverseDynamics<AutoDiffXd>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
