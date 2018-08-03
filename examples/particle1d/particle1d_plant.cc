#include "drake/examples/particle1d/particle1d_plant.h"

#include <cmath>

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace particle1d {

// class constructor
template <typename T>
Particle1dPlant<T>::Particle1dPlant() {
  const int number_generalized_position = 1;
  const int number_generalized_velocity = 1;
  const int number_miscellaneous_variables = 0;
  this->DeclareContinuousState(number_generalized_position,
                               number_generalized_velocity,
                               number_miscellaneous_variables);

  // Provide the output port with information as to how many scalars are to be
  // returned and which method to call to fill array that the port will pass.
  // Note: prototype_output_vector is just a model of how much space is needed.
  const int number_output_scalars = 2;
  systems::BasicVector<T> prototype_output_vector(number_output_scalars);
  this->DeclareVectorOutputPort(prototype_output_vector,
                                &Particle1dPlant::CopyStateOut);
}

template <typename T>
void Particle1dPlant<T>::SetConstantParameters(
    const RigidBodyTree<double>& tree) {
  std::string body_name = "RigidBox";
  std::string model_name = "particle1d";
  // Finds the mass of a body based on the body's name and the model's name.
  // These names are used in the .urdf file (currently named particle1d.urdf).
  const auto body = tree.FindBody(body_name, model_name);
  // Pass in the mass from the urdf.
  particle1d_.set_mass(body->get_mass());
}

template <typename T>
void Particle1dPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                      systems::BasicVector<T>* output) const {
  // Get current state from the context.
  const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector();
  // Write system output.
  output->set_value(continuous_state_vector.CopyToVector());
}

// Compute the actual physics.
template <typename T>
void Particle1dPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T> *derivatives) const {

  const systems::BasicVector<T>& state = get_state(context);
  systems::BasicVector<T>& derivative = get_mutable_state(derivatives);

  int x_index = 0;
  int xDt_index = 1;
  T state_array[2];
  T stateDt_array[2];
  state_array[x_index] = state.GetAtIndex(x_index);
  state_array[xDt_index] = state.GetAtIndex(xDt_index);

  const T system_time = context.get_time();

  // Calculates the equations of motion given the current time and state.
  particle1d_.CalcDerivativesToStateDt(system_time, state_array,
                                          stateDt_array);

  // Assigns values of the state derivatives to the system derivative vector.
  xDt_index = 0;
  int xDDt_index = 1;
  derivative.SetAtIndex(xDt_index, stateDt_array[xDt_index]);
  derivative.SetAtIndex(xDDt_index, stateDt_array[xDDt_index]);
}

} // namespace particle1d
} // namespace examples
} // namespace drake

// Explicitly instantiate on non-symbolic scalar types.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::particle1d::Particle1dPlant)