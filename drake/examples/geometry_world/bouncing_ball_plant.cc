#include "drake/examples/geometry_world/bouncing_ball_plant.h"

#include <algorithm>

namespace drake {
namespace examples {
namespace bouncing_ball {

using systems::Context;
using systems::Value;
using std::make_unique;

template <typename T>
BouncingBallPlant<T>::BouncingBallPlant(const Vector2<T>& init_position)
    : init_position_(init_position) {
  state_port_ =
      this->DeclareVectorOutputPort(BouncingBallVector<T>(),
                                    &BouncingBallPlant::CopyStateToOutput)
          .get_index();

    this->DeclareContinuousState(
      BouncingBallVector<T>(),
      1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
  static_assert(BouncingBallVectorIndices::kNumCoordinates == 1 + 1, "");
}

template <typename T>
BouncingBallPlant<T>::~BouncingBallPlant() {}

template <typename T>
const systems::OutputPort<T>&
BouncingBallPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_port_);
}

// Updates the state output port.
template <typename T>
void BouncingBallPlant<T>::CopyStateToOutput(
    const Context<T>& context,
    BouncingBallVector<T>* state_output_vector) const {
  state_output_vector->set_value(get_state(context).get_value());
}

// Compute the actual physics.
template <typename T>
void BouncingBallPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::max;

  const BouncingBallVector<T>& state = get_state(context);
  BouncingBallVector<T>& derivative_vector = get_mutable_state(derivatives);

  derivative_vector.set_z(state.zdot());

  const T& x = -state.z();        // Penetration depth, > 0 at penetration.
  const T& xdot = -state.zdot();  // Penetration rate, > 0 implies increasing
                                  // penetration.

  const T fN = max(0.0, k_ * x * (1.0 - d_ * xdot));

  derivative_vector.set_zdot((- m_ * g_ + fN) / m_);
}

template class BouncingBallPlant<double>;

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake
