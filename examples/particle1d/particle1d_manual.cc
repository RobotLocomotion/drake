// -----------------------------------------------------------------------------
#include "drake/examples/particle1d/particle1d_manual.h"

// -----------------------------------------------------------------------------
namespace drake {
namespace examples {
namespace particle1d {

template<typename T>
Particle1dManual<T>::Particle1dManual() {
  // Assign default value for this class's constant parameters (for this class,
  // only mass).
  this->set_mass(1);
}

// -----------------------------------------------------------------------------
template<typename T>
void Particle1dManual<T>::CalcDerivativesToStateDt(const T t, const T state[],
                                                   T stateDt[]) {
  // Set state variables x and xDt from their corresponding values in state[].
  particle_data_.x = state[0];
  particle_data_.xDt = state[1];

  using std::cos;
  particle_data_.F = cos(t);
  particle_data_.xDDt = particle_data_.F / particle_data_.mass;

  stateDt[0] = particle_data_.xDt;
  stateDt[1] = particle_data_.xDDt;
}

// -----------------------------------------------------------------------------
template class Particle1dManual<double>;
template class Particle1dManual<AutoDiffXd>;

} // namespace particle1d
} // namespace examples
} // namespace drake
