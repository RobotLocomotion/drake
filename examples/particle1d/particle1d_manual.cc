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
  particle_data_.mass_ = 1;
  //mass_ = 1;
}

// -----------------------------------------------------------------------------
/*
template <typename T>
typename Particle1dManual<T>::ParticleData Particle1dManual<T>::OutputData() {
  ParticleData output_particle_data;
  output_particle_data.x = x_;
  output_particle_data.F = F_;
  return output_particle_data;
}*/

// -----------------------------------------------------------------------------
template<typename T>
void Particle1dManual<T>::CalcDerivativesToStateDt(const T t, const T state[],
                                                   T stateDt[]) {
  SetVariablesFromState(state);

  using std::cos;
  //F_ = cos(t);
  //xDDt_ = F_ / mass_;

  particle_data_.F_ = cos(t);
  particle_data_.xDDt_ = particle_data_.F_ / particle_data_.mass_;

  stateDt[0] = particle_data_.xDt_;
  stateDt[1] = particle_data_.xDDt_;
}

// -----------------------------------------------------------------------------
template<typename T>
void Particle1dManual<T>::SetVariablesFromState(const T state[]) {
  particle_data_.x_ = state[0];
  particle_data_.xDt_ = state[1];
}

// -----------------------------------------------------------------------------
template
class Particle1dManual<double>;
template
class Particle1dManual<drake::AutoDiffXd>;

} // namespace particle1d
} // namespace examples
} // namespace drake
