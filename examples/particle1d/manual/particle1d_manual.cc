// -----------------------------------------------------------------------------
#include "drake/examples/particle1d/manual/particle1d_manual.h"

// -----------------------------------------------------------------------------
namespace Manual {
template <typename T>
Particle1dManual<T>::Particle1dManual() {
  // Assign default value for this class's constant parameters (for this class,
  // only mass).
  mass_ = 1;
}

// -----------------------------------------------------------------------------
template <typename T>
typename Particle1dManual<T>::ParticleData Particle1dManual<T>::OutputData() {
  ParticleData output_particle_data;
  output_particle_data.x = x_;
  output_particle_data.xDt = xDt_;
  output_particle_data.xDDt = xDDt_;
  output_particle_data.F = F_;
  return output_particle_data;
}

// -----------------------------------------------------------------------------
template <typename T>
void Particle1dManual<T>::CalcDerivativesToStateDt(const T t, const T state[],
                                                   T stateDt[]) {
  SetVariablesFromState(state);

  using std::cos;
  F_ = cos(t);
  xDDt_ = F_ / mass_;

  stateDt[0] = xDt_;
  stateDt[1] = xDDt_;
}

// -----------------------------------------------------------------------------
template <typename T>
void Particle1dManual<T>::SetVariablesFromState(const T state[]) {
  x_ = state[0];
  xDt_ = state[1];
}

// -----------------------------------------------------------------------------
template class Particle1dManual<double>;
template class Particle1dManual<drake::AutoDiffXd>;

}  // namespace Manual