// -----------------------------------------------------------------------------
// Equation of motion for a simple particle that only has one degree of freedom.
//
// Instantiated templates for the following kinds of T's are provided:
// - double
// - AutoDiffXd
#include <cmath>
#include "drake/common/autodiff.h"

// -----------------------------------------------------------------------------
namespace Manual {
// This is a model of a particle that is only allowed to move horizontally on
// Earth (a Newtonian reference frame N). The particle has mass, No is a point
// fixed on Earth, and Nx is a horizontal unit vector fixed on Earth.
// -----------------------------------------------------------------------------
template <typename T>
class Particle1dManual {
 public:
  // Constructs a particle and assigns the default value of 1 kg for its sole
  // constant parameter (mass).
  Particle1dManual();

  // Calculates the time-derivative of the state and assigns these values to the
  // stateDt array.
  void CalcDerivativesToStateDt(const T t, const T state[], T stateDt[]);

  // Struct to hold the particle data
  struct ParticleData {
    T x_;
    T xDt_;
    T xDDt_;
    T F_;
    T mass_;
  };

  ParticleData& get_particle_data() { return particle_data_; };


 private:
  // Model variables are set via CalcDerivativesToStateDt. x_ is the
  // particle's Nx measure from No. xDt_ and xDDt_ are ẋ and ẍ (the 1ˢᵗ and 2ⁿᵈ
  // time-derivatives of x_). F_ is the Nx measure of the force on the particle.
  // T x_, xDt_, xDDt_, F_;

  // Model parameters (constants), only need to be set once (e.g., at t = 0).
  // T mass_;

  // Method to output the ParticleData as a struct.
  ParticleData particle_data_;

  // Set local state variables x_ and xDt_ from their corresponding values in
  // state[].
  void SetVariablesFromState(const T state[]);
};

// -----------------------------------------------------------------------------
}  // namespace Manual
