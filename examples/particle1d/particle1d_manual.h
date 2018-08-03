#pragma once

#include <cmath>
#include "drake/common/autodiff.h"

// -----------------------------------------------------------------------------
namespace drake {
namespace examples {
namespace particle1d {

/// This class represents the equation of motion for a simple particle that only
/// has one degree of freedom. This model is only allowed to move horizontally on
/// Earth (a Newtonian reference frame N). The particle has mass, No is a point
/// fixed on Earth, and Nx is a horizontal unit vector fixed on Earth.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template<typename T>
class Particle1dManual {
 public:
  /// Constructs a particle and assigns the default value of 1 kg for its sole
  /// constant parameter (mass).
  Particle1dManual();

  /// Calculates the time-derivative of the state and assigns these values to
  /// the stateDt array.
  /// @param[in] t The time at which the derivative will be evaluated.
  /// @param[in] state A 2d array whose elements state[0] and state[1] are
  /// assigned as x and ẋ, respectively.
  /// @param[out] stateDt A 2d array whose elements are the time derivatives
  /// of the state.
  void CalcDerivativesToStateDt(const T t, const T state[], T stateDt[]);

  /// Struct to hold the particle member data.
  struct ParticleData {
    /// The particles Nx measure from a point fixed in N [m].
    T x;

    /// The 1ˢᵗ time derivative of the particles position (velocity) [m/s].
    T xDt;

    /// The 2ⁿᵈ time derivative of the particles position (acceleration) [m/s²].
    T xDDt;

    /// The force applied to the particle [N].
    T F;

    /// The mass of the particle [kg].
    T mass;
  };

  /// Returns a ParticleData struct which contains the particle's relevant data
  /// such as it's state, mass, and time derivatives.
  /// @returns ParticleData struct with particle's relevant data.
  const ParticleData& get_particle_data() const { return particle_data_; };

  /// Sets the particle's mass. The mass of the particle is initialized to 1 kg
  /// in the constructor.
  /// @param[in] mass mass of the particle in kg.
  void set_mass(T mass) { particle_data_.mass = mass; }

 private:
  // ParticleData object to hold the member data.
  ParticleData particle_data_;
};

// -----------------------------------------------------------------------------

} // namespace particle1d
} // namespace examples
} // drake
