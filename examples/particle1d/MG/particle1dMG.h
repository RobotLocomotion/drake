
// -----------------------------------------------------------------------------
// File: SimpleBlock.h created Apr 17 2018 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause.
// This copyright notice must appear in all copies and distributions.
// MotionGenesis Professional Licensee: Toyota Research Institute
// -----------------------------------------------------------------------------
// Instantiated templates for the following kinds of T's are provided:
// - double
// - AutoDiffXd
#include <cmath>
#include "drake/common/autodiff.h"

// -----------------------------------------------------------------------------
namespace MotionGenesis {
// This is a model of a particle that is only allowed to move horizontally on
// Earth (a Newtonian reference frame N). The particle has mass, No is a point
// fixed on Earth, and Nx is a horizontal unit vector fixed on Earth.
// -----------------------------------------------------------------------------
template <typename T>
class Particle1dMG {
 public:
  // Constructs a particle and assigns the default value of 1 kg for its sole
  // constant parameter (mass).
  Particle1dMG();

  // Calculates the time-derivative of the state and assigns these values to the
  // stateDt array.
  void CalcDerivativesToStateDt(const T t, const T state[], T stateDt[]);

  // Model parameters (constants), only need to be set once (e.g., at t = 0).
  T mass;

  // Model variables set privately via CalcDerivativesToStateDt. x is the
  // particle's Nx measure from No. xDt and xDDt are the first and second
  // time-derivatives of x, respectively. F is the Nx measure of the force on
  // the particle.
  T x, xDt, xDDt, F;
 private:
  // Set local state variables x and xDt from their corresponding values in
  // state[].
  void SetVariablesFromState(const T state[]);
};

// -----------------------------------------------------------------------------
}  // namespace MotionGenesis
