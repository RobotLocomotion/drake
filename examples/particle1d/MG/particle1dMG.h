
// -----------------------------------------------------------------------------
// File: SimpleBlock.h created Apr 17 2018 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause.
// This copyright notice must appear in all copies and distributions.
// MotionGenesis Professional Licensee: Toyota Research Institute
// -----------------------------------------------------------------------------
#include <cmath>

// -----------------------------------------------------------------------------
namespace MotionGenesis {

// -----------------------------------------------------------------------------
template <typename T>
class Particle1dMG {
 public:
  // Construct a model of a particle.
  Particle1dMG();

  // Calculates the time-derivative of the state and assigns the value to the
  // state_Dt array.
  void CalcDerivativesToStateDt(const T t, const T state[], T state_Dt[]);

  // Model parameters (constants), only need to be set once (e.g., at t = 0).
  T m;

 private:
  // Assigns state variables from system context to local variables
  void SetVariablesFromState(const T state[]);

  // Calculate any specialized outputs
  void CalculateSpecifiedAssignedQuantities(const T t);

  // Model variables (not constants), set privately from state.
  T x, xp, xpp, F;
};

// -----------------------------------------------------------------------------
}  // namespace MotionGenesis
