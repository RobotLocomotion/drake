//------------------------------------------------------------------------------
// File: SimpleBlock.cc created Apr 17 2018 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause.
// This copyright notice must appear in all copies and distributions.
// MotionGenesis Professional Licensee: Toyota Research Institute
// -----------------------------------------------------------------------------
#include "drake/examples/particle1d/MG/particle1dMG.h"
#include "drake/common/autodiff.h"

// -----------------------------------------------------------------------------
namespace MotionGenesis {
template <typename T>
Particle1dMG<T>::Particle1dMG() {

  // Assign default value for this class's constant parameters (for this class,
  // only mass).
  mass = 1;
}

// -----------------------------------------------------------------------------
template <typename T>
void Particle1dMG<T>::CalcDerivativesToStateDt(const T t,
                                               const T state[],
                                               T stateDt[])  {
  SetVariablesFromState(state);

  F = cos(t);
  xDDt = F / mass;

  stateDt[0] = xDt;
  stateDt[1] = xDDt;
}

// -----------------------------------------------------------------------------
template <typename T>
void Particle1dMG<T>::SetVariablesFromState(const T state[])  {
  x = state[0];
  xDt = state[1];
}

// -----------------------------------------------------------------------------
template class Particle1dMG<double>;
template class Particle1dMG<drake::AutoDiffXd>;

}  // namespace MotionGenesis
