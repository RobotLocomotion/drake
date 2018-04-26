//------------------------------------------------------------------------------
// File: SimpleBlock.cc created Apr 17 2018 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause.
// This copyright notice must appear in all copies and distributions.
// MotionGenesis Professional Licensee: Toyota Research Institute
// -----------------------------------------------------------------------------
#include <iostream>
#include "drake/examples/particle1d/MG/particle1dMG.h"
#include "drake/common/autodiff.h"

// -----------------------------------------------------------------------------
namespace MotionGenesis {

// -----------------------------------------------------------------------------
template <typename T>
void Particle1dMG<T>::CalcDerivativesToStateDt(const T t,
                                               const T state[],
                                               T state_Dt[])  {
  SetVariablesFromState(state);
  CalculateSpecifiedAssignedQuantities(t);

  state_Dt[0] = xp;
  state_Dt[1] = xpp;
}

template <typename T>
Particle1dMG<T>::Particle1dMG() {

  // initialize constants, variables, and initial states. These are default
  // values, actual model values should be assigned from the plant.
  m = 1;
}

// -----------------------------------------------------------------------------
template <typename T>
void Particle1dMG<T>::SetVariablesFromState(const T state[])  {
  x = state[0];
  xp = state[1];
}

// -----------------------------------------------------------------------------
template <typename T>
void Particle1dMG<T>::CalculateSpecifiedAssignedQuantities(const T t) {
  F = cos(t);
  xpp = F / m;
}

// -----------------------------------------------------------------------------
template class Particle1dMG<double>;
template class Particle1dMG<drake::AutoDiffXd>;

}  // namespace MotionGenesis
