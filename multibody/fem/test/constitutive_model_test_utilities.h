#pragma once

/** @file
 This file provides utilities that facilitate testing the subclasses of
 ConstitutiveModel instead of the ConstitutiveModel class itself. */

#include <array>

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

/* Creates an array of arbitrary autodiff deformation gradients. */
Matrix3<AutoDiffXd> MakeDeformationGradients();

/* Tests the constructor and the accessors for St.Venant-Kirchhoff like
constitutive models.
@tparam Model    Must be instantiations of LinearConstitutiveModel,
                 LinearCorotatedModel, CorotatedModel, or NeoHookeanModel. */
template <class Model>
void TestParameters();

/* Tests that the energy density and the stress are zero at the undeformed
state.
@param  nonzero_rest_state The model is allowed to have non-zero energy at the
                           undeformed state, i.e. the energy minimum value is
                           not zero.
@tparam Model    Must be instantiations of a concrete ConstitutiveModel. */
template <class Model>
void TestUndeformedState(bool nonzero_rest_state = false);

/* Tests that the energy density and the stress are consistent by verifying
the stress matches the derivative of energy density produced by automatic
differentiation.
@tparam Model    Must be AutoDiffXd instantiations of a concrete
ConstitutiveModel. */
template <class Model>
void TestPIsDerivativeOfPsi();

/* Tests that the stress and the stress derivatives are consistent by verifying
the handcrafted derivative matches that produced by automatic differentiation.
@tparam Model    Must be AutoDiffXd instantiations of a concrete
ConstitutiveModel. */
template <class Model>
void TestdPdFIsDerivativeOfP();

/* Tests that the filtered Hessian is dPdF with its negative eigenvalues set to
tiny positive numbers.
@tparam Model    Must be AutoDiffXd instantiations of a concrete
ConstitutiveModel. */
template <class Model>
void TestSpdness();

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
