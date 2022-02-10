#pragma once

/** @file
 This file provides utilities that facilitates testing the subclasses of
 ConstitutiveModel instead of the ConstitutiveModel class itself. */

#include <array>

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

/* Creates an array of arbitrary autodiff deformation gradients. */
template <int num_locations>
std::array<Matrix3<AutoDiffXd>, num_locations> MakeDeformationGradients();

/* Tests the constructor and the accessors for St.Venant-Kirchhoff like
constitutive models.
@tparam Model    Must be instantiations of LinearConstitutiveModel or
CorotatedModel. */
template <class Model>
void TestParameters();

/* Tests that the energy density and the stress are zero at the undeformed
state.
@tparam Model    Must be instantiations of a concrete ConstitutiveModel. */
template <class Model>
void TestUndeformedState();

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

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
