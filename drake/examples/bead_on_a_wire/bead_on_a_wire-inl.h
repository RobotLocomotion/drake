#pragma once

// @file
// Template method implementations for bead_on_a_wire.h.
// Most users should only include that file, not this one.
// For background, see http://drake.mit.edu/cxx_inl.html.

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/examples/bead_on_a_wire/bead_on_a_wire.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace bead_on_a_wire {

template <typename T>
BeadOnAWire<T>::BeadOnAWire(BeadOnAWire<T>::CoordinateType type) {
  if (type == BeadOnAWire<T>::kMinimalCoordinates) {
    this->DeclareContinuousState(1, 1, 0);
    this->DeclareInputPort(systems::kVectorValued, 1);
    this->DeclareOutputPort(systems::kVectorValued, 2);
  } else {
    this->DeclareContinuousState(3, 3, 0);
    this->DeclareInputPort(systems::kVectorValued, 3);
    this->DeclareOutputPort(systems::kVectorValued, 6);
  }
  coordinate_type_ = type;
}

template <class T>
//Vector3<Eigen::AutoDiffScalar<Eigen::Matrix<Eigen::AutoDiffScalar<drake::Vector1d>, 1, 1>>>
  Eigen::Matrix<typename BeadOnAWire<T>::DScalar, 3, 1>
  BeadOnAWire<T>::sinusoidal_function(const typename BeadOnAWire<T>::DScalar& s) {
  using std::cos;
  using std::sin;
  return Vector3<BeadOnAWire<T>::DScalar>(cos(s),
                          sin(s),
                          s);
}

template <class T>
typename BeadOnAWire<T>::DScalar BeadOnAWire<T>::inverse_sinusoidal_function(const Vector3<typename BeadOnAWire<T>::DScalar>& v) {
  using std::atan2;
  return atan2(v(1), v(0));
 }

/*
template <class T, class ADiff>
Eigen::Vector3<ADiff> BeadOnAWire<T>::sinusoidal_function(
    const ADiff& s) {
  return Eigen::Vector3<ADiff>(std::cos(s), std::sin(s), s);
}


template <class T>
Eigen::VectorXd BeadOnAWire<T>::CalcVelocityChangeFromConstraintImpulses(
        const systems::Context<T>& context,
        const Eigen::VectorXd& lambda) const {
  // Determine the Jacobian matrix of the partial derivatives of the constraint
  // equations taken with respect to the generalized velocity variables.
  Eigen::MatrixXd J = CalcConstraintJacobian(context);

  // The bead has unit mass, so the constraint impulse yields a change in
  // velocity of just J^T * lambda
  return (J.transpose() * lambda);
}
*/

/*
/// Computes the time derivative of the constraint equations, evaluated at
/// the current generalized coordinates and generalized velocity.
template <class T>
Eigen::VectorXd BeadOnAWire<T>::EvalConstraintEquationDot(
    const systems::Context<T>& context) const {
  // The constraint function is defined as:
  // g(x) = f(f⁻¹(x)) - x
  // where x is the position of the bead and f() is the parametric wire
  // function. Therefore:
  // dg/dt(x) = df/dt (f⁻¹(x)) ⋅ df⁻¹/dt (x) - v

  // For example, assume that f(s) = | cos(s) sin(s) s | and that f⁻¹(x) = x₃
  // Then dg/dt = | -sin(v₃) cos(v₃) 1 | ⋅ v₃ - v₃
  // TODO(edrumwri): Finish implementing this.
}

template <class T>
Eigen::VectorXd BeadOnAWire<T>::CalcConstraintEquationOutput(
    const systems::Context<T>& context) const {
  // The constraint function is defined as:
  // g(x) = f(f⁻¹(x)) - x
  // where x is the position of the bead and f() is the parametric wire
  // function.

  // Get the position of the bead.
  const Vector3<T> x = context.get_continuous_state()->
      get_generalized_position().CopyToVector();

  // Call the inverse function if there is one.
  double s;
  if (inv_f_)
    s = inv_f_(x);
  else {
    // TODO(edrumwri): Implement generic method.
  }

  // Call the forward method and return the differene
  return f_(s) - x;
}
*/

template <typename T>
void BeadOnAWire<T>::DoCalcOutput(const systems::Context<T>& context,
                               systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  output_vector->get_mutable_value() =
      context.get_continuous_state()->CopyToVector();
}

template <typename T>
void BeadOnAWire<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  using std::sin;
  using std::cos;
  using std::abs;
  /*
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // Get the inputs.
  const auto input = this->EvalEigenVectorInput(context, 0);

  // Compute the derivatives using the desired coordinate representation.
  if (coordinate_type_ == kMinimalCoordinates) {
    // Get the necessary parts of the state.
    const T s = state.GetAtIndex(0);
    const T sdot = state.GetAtIndex(1);

    // Get the external force.
    const T fext = input(0);

    // Set velocity component of derivative.
    f->SetAtIndex(0, sdot);

    // TODO(edrumwri): Compute acceleration from Lagrangian Dynamics.
  } else {
    // Compute acceleration from unconstrained Newtonian dynamics.
    const T x = state.GetAtIndex(0);
    const T y = state.GetAtIndex(1);
    const T z = state.GetAtIndex(2);
    const T xdot = state.GetAtIndex(3);
    const T ydot = state.GetAtIndex(4);
    const T zdot = state.GetAtIndex(5);

    // Get the external force.
    const Vector3<T> fext = input.segment(0,3);

    // Set velocity components of derivative.
    f->SetAtIndex(0, xdot);
    f->SetAtIndex(1, ydot);
    f->SetAtIndex(2, zdot);

    // TODO(edrumwri): Compute accelerations.
  }
   */
}

template <typename T>
void BeadOnAWire<T>::SetDefaultState(const systems::Context<T>& context,
                                  systems::State<T>* state) const {
  // TODO(edrumwri): Fix default state to be consistent.
  VectorX<T> x0(6);
  x0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // Initial state.
  state->get_mutable_continuous_state()->SetFromVector(x0);
}

}  // namespace bead_on_a_wire
}  // namespace drake
