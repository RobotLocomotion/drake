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
Eigen::Matrix<typename BeadOnAWire<T>::DScalar, 3, 1>
  BeadOnAWire<T>::sinusoidal_function(
      const typename BeadOnAWire<T>::DScalar& s) {
  using std::cos;
  using std::sin;
  return Vector3<BeadOnAWire<T>::DScalar>(cos(s),
                          sin(s),
                          s);
}

template <class T>
typename BeadOnAWire<T>::DScalar
    BeadOnAWire<T>::inverse_sinusoidal_function(
        const Vector3<typename BeadOnAWire<T>::DScalar>& v) {
  using std::atan2;
  return atan2(v(1), v(0));
}

template <class T>
Eigen::VectorXd BeadOnAWire<T>::DoEvalConstraintEquations(
    const systems::Context<T>& context) const {
  // The constraint function is defined as:
  // g(x) = f(f⁻¹(x)) - x
  // where x is the position of the bead and f() is the parametric wire
  // function.

  // Get the position of the bead.
  const int three_d = 3;
  Eigen::Matrix<DScalar, 3, 1> x;
  const auto position = context.get_continuous_state()->
      get_generalized_position().CopyToVector();
  for (int i = 0; i < three_d; ++i)
    x(i).value() = position[i];

  // Call the inverse function if there is one.
  DScalar s;
  if (inv_f_) {
    s = inv_f_(x);
  } else {
    // TODO(edrumwri): Implement generic method.
  }

  // Get the output position.
  Eigen::Matrix<DScalar, 3, 1> fs = f_(s);
  Eigen::VectorXd xprime(3);
  for (int i=0; i< three_d; ++i)
    xprime[i] = fs(i).value().value() - x(i).value().value();

  return xprime;
}

/// Computes the time derivative of the constraint equations, evaluated at
/// the current generalized coordinates and generalized velocity.
template <class T>
Eigen::VectorXd BeadOnAWire<T>::DoEvalConstraintEquationsDot(
    const systems::Context<T>& context) const {
  // The constraint function is defined as:
  // g(x) = f(f⁻¹(x)) - x
  // where x is the position of the bead and f() is the parametric wire
  // function. Therefore:
  // dg/dt(x) = df/dt (f⁻¹(x)) ⋅ df⁻¹/dt (x) - v

  // For example, assume that f(s) = | cos(s) sin(s) s | and that
  // f⁻¹(x) = atan2(x(2),x(1)).
  //
  // Then dg/dt = | -sin(atan2(x(2),x(1))) cos(atan2(x(2),x(1))) 1 | ⋅
  //              d/dt atan2(x(2),x(1)) - | v(1) v(2) v(3) |

  // Compute df/dt (f⁻¹(x)). The result will be a vector.
  const auto& xc = context.get_continuous_state()->get_vector();
  Eigen::Matrix<BeadOnAWire<T>::DScalar, 3, 1> x;
  x(0).value() = xc.GetAtIndex(0);
  x(1).value() = xc.GetAtIndex(1);
  x(2).value() = xc.GetAtIndex(2);
  BeadOnAWire<T>::DScalar sprime = inv_f_(x);
  sprime.derivatives()(0) = 1;
  const Eigen::Matrix<BeadOnAWire<T>::DScalar, 3, 1> fprime = f_(sprime);

  // Compute df⁻¹/dt (x). This result will be a scalar.
  Eigen::Matrix<BeadOnAWire<T>::DScalar, 3, 1> xprime;
  xprime(0).value() = xc.GetAtIndex(0);
  xprime(1).value() = xc.GetAtIndex(1);
  xprime(2).value() = xc.GetAtIndex(2);
  xprime(0).value().derivatives()(0) = xc.GetAtIndex(3);
  xprime(1).value().derivatives()(0) = xc.GetAtIndex(4);
  xprime(2).value().derivatives()(0) = xc.GetAtIndex(5);
  const double dinvf_dt = inv_f_(xprime).value().derivatives()(0);

  // Set the velocity vector.
  const Eigen::Vector3d v(xc.GetAtIndex(3), xc.GetAtIndex(4),
                          xc.GetAtIndex(5));

  // Compute the result.
  const int three_d = 3;
  Eigen::VectorXd result(three_d);
  for (int i=0; i< three_d; ++i)
    result(i) = fprime(i).derivatives()(0).value()*dinvf_dt - v(i);

  return result;
}

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

template <class T>
int BeadOnAWire<T>::do_get_num_constraint_equations(
    const systems::Context<T>& context) const {
  return (coordinate_type_ == kAbsoluteCoordinates) ? 3 : 0;
}

template <class T>
Eigen::VectorXd BeadOnAWire<T>::DoCalcVelocityChangeFromConstraintImpulses(
    const systems::Context<T>& context, const Eigen::MatrixXd& J,
    const Eigen::VectorXd& lambda) const {
  DRAKE_DEMAND(coordinate_type_ == kAbsoluteCoordinates);

  // The bead on the wire is massless, so the velocity change is equal to
  // simply Jᵀλ
  return J.transpose() * lambda;
}

template <typename T>
void BeadOnAWire<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  using std::sin;
  using std::cos;
  using std::abs;

  const systems::VectorBase<T>& state = context.get_continuous_state_vector();

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // Get the inputs.
  const auto input = this->EvalEigenVectorInput(context, 0);

  // Compute the derivatives using the desired coordinate representation.
  if (coordinate_type_ == kMinimalCoordinates) {
    /*
    // Get the necessary parts of the state.
    const T s = state.GetAtIndex(0);
    const T sdot = state.GetAtIndex(1);

    // Get the external force.
    const T fext = input(0);

    // Set velocity component of derivative.
    f->SetAtIndex(0, sdot);

    // TODO(edrumwri): Compute acceleration from Lagrangian Dynamics.
     */
  } else {
    // Compute acceleration from unconstrained Newtonian dynamics.
    const T xdot = state.GetAtIndex(3);
    const T ydot = state.GetAtIndex(4);
    const T zdot = state.GetAtIndex(5);

    // Get the external force.
    const Vector3<T> fext = input.segment(0, 3);

    // Set velocity components of derivative.
    f->SetAtIndex(0, xdot);
    f->SetAtIndex(1, ydot);
    f->SetAtIndex(2, zdot);
    f->SetAtIndex(3, fext(0));
    f->SetAtIndex(4, fext(1));
    f->SetAtIndex(5, fext(2));
  }
}

template <typename T>
void BeadOnAWire<T>::SetDefaultState(const systems::Context<T>& context,
                                  systems::State<T>* state) const {
  // TODO(edrumwri): Fix default state to be consistent no matter the
  //                parametric wire function used..

  // Use a consistent default state for the sinusoidal bead-on-the-wire
  // example.
  VectorX<T> x0(6);
  const double s = 0.0;
  x0 << std::cos(s), std::sin(s), s, 0, 0, 0;
  state->get_mutable_continuous_state()->SetFromVector(x0);
}

}  // namespace bead_on_a_wire
}  // namespace drake
