#include "drake/examples/bead_on_a_wire/bead_on_a_wire.h"

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace examples {
namespace bead_on_a_wire {

template <typename T>
BeadOnAWire<T>::BeadOnAWire(BeadOnAWire<T>::CoordinateType type)
    : coordinate_type_(type) {
  const int n = (type == BeadOnAWire<T>::kMinimalCoordinates) ? 1 : 3;
  auto state_index = this->DeclareContinuousState(n, n, 0);
  this->DeclareInputPort(systems::kUseDefaultName, systems::kVectorValued, n);
  this->DeclareStateOutputPort(systems::kUseDefaultName, state_index);
}

template <class T>
Eigen::Matrix<typename BeadOnAWire<T>::ArcLength, 3, 1>
  BeadOnAWire<T>::helix_function(
      const typename BeadOnAWire<T>::ArcLength& s) {
  using std::cos;
  using std::sin;
  return Vector3<BeadOnAWire<T>::ArcLength>(cos(s),
                                          sin(s),
                                          s);
}

template <class T>
typename BeadOnAWire<T>::ArcLength
    BeadOnAWire<T>::inverse_helix_function(
        const Vector3<typename BeadOnAWire<T>::ArcLength>& v) {
  using std::atan2;
  return atan2(v(1), v(0));
}

template <class T>
Eigen::VectorXd BeadOnAWire<T>::EvalConstraintEquations(
    const systems::Context<T>& context) const {
  // Return a zero vector if this system is in minimal coordinates.
  if (coordinate_type_ == BeadOnAWire<T>::kMinimalCoordinates)
    return Eigen::VectorXd(0);

  // The constraint function is defined as:
  // g(x) = f(f⁻¹(x)) - x
  // where x is the position of the bead and f() is the parametric wire
  // function.

  // Get the position of the bead.
  constexpr int three_d = 3;
  Eigen::Matrix<ArcLength, three_d, 1> x;
  const auto position = context.get_continuous_state().
      get_generalized_position().CopyToVector();
  for (int i = 0; i < three_d; ++i)
    x(i).value() = position[i];

  // Call the inverse function.
  ArcLength s = inv_f_(x);

  // Get the output position.
  Eigen::Matrix<ArcLength, three_d, 1> fs = f_(s);
  Eigen::VectorXd xprime(three_d);
  for (int i = 0; i < three_d; ++i)
    xprime[i] = fs(i).value().value() - x(i).value().value();

  return xprime;
}

template <class T>
Eigen::VectorXd BeadOnAWire<T>::EvalConstraintEquationsDot(
    const systems::Context<T>& context) const {
  constexpr int three_d = 3;

  // Return a zero vector if this system is in minimal coordinates.
  if (coordinate_type_ == BeadOnAWire<T>::kMinimalCoordinates)
    return Eigen::VectorXd(0);

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
  const auto& xc = context.get_continuous_state().get_vector();
  Eigen::Matrix<BeadOnAWire<T>::ArcLength, three_d, 1> x;
  x(0).value() = xc.GetAtIndex(0);
  x(1).value() = xc.GetAtIndex(1);
  x(2).value() = xc.GetAtIndex(2);
  BeadOnAWire<T>::ArcLength sprime = inv_f_(x);
  sprime.derivatives()(0) = 1;
  const Eigen::Matrix<BeadOnAWire<T>::ArcLength, three_d, 1> fprime =
      f_(sprime);

  // Compute df⁻¹/dt (x). This result will be a scalar.
  Eigen::Matrix<BeadOnAWire<T>::ArcLength, three_d, 1> xprime;
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
  Eigen::VectorXd result = get_pfunction_first_derivative(fprime)*dinvf_dt - v;

  return result;
}

template <class T>
int BeadOnAWire<T>::get_num_constraint_equations(
    const systems::Context<T>&) const {
  return (coordinate_type_ == kAbsoluteCoordinates) ? 3 : 0;
}

template <class T>
Eigen::VectorXd BeadOnAWire<T>::CalcVelocityChangeFromConstraintImpulses(
    const systems::Context<T>&, const Eigen::MatrixXd& J,
    const Eigen::VectorXd& lambda) const {

  // TODO(edrumwri): Test this method as soon as DAE solver is available,
  //                 (necessarily removing abort() first).
  if (true) {
    throw std::logic_error(
        "CalcVelocityChangeFromConstraintImpulses requires testing.");
  }

  // The bead on the wire is unit mass, so the velocity change is equal to
  // simply Jᵀλ
  if (coordinate_type_ == kAbsoluteCoordinates)
    return J.transpose() * lambda;
  else
    return Eigen::Matrix<T, 1, 1>(0);
}

template <class T>
double BeadOnAWire<T>::get_inv_pfunction_output(const ArcLength& m) {
  return m.value().value();
}

template <class T>
double BeadOnAWire<T>::get_inv_pfunction_first_derivative(const ArcLength& m) {
  return m.value().derivatives()(0);
}

template <class T>
double BeadOnAWire<T>::get_inv_pfunction_second_derivative(const ArcLength& m) {
  return m.derivatives()(0).derivatives()(0);
}

template <class T>
Eigen::Vector3d BeadOnAWire<T>::get_pfunction_output(
    const Eigen::Matrix<ArcLength, 3, 1>& m) {
  const double x = m(0).value().value();
  const double y = m(1).value().value();
  const double z = m(2).value().value();
  return Eigen::Vector3d(x, y, z);
}

template <class T>
Eigen::Vector3d BeadOnAWire<T>::get_pfunction_first_derivative(
    const Eigen::Matrix<ArcLength, 3, 1>& m) {
  const double x = m(0).derivatives()(0).value();
  const double y = m(1).derivatives()(0).value();
  const double z = m(2).derivatives()(0).value();
  return Eigen::Vector3d(x, y, z);
}

template <class T>
Eigen::Vector3d BeadOnAWire<T>::get_pfunction_second_derivative(
    const Eigen::Matrix<ArcLength, 3, 1>& m) {
  const double x = m(0).derivatives()(0).derivatives()(0);
  const double y = m(1).derivatives()(0).derivatives()(0);
  const double z = m(2).derivatives()(0).derivatives()(0);
  return Eigen::Vector3d(x, y, z);
}

template <typename T>
void BeadOnAWire<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::sin;
  using std::cos;
  using std::abs;

  const systems::VectorBase<T>& state = context.get_continuous_state_vector();

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>& f = derivatives->get_mutable_vector();

  // Get the inputs.
  const VectorX<T>& input = this->get_input_port(0).Eval(context);

  // Compute the derivatives using the desired coordinate representation.
  if (coordinate_type_ == kMinimalCoordinates) {
    // Get the necessary parts of the state.
    const T s = state.GetAtIndex(0);
    const T s_dot = state.GetAtIndex(1);

    // Get the external force.
    const double tau = input(0);

    // Compute derivatives.
    ArcLength s_in;
    s_in.value().value() = s;
    s_in.derivatives()(0) = 1;
    s_in.value().derivatives()(0) = 1;
    Eigen::Matrix<ArcLength, 3, 1> foutput = f_(s_in);

    // From the description in the header file, the dynamics of the bead in
    // minimal coordinates is:
    // df/ds⋅ṡ²⋅d²f/ds² + (df/ds)₃⋅ag - (df/ds)²⋅dṡ/dt = τ
    // Implying that:
    // dṡ/dt = (-τ + (df(s)/ds)₃⋅ag - df(s)/ds⋅ṡ²⋅d²f/ds²) / (df/ds)²
    // This derivation was double-checked with the following Mathematica code:
    // L:= 1/2*(f'[s[t]] * s'[t])^2 + Dot[z, f[s[t]]]*ag
    // Solve[D[L, s[t]] - D[D[L, s'[t]], t] == tau, s''[t]]
    // where L is the definition of the Lagrangian and Dot[z, f[s[t]]] is
    // equivalent to f₃ (i.e., assuming that z is the unit vector [0 0 1]ᵀ).
    const double ag = get_gravitational_acceleration();
    const Eigen::Vector3d dfds = get_pfunction_first_derivative(foutput);
    const Eigen::Vector3d d2fds2 = get_pfunction_second_derivative(foutput);
    const double s_ddot = (tau + dfds(2)*ag - dfds.dot(d2fds2)*s_dot*s_dot) /
                          dfds.dot(dfds);

    // Set derivative.
    f.SetAtIndex(0, s_dot);
    f.SetAtIndex(1, s_ddot);
  } else {
    // Compute acceleration from unconstrained Newtonian dynamics.
    const T xdot = state.GetAtIndex(3);
    const T ydot = state.GetAtIndex(4);
    const T zdot = state.GetAtIndex(5);

    // Get the external force.
    const Vector3<T> fext = input.segment(0, 3);

    // Set velocity components of derivative.
    f.SetAtIndex(0, xdot);
    f.SetAtIndex(1, ydot);
    f.SetAtIndex(2, zdot);
    f.SetAtIndex(3, fext(0));
    f.SetAtIndex(4, fext(1));
    f.SetAtIndex(5, fext(2) + get_gravitational_acceleration());
  }
}

/// Sets the default state for the bead-on-a-wire system to `s = 0, ds/dt = 0`
/// for the bead represented in minimal coordinates; for the bead represented
/// in absolute coordinates, the default state is set to `f(0), ds/dt(0)⋅0 = 0`.
template <typename T>
void BeadOnAWire<T>::SetDefaultState(const systems::Context<T>&,
                                     systems::State<T>* state) const {
  // Use a consistent default state for the helix bead-on-the-wire
  // example.
  const double s = 1.0, s_dot = 1.0;
  VectorX<T> x0;
  if (coordinate_type_ == BeadOnAWire<T>::kAbsoluteCoordinates) {
    const int state_size = 6;

    // Evaluate the wire parameter function at s.
    ArcLength sprime;
    sprime.value() = s;
    sprime.derivatives()(0).value() = 1;
    const Eigen::Matrix<ArcLength, 3, 1> q = f_(sprime);

    // Set x0 appropriately.
    x0.resize(state_size);
    x0 << q(0).value().value(),
          q(1).value().value(),
          q(2).value().value(),
          q(0).derivatives()(0).value() * s_dot,
          q(1).derivatives()(0).value() * s_dot,
          q(2).derivatives()(0).value() * s_dot;
  } else {
    const int state_size = 2;
    x0.resize(state_size);
    x0 << s, s_dot;
  }
  state->get_mutable_continuous_state().SetFromVector(x0);
}

template class BeadOnAWire<double>;

}  // namespace bead_on_a_wire
}  // namespace examples
}  // namespace drake
