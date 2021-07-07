#pragma once

#include <cmath>
#include <stdexcept>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// The state of a one-dimensional spring-mass system, consisting of the
/// position and velocity of the mass, in meters and meters/s.
///
/// @tparam_default_scalar
template <typename T>
class SpringMassStateVector : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringMassStateVector)

  /// @param initial_position The position of the mass in meters.
  /// @param initial_velocity The velocity of the mass in meters / second.
  SpringMassStateVector(const T& initial_position, const T& initial_velocity);
  /// Creates a state with position and velocity set to zero.
  SpringMassStateVector();
  ~SpringMassStateVector() override;

  /// Returns the position of the mass in meters, where zero is the point
  /// where the spring exerts no force.
  T get_position() const;

  /// Sets the position of the mass in meters.
  void set_position(const T& q);

  /// Returns the velocity of the mass in meters per second.
  T get_velocity() const;

  /// Sets the velocity of the mass in meters per second.
  void set_velocity(const T& v);

  /// Returns the integral of conservative power, in watts.
  T get_conservative_work() const;

  /// Initialize the conservative work integral to a given value.
  void set_conservative_work(const T& e);

 private:
  [[nodiscard]] SpringMassStateVector<T>* DoClone() const override;
};

/// A model of a one-dimensional spring-mass system.
///
/// @verbatim
/// |-----\/\/ k /\/\----( m )  +x
/// @endverbatim
/// Units are MKS (meters-kilograms-seconds).
///
/// @tparam_default_scalar
/// @ingroup rigid_body_systems
template <typename T>
class SpringMassSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringMassSystem)

  /// Constructs a spring-mass system with a fixed spring constant and given
  /// mass. Subclasses must use the protected constructor, not this one.
  /// @param[in] spring_constant_N_per_m The spring constant in N/m.
  /// @param[in] mass_Kg The actual value in Kg of the mass attached to the
  /// spring.
  /// @param[in] system_is_forced If `true`, the system has an input port for an
  /// external force. If `false`, the system has no inputs.
  SpringMassSystem(double spring_constant_N_per_m, double mass_kg,
                   bool system_is_forced = false);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit SpringMassSystem(const SpringMassSystem<U>&);

  // Provide methods specific to this System.

  /// Returns the input port to the externally applied force.
  const InputPort<T>& get_force_port() const;

  /// Returns the spring constant k that was provided at construction, in N/m.
  double get_spring_constant() const { return spring_constant_N_per_m_; }

  /// Returns the mass m that was provided at construction, in kg.
  double get_mass() const { return mass_kg_; }

  /// Returns true iff the system is forced.
  bool get_system_is_forced() const { return system_is_forced_; }

  /// Gets the current position of the mass in the given Context.
  T get_position(const Context<T>& context) const {
    return get_state(context).get_position();
  }

  /// Gets the current velocity of the mass in the given Context.
  T get_velocity(const Context<T>& context) const {
    return get_state(context).get_velocity();
  }

  /// @returns the external driving force to the system.
  T get_input_force(const Context<T>& context) const {
    T external_force = 0;
    DRAKE_ASSERT(system_is_forced_ == (context.num_input_ports() == 1));
    if (system_is_forced_) {
      external_force = get_force_port().Eval(context)[0];
    }
    return external_force;
  }

  /// Gets the current value of the conservative power integral in the given
  /// Context.
  T get_conservative_work(const Context<T>& context) const {
    return get_state(context).get_conservative_work();
  }

  /// Sets the position of the mass in the given Context.
  void set_position(Context<T>* context, const T& position) const {
    get_mutable_state(context).set_position(position);
  }

  /// Sets the velocity of the mass in the given Context.
  void set_velocity(Context<T>* context, const T& velocity) const {
    get_mutable_state(context).set_velocity(velocity);
  }

  /// Sets the initial value of the conservative power integral in the given
  /// Context.
  void set_conservative_work(Context<T>* context, const T& energy) const {
    get_mutable_state(context).set_conservative_work(energy);
  }

  /// Returns the force being applied by the spring to the mass in the given
  /// Context. This force f is given by `f = -k (x-x0)`; the spring applies the
  /// opposite force -f to the world attachment point at the other end. The
  /// force is in newtons N (kg-m/s^2).
  T EvalSpringForce(const Context<T>& context) const;

  /// Returns the potential energy currently stored in the spring in the given
  /// Context. For this linear spring, `pe = k (x-x0)^2 / 2`, so that spring
  /// force `f = -k (x-x0)` is the negative gradient of pe. The rate of change
  /// of potential energy (that is, power that adding to potential energy) is
  /// @verbatim
  ///   power_pe = d/dt pe
  ///            = k (x-x0) v
  ///            = -f v.
  /// @endverbatim
  /// Energy is in joules J (N-m).
  T DoCalcPotentialEnergy(const Context<T>& context) const override;

  /// Returns the current kinetic energy of the moving mass in the given
  /// Context. This is `ke = m v^2 / 2` for this system. The rate of change of
  /// kinetic energy (that is, power that adding to kinetic energy) is
  /// @verbatim
  ///   power_ke = d/dt ke
  ///            = m v a
  ///            = m v (f/m)
  ///            = f v
  ///            = -power_pe
  /// @endverbatim
  /// (assuming the only force is due to the spring). Energy is in joules.
  /// @see EvalSpringForce(), EvalPotentialEnergy()
  T DoCalcKineticEnergy(const Context<T>& context) const override;

  /// Returns the rate at which mechanical energy is being converted from
  /// potential energy in the spring to kinetic energy of the mass by this
  /// spring-mass system in the given Context. For this
  /// system, we have conservative power @verbatim
  ///   power_c = f v
  ///           = power_ke
  ///           = -power_pe
  /// @endverbatim
  /// This quantity is positive when the spring is accelerating the mass and
  /// negative when the spring is decelerating the mass.
  T DoCalcConservativePower(const Context<T>& context) const override;

  // TODO(sherm1) Currently this is a conservative system so there is no power
  // generated or consumed. Add some kind of dissipation and/or actuation to
  // make this more interesting. Russ suggests adding an Input which is a
  // horizontal control force on the mass.

  /// Returns power that doesn't involve the conservative spring element. (There
  /// is none in this system.)
  T DoCalcNonConservativePower(const Context<T>& context) const override;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

  /// Returns the closed-form position and velocity solution for this system
  /// from the given initial conditions.
  /// @param x0 the position of the spring at time t = 0.
  /// @param v0 the velocity of the spring at time t = 0.
  /// @param tf the time at which to return the position and velocity.
  /// @param[out] xf the position of the spring at time tf, on return.
  /// @param[out] vf the velocity of the spring at time tf, on return.
  /// @throws std::exception if xf or vf is nullptr or if the system is
  ///         forced.
  void GetClosedFormSolution(const T& x0, const T& v0, const T& tf,
                             T* xf, T* vf) const {
    using std::sqrt;
    using std::sin;
    using std::cos;

    if (!xf || !vf)
      throw std::logic_error("Passed final position/velocity is null.");
    if (system_is_forced_)
      throw std::logic_error("Can only compute closed form solution on "
                                 "unforced system");

    // d^2x/dt^2 = -kx/m
    // solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
    // where omega = sqrt(k/m)
    // ẋ(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
    // for t = 0, x(0) = c1, ẋ(0) = c2*omega

    // Setup c1 and c2 for ODE constants.
    const T omega = sqrt(get_spring_constant() / get_mass());
    const T c1 = x0;
    const T c2 = v0 / omega;
    *xf = c1*cos(omega*tf) + c2*sin(omega*tf);
    *vf = -c1*sin(omega*tf)*omega + c2*cos(omega*tf)*omega;
  }

 protected:
  /// Constructor that specifies @ref system_scalar_conversion support.
  SpringMassSystem(
      SystemScalarConverter converter,
      double spring_constant_N_per_m,
      double mass_kg,
      bool system_is_forced);

 private:
  // This is the calculator method for the output port.
  void SetOutputValues(const Context<T>& context,
                       SpringMassStateVector<T>* output) const;

  // TODO(david-german-tri): Add a cast that is dynamic_cast in Debug mode,
  // and static_cast in Release mode.

  static const SpringMassStateVector<T>& get_state(
      const ContinuousState<T>& cstate) {
    return dynamic_cast<const SpringMassStateVector<T>&>(cstate.get_vector());
  }

  static SpringMassStateVector<T>& get_mutable_state(
      ContinuousState<T>* cstate) {
    return dynamic_cast<SpringMassStateVector<T>&>(
        cstate->get_mutable_vector());
  }

  static const SpringMassStateVector<T>& get_state(const Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  static SpringMassStateVector<T>& get_mutable_state(Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  const double spring_constant_N_per_m_{};
  const double mass_kg_{};
  const bool system_is_forced_{false};
};

}  // namespace systems
}  // namespace drake
