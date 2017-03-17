#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// The state of a one-dimensional spring-mass system, consisting of the
/// position and velocity of the mass, in meters and meters/s.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
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
  SpringMassStateVector<T>* DoClone() const override;
};

/// A model of a one-dimensional spring-mass system.
///
/// @verbatim
/// |-----\/\/ k /\/\----( m )  +x
/// @endverbatim
/// Units are MKS (meters-kilograms-seconds).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - const T&
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
///
/// @ingroup rigid_body_systems
template <typename T>
class SpringMassSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringMassSystem)

  /// Construct a spring-mass system with a fixed spring constant and given
  /// mass.
  /// @param[in] name The name of the system.
  /// @param[in] spring_constant_N_per_m The spring constant in N/m.
  /// @param[in] mass_Kg The actual value in Kg of the mass attached to the
  /// spring.
  /// @param[in] system_is_forced If `true`, the system has an input port for an
  /// external force. If `false`, the system has no inputs.
  SpringMassSystem(const T& spring_constant_N_per_m, const T& mass_kg,
                   bool system_is_forced = false);

  using MyContext = Context<T>;
  using MyContinuousState = ContinuousState<T>;
  using MyOutput = SystemOutput<T>;

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  // Provide methods specific to this System.

  /// Returns the input port to the externally applied force.
  const InputPortDescriptor<T>& get_force_port() const;

  /// Returns the port to output state.
  const OutputPortDescriptor<T>& get_output_port() const;

  /// Returns the spring constant k that was provided at construction, in N/m.
  const T& get_spring_constant() const { return spring_constant_N_per_m_; }

  /// Returns the mass m that was provided at construction, in kg.
  const T& get_mass() const { return mass_kg_; }

  /// Gets the current position of the mass in the given Context.
  T get_position(const MyContext& context) const {
    return get_state(context).get_position();
  }

  /// Gets the current velocity of the mass in the given Context.
  T get_velocity(const MyContext& context) const {
    return get_state(context).get_velocity();
  }

  /// @returns the external driving force to the system.
  T get_input_force(const MyContext& context) const {
    T external_force = 0;
    DRAKE_ASSERT(system_is_forced_ == (context.get_num_input_ports() == 1));
    if (system_is_forced_) {
      external_force = this->EvalVectorInput(context, 0)->GetAtIndex(0);
    }
    return external_force;
  }

  /// Gets the current value of the conservative power integral in the given
  /// Context.
  T get_conservative_work(const MyContext& context) const {
    return get_state(context).get_conservative_work();
  }

  /// Sets the position of the mass in the given Context.
  void set_position(MyContext* context, const T& position) const {
    get_mutable_state(context)->set_position(position);
  }

  /// Sets the velocity of the mass in the given Context.
  void set_velocity(MyContext* context, const T& velocity) const {
    get_mutable_state(context)->set_velocity(velocity);
  }

  /// Sets the initial value of the conservative power integral in the given
  /// Context.
  void set_conservative_work(MyContext* context, const T& energy) const {
    get_mutable_state(context)->set_conservative_work(energy);
  }

  /// Returns the force being applied by the spring to the mass in the given
  /// Context. This force f is given by `f = -k (x-x0)`; the spring applies the
  /// opposite force -f to the world attachment point at the other end. The
  /// force is in newtons N (kg-m/s^2).
  T EvalSpringForce(const MyContext& context) const;

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
  T DoCalcPotentialEnergy(const MyContext& context) const override;

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
  T DoCalcKineticEnergy(const MyContext& context) const override;

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
  T DoCalcConservativePower(const MyContext& context) const override;

  // TODO(sherm1) Currently this is a conservative system so there is no power
  // generated or consumed. Add some kind of dissipation and/or actuation to
  // make this more interesting. Russ suggests adding an Input which is a
  // horizontal control force on the mass.

  /// Returns power that doesn't involve the conservative spring element. (There
  /// is none in this system.)
  T DoCalcNonConservativePower(const MyContext& context) const override;

  // System<T> overrides.
  void DoCalcOutput(const MyContext& context, MyOutput* output) const override;
  void DoCalcTimeDerivatives(const MyContext& context,
                             MyContinuousState* derivatives) const override;

 private:
  // TODO(david-german-tri): Add a cast that is dynamic_cast in Debug mode,
  // and static_cast in Release mode.

  static const SpringMassStateVector<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const SpringMassStateVector<T>&>(cstate.get_vector());
  }

  static SpringMassStateVector<T>* get_mutable_state(
      MyContinuousState* cstate) {
    return dynamic_cast<SpringMassStateVector<T>*>(
        cstate->get_mutable_vector());
  }

  static const SpringMassStateVector<T>& get_output(const MyOutput& output) {
    return dynamic_cast<const SpringMassStateVector<T>&>(
        *output.get_vector_data(0));
  }

  static SpringMassStateVector<T>* get_mutable_output(MyOutput* output) {
    return dynamic_cast<SpringMassStateVector<T>*>(
        output->GetMutableVectorData(0));
  }

  static const SpringMassStateVector<T>& get_state(const MyContext& context) {
    return get_state(*context.get_continuous_state());
  }

  static SpringMassStateVector<T>* get_mutable_state(MyContext* context) {
    return get_mutable_state(context->get_mutable_continuous_state());
  }

  const T spring_constant_N_per_m_{};
  const T mass_kg_{};
  const bool system_is_forced_{false};
};

}  // namespace systems
}  // namespace drake
