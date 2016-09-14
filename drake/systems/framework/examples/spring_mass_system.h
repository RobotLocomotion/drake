#pragma once

#include <memory>
#include <string>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// The state of a one-dimensional spring-mass system, consisting of the
/// position and velocity of the mass, in meters and meters/s.

class DRAKESYSTEMFRAMEWORK_EXPORT SpringMassStateVector
    : public BasicVector<double> {
 public:
  /// @param initial_position The position of the mass in meters.
  /// @param initial_velocity The velocity of the mass in meters / second.
  SpringMassStateVector(double initial_position, double initial_velocity);
  /// Creates a state with position and velocity set to zero.
  SpringMassStateVector();
  ~SpringMassStateVector() override;

  /// Returns the position of the mass in meters, where zero is the point
  /// where the spring exerts no force.
  double get_position() const;

  /// Sets the position of the mass in meters.
  void set_position(double q);

  /// Returns the velocity of the mass in meters per second.
  double get_velocity() const;

  /// Sets the velocity of the mass in meters per second.
  void set_velocity(double v);

  /// Returns the integral of conservative power, in watts.
  double get_conservative_work() const;

  /// Initialize the conservative work integral to a given value.
  void set_conservative_work(double e);

 private:
  SpringMassStateVector* DoClone() const override;
};

/// A model of a one-dimensional spring-mass system.
///
/// @verbatim
/// |-----\/\/ k /\/\----( m )  +x
/// @endverbatim
///
/// Units are MKS (meters-kilograms-seconds).
/// @ingroup systems

class DRAKESYSTEMFRAMEWORK_EXPORT SpringMassSystem : public System<double> {
 public:
  /// Construct a spring-mass system with a fixed spring constant and given
  /// mass.
  /// @param[in] name The name of the system.
  /// @param[in] spring_constant_N_per_m The spring constant in N/m.
  /// @param[in] mass_Kg The actual value in Kg of the mass attached to the
  /// spring.
  /// @param[in] system_is_forced If `true`, the system has an input port for an
  /// external force. If `false`, the system has no inputs.
  SpringMassSystem(double spring_constant_N_per_m, double mass_kg,
                   bool system_is_forced = false);

  using MyContext = Context<double>;
  using MyContinuousState = ContinuousState<double>;
  using MyOutput = SystemOutput<double>;

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  // Provide methods specific to this System.

  /// Returns the spring constant k that was provided at construction, in N/m.
  double get_spring_constant() const { return spring_constant_N_per_m_; }

  /// Returns the mass m that was provided at construction, in kg.
  double get_mass() const { return mass_kg_; }

  /// Gets the current position of the mass in the given Context.
  double get_position(const MyContext& context) const {
    return get_state(context).get_position();
  }

  /// Gets the current velocity of the mass in the given Context.
  double get_velocity(const MyContext& context) const {
    return get_state(context).get_velocity();
  }

  /// @returns the external driving force to the system.
  double get_input_force(const MyContext& context) const {
    double external_force = 0;
    DRAKE_ASSERT(system_is_forced_ == (context.get_num_input_ports() == 1));
    if (system_is_forced_) {
      external_force = this->EvalVectorInput(context, 0)->GetAtIndex(0);
    }
    return external_force;
  }

  /// Gets the current value of the conservative power integral in the given
  /// Context.
  double get_conservative_work(const MyContext& context) const {
    return get_state(context).get_conservative_work();
  }

  /// Sets the position of the mass in the given Context.
  void set_position(MyContext* context, double position) const {
    get_mutable_state(context)->set_position(position);
  }

  /// Sets the velocity of the mass in the given Context.
  void set_velocity(MyContext* context, double velocity) const {
    get_mutable_state(context)->set_velocity(velocity);
  }

  /// Sets the initial value of the conservative power integral in the given
  /// Context.
  void set_conservative_work(MyContext* context, double energy) const {
    get_mutable_state(context)->set_conservative_work(energy);
  }

  /// Returns the force being applied by the spring to the mass in the given
  /// Context. This force f is given by `f = -k (x-x0)`; the spring applies the
  /// opposite force -f to the world attachment point at the other end. The
  /// force is in newtons N (kg-m/s^2).
  double EvalSpringForce(const MyContext& context) const;

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
  double EvalPotentialEnergy(const MyContext& context) const override;

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
  double EvalKineticEnergy(const MyContext& context) const override;

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
  double EvalConservativePower(const MyContext& context) const override;

  // TODO(sherm1) Currently this is a conservative system so there is no power
  // generated or consumed. Add some kind of dissipation and/or actuation to
  // make this more interesting. Russ suggests adding an Input which is a
  // horizontal control force on the mass.

  /// Returns power that doesn't involve the conservative spring element. (There
  /// is none in this system.)
  double EvalNonConservativePower(const MyContext& context) const override;

  // Implement base class methods.

  /// Allocates a state of type SpringMassStateVector.
  /// Allocates no input ports.
  std::unique_ptr<MyContext> CreateDefaultContext() const override;

  /// Allocates a single output port of type SpringMassStateVector.
  std::unique_ptr<MyOutput> AllocateOutput(
      const MyContext& context) const override;

  /// Allocates state derivatives of type SpringMassStateVector.
  std::unique_ptr<MyContinuousState> AllocateTimeDerivatives() const override;

  void EvalOutput(const MyContext& context, MyOutput* output) const override;

  void EvalTimeDerivatives(const MyContext& context,
                           MyContinuousState* derivatives) const override;

 private:
  // TODO(david-german-tri): Add a cast that is dynamic_cast in Debug mode,
  // and static_cast in Release mode.

  static const SpringMassStateVector& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const SpringMassStateVector&>(cstate.get_state());
  }

  static SpringMassStateVector* get_mutable_state(MyContinuousState* cstate) {
    return dynamic_cast<SpringMassStateVector*>(cstate->get_mutable_state());
  }

  static const SpringMassStateVector& get_output(const MyOutput& output) {
    return dynamic_cast<const SpringMassStateVector&>(
        *output.get_vector_data(0));
  }

  static SpringMassStateVector* get_mutable_output(MyOutput* output) {
    return dynamic_cast<SpringMassStateVector*>(
        output->GetMutableVectorData(0));
  }

  static const SpringMassStateVector& get_state(const MyContext& context) {
    return get_state(*context.get_state().continuous_state);
  }

  static SpringMassStateVector* get_mutable_state(MyContext* context) {
    return get_mutable_state(
        context->get_mutable_state()->continuous_state.get());
  }

  const double spring_constant_N_per_m_{};
  const double mass_kg_{};
  const bool system_is_forced_{false};
};

}  // namespace systems
}  // namespace drake
