#pragma once

#include <memory>
#include <string>

#include "drake/drakeSpringMassSystem_export.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_system.h"
#include "drake/systems/framework/state_vector.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace examples {

/// The state of a one-dimensional spring-mass system, consisting of the
/// position and velocity of the mass, in meters and meters/s.
class DRAKESPRINGMASSSYSTEM_EXPORT SpringMassStateVector
    : public systems::BasicStateVector<double> {
 public:
  /// @param initial_position The position of the mass in meters.
  /// @param initial_velocity The velocity of the mass in meters / second.
  SpringMassStateVector(double initial_position, double initial_velocity);
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

/// The output of a one-dimensional spring-mass system, consisting of the
/// position and velocity of the mass, in meters. Note that although this 
/// system tracks work done as a state variable, we are not reporting that
/// as an Output.
class DRAKESPRINGMASSSYSTEM_EXPORT SpringMassOutputVector
    : public systems::BasicVector<double> {
 public:
  SpringMassOutputVector();
  ~SpringMassOutputVector() override;

  /// Returns the position of the mass in meters, where zero is the point
  /// where the spring exerts no force.
  double get_position() const;

  /// Sets the position of the mass in meters.
  void set_position(double q);

  /// Returns the velocity of the mass in meters per second.
  double get_velocity() const;

  /// Sets the velocity of the mass in meters per second.
  void set_velocity(double v);

 private:
  SpringMassOutputVector* DoClone() const override;
};

/// A model of a one-dimensional spring-mass system.
///
/// @verbatim
/// |-----\/\/ k /\/\----( m )  +x
/// @endverbatim
///
/// Units are MKS (meters-kilograms-seconds).
class DRAKESPRINGMASSSYSTEM_EXPORT SpringMassSystem
    : public systems::ContinuousSystem<double> {
 public:
  /// Construct a spring-mass system with a fixed spring constant and given
  /// mass.
  SpringMassSystem(const std::string& name, double spring_constant_N_per_m,
                   double mass_kg);

  using MyContext = systems::Context<double>;
  using MyContinuousState = systems::ContinuousState<double>;
  using MyOutput = systems::SystemOutput<double>;

  // Provide methods specific to this System.

  /// Gets the current position of the mass in the given Context.
  double get_position(const MyContext& context) const {
    return get_state(context).get_position();
  }

  /// Gets the current velocity of the mass in the given Context.
  double get_velocity(const MyContext& context) const {
    return get_state(context).get_velocity();
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
  /// force `f = -k (x-x0)` is the negative gradient of pe. Power that is being
  /// stored as potential energy will then be @verbatim
  ///   power_pe = d/dt pe
  ///            = k (x-x0) v
  ///            = -f v.
  /// @endverbatim
  /// Energy is in joules J (N-m).
  double EvalPotentialEnergy(const MyContext& context) const override;

  /// Returns the current kinetic energy of the moving mass in the given Context.
  /// This is `ke = m v^2 / 2` for this system. The rate of change of kinetic
  /// energy is @verbatim
  ///   d/dt ke = m v a
  ///           = m v (f/m)
  ///           = f v
  ///           = -power_pe
  /// @endverbatim
  /// (assuming the only force is due to the spring). Energy is in joules.
  /// @see EvalSpringForce(), EvalPotentialEnergy()
  double EvalKineticEnergy(const MyContext& context) const override;

  /// Returns the rate at which mechanical energy is being converted from 
  /// potential energy in the spring to kinetic energy of the mass by this 
  /// spring-mass system in the given Context. For this
  /// system, we have conservative power @verbatim
  ///   power_c = f v
  ///           = -power_pe
  /// @endverbatim
  /// That quantity is positive when the spring is accelerating the mass and 
  /// negative when the spring is decelerating the mass.
  double EvalConservativePower(const MyContext& context) const override;

  // TODO(sherm1) Currently this is a conservative system so there is no power
  // generated or consumed. Add some kind of dissipation and/or actuation to
  // make this more interesting.

  /// Returns power that doesn't involve the conservative spring element. (There
  /// is none in this system.)
  double EvalNonConservativePower(const MyContext& context) const override;

  // Implement base class methods.

  ~SpringMassSystem() override;

  std::string get_name() const override;

  /// Allocates a state of type SpringMassStateVector.
  /// Allocates no input ports.
  std::unique_ptr<MyContext> CreateDefaultContext() const override;

  /// Allocates a single output port of type SpringMassStateVector.
  std::unique_ptr<MyOutput> AllocateOutput() const override;

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

  static const SpringMassOutputVector& get_output(const MyOutput& output) {
    return dynamic_cast<const SpringMassOutputVector&>(
        *output.ports[0]->get_vector_data());
  }

  static SpringMassOutputVector* get_mutable_output(MyOutput* output) {
    return dynamic_cast<SpringMassOutputVector*>(
        output->ports[0]->GetMutableVectorData());
  }

  static const SpringMassStateVector& get_state(const MyContext& context) {
    return get_state(*context.get_state().continuous_state);
  }

  static SpringMassStateVector* get_mutable_state(MyContext* context) {
    return get_mutable_state(
        context->get_mutable_state()->continuous_state.get());
  }

  const std::string name_;
  const double spring_constant_N_per_m_;
  const double mass_kg_;
};

}  // namespace examples
}  // namespace drake
