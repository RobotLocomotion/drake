#pragma once

#include <memory>
#include <string>

#include "drake/drakeSpringMassSystem_export.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_system.h"
#include "drake/systems/framework/state_vector_interface.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace examples {

/// The state of a one-dimensional spring-mass system, consisting of the
/// position and velocity of the mass, in meters.
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
  void set_position(double v);

  /// Returns the velocity of the mass in meters per second.
  double get_velocity() const;

  /// Sets the velocity of the mass in meters per second.
  void set_velocity(double v);
};

/// The output of a one-dimensional spring-mass system, consisting of the
/// position and velocity of the mass, in meters.
class DRAKESPRINGMASSSYSTEM_EXPORT SpringMassOutputVector
    : public systems::BasicVector<double> {
 public:
  SpringMassOutputVector();
  ~SpringMassOutputVector() override;

  /// Returns the position of the mass in meters, where zero is the point
  /// where the spring exerts no force.
  double get_position() const;

  /// Sets the position of the mass in meters.
  void set_position(double v);

  /// Returns the velocity of the mass in meters per second.
  double get_velocity() const;

  /// Sets the velocity of the mass in meters per second.
  void set_velocity(double v);
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
  SpringMassSystem(const std::string& name, double spring_constant_N_per_m,
                   double mass_kg);

  using MyContext = systems::Context<double>;
  using MyContinuousState = systems::ContinuousState<double>;
  using MyOutput = systems::SystemOutput<double>;

  // Provide methods specific to this System.

  /// Get the current position of the mass in the given Context.
  double get_position(const MyContext& context) const {
    return get_state(context).get_position();
  }

  /// Get the current velocity of the mass in the given Context.
  double get_velocity(const MyContext& context) const {
    return get_state(context).get_velocity();
  }

  /// Set the position of the mass in the given Context.
  void set_position(MyContext* context, double position) const {
    get_mutable_state(context)->set_position(position);
  }

  /// Set the velocity of the mass in the given Context.
  void set_velocity(MyContext* context, double velocity) const {
    get_mutable_state(context)->set_velocity(velocity);
  }

  /** Return the force being applied by the spring to the mass in the given
  Context. This force f is given by `f = -k (x-x0)`; the spring applies the
  opposite force -f to the world attachment point at the other end. The force is
  in newtons N (kg-m/s^2). **/
  double GetSpringForce(const MyContext& context) const;

  /** Return the potential energy currently stored in the spring in the given
  Context. For this linear spring, `pe = k (x-x0)^2 / 2`, so that spring force
  `f = -k (x-x0)` is the negative gradient of pe. Power due to potential energy
  will then be @verbatim
    power_pe = d/dt pe
             = k (x-x0) v
             = -f v.
  @endverbatim
  Energy is in joules J (N-m).

  TODO(david-german-tri) These energy and power methods ought to be part of the
  system interface so that everyone has to think about them. System diagrams
  should be able to implement them by summing over contained Systems. There
  should be an option for whether to allocate a "misc" continuous state variable
  z to integrate power into work so that we can check for conservation of energy
  during a simulation. **/
  double GetPotentialEnergy(const MyContext& context) const;

  /** Return the current kinetic energy of the moving mass in the given Context.
  This is `ke = m v^2 / 2` for this system. The rate of change of kinetic energy
  is @verbatim
    d/dt ke = m v a
            = m v (f/m)
            = f v
            = -power_pe
  @endverbatim
  (assuming the only force is due to the spring). Energy is in joules.
  @see GetSpringForce(), GetPotentialEnergy() **/
  double GetKineticEnergy(const MyContext& context) const;

  /** Return the rate at which mechanical energy is being generated (positive)
  or dissipated (negative) *other than* by conversion between potential and
  kinetic energy (in the given Context). Integrating this quantity yields work
  W, and the total energy `E=PE+KE-W` should be conserved by any
  physically-correct model, to within integration accuracy of W. Power is in
  watts (J/s). (Watts are abbreviated W but not to be confused with work!)

  TODO(sherm1) Currently this is a conservative system so there is no power
  generated or consumed. Add some kind of dissipation and/or actuation to make
  this more interesting. **/
  double GetPower(const MyContext& context) const;


  // Implement base class methods.

  ~SpringMassSystem() override;

  std::string get_name() const override;

  /// Allocates a state of type SpringMassStateVector.
  /// Allocates no input ports.
  std::unique_ptr<MyContext> CreateDefaultContext() const override;

  /// Allocates a single output port of type SpringMassStateVector.
  std::unique_ptr<MyOutput> AllocateOutput() const override;

  /// Allocates state derivatives of type SpringMassStateVector.
  std::unique_ptr<MyContinuousState> AllocateDerivatives() const override;

  void GetOutput(const MyContext& context, MyOutput* output) const override;

  void GetDerivatives(const MyContext& context,
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
        *output.ports[0].vector_output);
  }

  static SpringMassOutputVector* get_mutable_output(MyOutput* output) {
    return dynamic_cast<SpringMassOutputVector*>(
        output->ports[0].vector_output.get());
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
