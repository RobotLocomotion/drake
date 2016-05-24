#pragma once

#include <memory>
#include <string>

#include "drake/drakeSpringMassSystem_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_system_interface.h"
#include "drake/systems/framework/state_vector_interface.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace examples {

/// The state of a one-dimensional spring-mass system, consisting of the
/// position and velocity of the mass, in meters. Because it implements both
/// VectorInterface and StateVectorInterface, it can be used as both a state
/// vector and an I/O port.
class DRAKESPRINGMASSSYSTEM_EXPORT SpringMassStateVector
    : public systems::BasicVector<double>,
      public systems::StateVectorInterface<double> {
 public:
  /// @param initial_position The position of the mass in meters.
  /// @param initial_velocity The velocity of the mass in meters / second.
  SpringMassStateVector(double initial_position, double initial_velocity);
  ~SpringMassStateVector() override;

  size_t size() const override;
  const double GetAtIndex(size_t index) const override;
  void SetAtIndex(size_t index, const double& value) override;
  void SetFromVector(
      const Eigen::VectorBlock<systems::VectorX<double>>& value) override;

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
/// |-----\/\/ k /\/\----( m )  +x
///
class DRAKESPRINGMASSSYSTEM_EXPORT SpringMassSystem
    : public systems::ContinuousSystemInterface<double> {
 public:
  SpringMassSystem(const std::string& name, double spring_constant_N_per_m,
                   double mass_kg);

  ~SpringMassSystem() override;

  std::string get_name() const override;

  /// Allocates a state of type SpringMassStateVector.
  /// Allocates no input ports.
  std::unique_ptr<systems::Context<double>> CreateDefaultContext()
      const override;

  /// Allocates a single output port of type SpringMassStateVector.
  std::unique_ptr<systems::SystemOutput<double>> CreateDefaultOutput()
      const override;

  /// Allocates state derivatives of type SpringMassStateVector.
  std::unique_ptr<systems::StateVectorInterface<double>>
  CreateDefaultStateDerivatives() const override;

  void Output(const systems::Context<double>& context,
              systems::SystemOutput<double>* output) const override;

  void Dynamics(
      const systems::Context<double>& context,
      systems::StateVectorInterface<double>* derivatives) const override;

  void MapVelocityToConfigurationDerivatives(
      const systems::Context<double>& context,
      systems::StateVectorInterface<double>* derivatives) const override;

 private:
  const std::string name_;
  const double spring_constant_N_per_m_;
  const double mass_kg_;
};

}  // namespace examples
}  // namespace drake
