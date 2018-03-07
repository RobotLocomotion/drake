#pragma once

#include <limits>
#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace pendulum {

/// This class is used to store the numerical parameters defining the model of
/// a simple pendulum with the method MakePendulumPlant().
/// Refer to this the documentation of this class's constructor for further
/// details on the parameters stored by this class and their default values.
class PendulumParameters {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PendulumParameters)

  /// Constructor used to initialize the physical parameters for a simple
  /// pendulum model.
  ///
  /// @param mass
  ///   Mass of link 1 (kg).
  /// @param m2
  ///   Mass of link 2 (kg).
  /// @param l1
  ///   Length of link 1 (m).
  /// @param l2
  ///   Length of link 2 (m).
  /// @param lc1
  ///   Vertical distance from shoulder joint to center of mass of link 1 (m).
  /// @param lc2
  ///   Vertical distance from elbow joint to center of mass of link 2 (m).
  /// @param Ic1
  ///   Inertia of link 1 about the center of mass of link 1 (kg⋅m²).
  /// @param Ic2
  ///   Inertia of link 2 about the center of mass of link 2 (kg*m^2).
  /// @param b1
  ///   Damping coefficient of the shoulder joint (N⋅m⋅s).
  /// @param b2
  ///   Damping coefficient of the elbow joint (N⋅m⋅s).
  /// @param g
  ///   Gravitational constant (m/s²).
  ///
  ///  - [Spong 1994] Spong, M.W., 1994. Swing up control of the acrobot.
  ///    In Robotics and Automation, 1994. Proceedings., 1994 IEEE International
  ///    Conference on (pp. 2356-2361). IEEE.
  PendulumParameters(
      double mass = 1.0,
      double length = 0.5,
      double gravity = 9.81) :
      mass_(mass),
      length_(length),
      g_(gravity) {}

  // getters for robot parameters
  double m() const { return mass_; }
  double l() const { return length_; }
  double g() const { return g_; }
  // Radius of the sphere used to visualize the point mass
  double point_mass_radius() const { return 0.025; }
  // Radius of the cylinder used to visualize the massless rod
  double massless_rod_radius() const { return 0.007; }

  // getters for modeling elements' names
  const std::string& body_name() const { return body_name_; }
  const std::string& pin_joint_name() const { return pin_joint_name_; }
  const std::string& actuator_name() const { return actuator_name_; }

 private:
  // Helper method for NaN initialization.
  static constexpr double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // The physical parameters of the model. They are initialized with NaN for a
  // quick detection of uninitialized values.
  double mass_{nan()},    // In kilograms.
      length_{nan()},     // In meters.
      g_{nan()};          // In m/s².

  // Modeling elements' names.
  std::string body_name_{"PointMass"};
  std::string pin_joint_name_{"PinJoint"};
  std::string actuator_name_{"PinJointActuator"};
};

/// This method makes a MultibodyPlant model of an idealized pendulum with a
/// point mass on the end of a massless rigid rod.
/// The pendulum oscillates in the x-z plane with its revolute axis coincident
/// with the y-axis. Gravity points downwards in the -z axis direction.
///
/// The parameters of the plant are:
/// - mass: the mass of the idealized point mass.
/// - length: the length of the massless rod on which the mass is suspended.
/// - gravity: the acceleration of gravity.
///
/// The simple pendulum is a canonical dynamical system as described in <a
/// href="http://underactuated.csail.mit.edu/underactuated.html?chapter=pend">
/// Chapter 2 of Underactuated Robotics</a>.
///
/// @param[in] default_parameters
///   Default parameters of the model set at construction. Refer to the
///   documentation of PendulumParameters for further details.
/// @param geometry_system
///   If a GeometrySystem is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for visualization.
///   If this argument is omitted, no geometry will be registered.
std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakePendulumPlant(
    const PendulumParameters& default_parameters,
    geometry::GeometrySystem<double>* geometry_system = nullptr);

}  // namespace pendulum
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
