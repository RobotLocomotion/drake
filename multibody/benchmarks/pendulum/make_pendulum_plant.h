#pragma once

#include <limits>
#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"

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
  ///   Value of the mass of the pendulum's point mass [kg].
  /// @param length
  ///   Length of the massless rod connecting the point mass to the world [m].
  /// @param damping
  ///   The joint's damping in N⋅m⋅s.
  /// @param gravity
  ///   Gravitational constant (m/s²).
  PendulumParameters(
      double mass = 1.0,
      double length = 0.5,
      double damping = 0.1,
      double gravity = 9.81) :
      mass_(mass),
      length_(length),
      damping_(damping),
      g_(gravity) {}

  // getters for pendulum parameters
  double m() const { return mass_; }
  double l() const { return length_; }
  double damping() const { return damping_; }
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
      damping_{nan()},    // Damping in N⋅m⋅s.
      g_{nan()};          // In m/s².

  // Modeling elements' names.
  std::string body_name_{"PointMass"};
  std::string pin_joint_name_{"PinJoint"};
  std::string actuator_name_{"PinJointActuator"};
};

/// This method makes a MultibodyPlant model of an idealized pendulum with a
/// point mass at the end of a massless rigid rod.
/// The pendulum oscillates in the x-z plane with its revolute axis coincident
/// with the y-axis. Gravity points downwards in the -z axis direction.
///
/// The parameters of the plant are:
///
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
/// @param scene_graph
///   If a SceneGraph is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for visualization.
///   If this argument is omitted, no geometry will be registered.
std::unique_ptr<MultibodyPlant<double>> MakePendulumPlant(
    const PendulumParameters& default_parameters = PendulumParameters(),
    geometry::SceneGraph<double>* scene_graph = nullptr);

}  // namespace pendulum
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
