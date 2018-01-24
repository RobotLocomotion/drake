#pragma once

#include <limits>
#include <memory>

#include "drake/common/drake_optional.h"
#include "drake/examples/multibody/acrobot/gen/acrobot_state.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/force_element.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace examples {
namespace multibody {
namespace acrobot {

class AcrobotDefaultParameters {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AcrobotDefaultParameters)

  AcrobotDefaultParameters(
      double m1 = 1.0,
      double m2 = 1.0,
      double l1 = 1.0,
      double l2 = 2.0,
      double lc1 = 0.5,
      double lc2 = 1.0,
      double Ic1 = .083,
      double Ic2 = .33,
      double b1 = 0.1,
      double b2 = 0.1,
      double g = 9.81) :
      m1_(m1),
      m2_(m2),
      l1_(l1),
      l2_(l2),
      lc1_(lc1),
      lc2_(lc2),
      Ic1_(Ic1),
      Ic2_(Ic2),
      b1_(b1),
      b2_(b2),
      g_(g) {}

  // getters for robot parameters
  double m1() const { return m1_; }
  double m2() const { return m2_; }
  double l1() const { return l1_; }
  double l2() const { return l2_; }
  double lc1() const { return lc1_; }
  double lc2() const { return lc2_; }
  double Ic1() const { return Ic1_; }
  double Ic2() const { return Ic2_; }
  double b1() const { return b1_; }
  double b2() const { return b2_; }
  double g() const { return g_; }

 private:
  // Helper method for NaN initialization.
  static constexpr double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // The physical parameters of the model. They are initialized with NaN for a
  // quick detection of uninitialized values.
  double m1_{nan()}, m2_{nan()},  // In kilograms.
      l1_{nan()}, l2_{nan()},     // In meters.
      lc1_{nan()}, lc2_{nan()},   // In meters.
      Ic1_{nan()}, Ic2_{nan()},   // In kg⋅m².
      b1_{nan()}, b2_{nan()},     // In N⋅m⋅s.
      g_{nan()};                  // In m/s².
};

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
/// This plant is modeled using a MultibodyTree.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @param m1 Mass of link 1 (kg).
/// @param m2 Mass of link 2 (kg).
/// @param l1 Length of link 1 (m).
/// @param l2 Length of link 2 (m).
/// @param lc1 Vertical distance from shoulder joint to center of mass of
/// link 1 (m).
/// @param lc2 Vertical distance from elbow joint to center of mass of
/// link 2 (m).
/// @param Ic1 Inertia of link 1 about the center of mass of link 1
/// (kg*m^2).
/// @param Ic2 Inertia of link 2 about the center of mass of link 2
/// (kg*m^2).
/// @param b1 Damping coefficient of the shoulder joint (kg*m^2/s).
/// @param b2 Damping coefficient of the elbow joint (kg*m^2/s).
/// @param g Gravitational constant (m/s^2).
///
/// The parameters are defaulted to values in Spong's paper (see
/// acrobot_spong_controller.cc for more details).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeAcrobotPlant(geometry::GeometrySystem<double>* geometry_system = nullptr);

}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake
