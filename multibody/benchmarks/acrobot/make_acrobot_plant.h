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
namespace acrobot {

/// This class is used to store the numerical parameters defining the model of
/// an acrobot with the method MakeAcrobotPlant().
/// Refer to this the documentation of this class's constructor for further
/// details on the parameters stored by this class and their default values.
///
/// @note The default constructor initializes the parameters in accordance to
/// the `acrobot.sdf` file in this same directory. Therefore this file and
/// `acrobot.sdf` MUST be kept in sync.
class AcrobotParameters {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AcrobotParameters)

  /// Constructor used to initialize the physical parameters for an acrobot
  /// model. The parameters are defaulted to values in Spong's paper
  /// [Spong 1994].
  ///
  /// @param m1
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
  AcrobotParameters(
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
  // Radii of the cylinders used for visualization.
  // The second link is made slightly thicker so that the difference between the
  // links can be seen when aligned.
  double r1() const { return 0.035; }
  double r2() const { return 0.07; }

  // getters for modeling elements' names
  const std::string& link1_name() const { return link1_name_; }
  const std::string& link2_name() const { return link2_name_; }
  const std::string& shoulder_joint_name() const {
    return shoulder_joint_name_;
  }
  const std::string& elbow_joint_name() const {
    return elbow_joint_name_;
  }
  const std::string& actuator_name() const { return actuator_name_; }

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

  // Modeling elements' names.
  std::string link1_name_{"Link1"};
  std::string link2_name_{"Link2"};
  std::string shoulder_joint_name_{"ShoulderJoint"};
  std::string elbow_joint_name_{"ElbowJoint"};
  std::string actuator_name_{"ElbowActuator"};
};

/// This method makes a MultibodyPlant model of the Acrobot - a canonical
/// underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// @param[in] default_parameters
///   Default parameters of the model set at construction. These parameters
///   include masses, link lengths, rotational inertias, etc. Refer to the
///   documentation of AcrobotParameters for further details.
/// @param[in] finalize
///   If `true`, MultibodyPlant::Finalize() gets called on the new plant.
/// @param scene_graph
///   If a SceneGraph is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for visualization.
///   If this argument is omitted, no geometry will be registered.
std::unique_ptr<MultibodyPlant<double>>
MakeAcrobotPlant(const AcrobotParameters& default_parameters, bool finalize,
                 geometry::SceneGraph<double>* scene_graph = nullptr);

}  // namespace acrobot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
