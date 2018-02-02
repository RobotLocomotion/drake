#pragma once

#include <limits>
#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {

template <typename T>
class KukaIiwaPlantBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KukaIiwaPlantBuilder);

  /// Instantiate a builder to make a MultibodyPlant model of the KUKA iiwa arm
  /// as specified in this class' documentation.
  KukaIiwaPlantBuilder() {}

  /// Construct a 7-DOF Kuka iiwa robot arm (from file kuka_iiwa_robot.urdf).
  /// The robot is constructed with 7 revolute joints.
  /// @param[in] gravity Earth's gravitational acceleration in m/s².  The world
  /// z-unit vector is vertically upward.  If a gravity value of 9.8 is passed
  /// to this constructor, it means the gravity vector is directed opposite the
  /// world upward z-unit vector (which is correct -- gravity is downward).
  std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<T>>
  Build() const;

 private:
  // Helper method to add revolute joint from Body A to Body B.
  // @param[in] A     Joint's parent  body (frame Ab will be welded to A).
  // @param[in] q123A SpaceXYZ angles describing the rotation matrix relating
  //                  unit vectors Ax, Ay, Az to soon-to-be created frame Ab.
  // @param[in] xyzA  Ax, Ay, Az measures of the position from Ao to Abo.
  // @param[in] B     Joint's child body (frame Ba will be welded to B
  //                  so it is coincident with body B's frame). In other words,
  //                  joint's frame Ba on the child body will be coincident with
  //                  the outboard body frame B.
  // @param[in] revolute_unit_vector
  //   Unit vector expressed in frame Ab that characterizes a positive rotation
  //   of Ba from Ab (right-hand-rule).
  // @return RevoluteJoint from frame Ab on Body A to frame Ba on Body B.
  static const RevoluteJoint<T>& AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
      const std::string& joint_name,
      const Body<T>& A,
      const Vector3<double>& q123A, const Vector3<double>& xyzA,
      const Body<T>& B, const Vector3<double>& revolute_unit_vector,
      drake::multibody::multibody_plant::MultibodyPlant<T>* plant);


  // Model's parameters:

  // Mass of each link (in kg).
  const double massA_ = 5.76;
  const double massB_ = 6.35;
  const double massC_ = 3.5;
  const double massD_ = 3.5;
  const double massE_ = 3.5;
  const double massF_ = 1.8;
  const double massG_ = 1.2;

  // Position of each body's center of mass from body origin, expressed in body.
  // Example: For a body B with center of mass Bcm and origin Bo, p_BoBcm_B is
  // the position from Bo to Bcm, expressed in terms of Bx, By, Bz (in meters).
  const Eigen::Vector3d p_AoAcm_A_{0,     -0.03,   0.12};
  const Eigen::Vector3d p_BoBcm_B_{0.0003, 0.059,  0.042};
  const Eigen::Vector3d p_CoCcm_C_{0,      0.03,   0.13};
  const Eigen::Vector3d p_DoDcm_D_{0,      0.067,  0.034};
  const Eigen::Vector3d p_EoEcm_E_{0.0001, 0.021,  0.076};
  const Eigen::Vector3d p_FoFcm_F_{0,      0.0006, 0.0004};
  const Eigen::Vector3d p_GoGcm_G_{0,      0,      0.02};

  // Inertia matrix of each body about its center of mass, expressed in body.
  // Example: For a body B with center of mass Bcm, I_Bcm_B is B's inertia
  // matrix about Bcm, expressed in terms of Bx, By, Bz (in kg * meters^2).
  const RotationalInertia<double> I_AAcm_A_{0.033,  0.0333, 0.0123};
  const RotationalInertia<double> I_BBcm_B_{0.0305, 0.0304, 0.011};
  const RotationalInertia<double> I_CCcm_C_{0.025,  0.0238, 0.0076};
  const RotationalInertia<double> I_DDcm_D_{0.017,  0.0164, 0.006};
  const RotationalInertia<double> I_EEcm_E_{0.01,   0.0087, 0.00449};
  const RotationalInertia<double> I_FFcm_F_{0.0049, 0.0047, 0.0036};
  const RotationalInertia<double> I_GGcm_G_{0.001,  0.001,  0.001};

  // Earth's gravitational acceleration.
  double gravity_{9.81};
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
/// @param[out] geometry_system
///   If a GeometrySystem is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for visualization.
///   If this argument is omitted, no geometry will be registered.
std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeKukaIiwaPlant();

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
