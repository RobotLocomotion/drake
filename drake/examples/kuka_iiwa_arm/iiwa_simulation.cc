#include "iiwa_simulation.h"

#include "drake/systems/LCMSystem.h"
#include "drake/Path.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using drake::RigidBodySystem;

std::shared_ptr<RigidBodySystem> CreateKukaIiwaSystem(void) {
  // Instantiates a rigid body system and adds the robot arm to it.
  auto rigid_body_system = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  rigid_body_system->addRobotFromFile(
      drake::getDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      DrakeJoint::FIXED);

  // Sets some simulation parameters.
  rigid_body_system->penetration_stiffness = 3000.0;
  rigid_body_system->penetration_damping = 0;

  // Adds the ground.
  double kBoxWidth = 3;
  double kBoxDepth = 0.2;
  DrakeShapes::Box geom(Eigen::Vector3d(kBoxWidth, kBoxWidth, kBoxDepth));
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0,
      -kBoxDepth / 2.0;  // top of the box is at z = 0

  const std::shared_ptr<RigidBodyTree>& tree =
      rigid_body_system->getRigidBodyTree();

  RigidBody& world = tree->world();
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758,
      1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  tree->addCollisionElement(
      RigidBodyCollisionElement(geom, T_element_to_link, &world), world,
      "terrain");
  tree->updateStaticCollisionElements();

  return rigid_body_system;
}

drake::SimulationOptions SetupSimulation(void) {
  // Specifies the simulation options.
  drake::SimulationOptions options;
  options.realtime_factor = 0;  // As fast as possible.
  options.initial_step_size = 0.002;

  // Prevents exception from being thrown when simulation runs slower than real
  // time, which it most likely will given the small step size.
  options.warn_real_time_violation = true;
  return (options);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
