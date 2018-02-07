#pragma once

#include <memory>

#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {

/// This method makes a MultibodyPlant model for a Kuka Iiwa arm as specified
/// in the file kuka_iiwa_robot.urdf contained in this same directory.
/// Links can be accessed by their name "iiwa_link_1" (base) through
/// "iiwa_link_7" (end effector). The "world" body can be accessed with
/// MultibodyPlant::get_world_body().
/// Joints can be accessed by their name "iiwa_joint_1" (from the base) through
/// "iiwa_joint_7" (to the end effector).
std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeKukaIiwaPlant();

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
