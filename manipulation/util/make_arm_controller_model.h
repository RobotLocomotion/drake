#pragma once

#include <memory>
#include <optional>

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/model_instance_info.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace internal {

/* Constructs a separate plant (different from the simulated plant!) that can be
 used to instantiate a controller system such as `InverseDynamicsController`.
 The difference between this plant and @p simulation_plant is the dynamics
 modeling error.

 The returned plant will contain an arm model loaded from @p arm_info.model_path
 and welded to the world at @p arm_info.child_frame_name at the same pose as
 @p simulation_plant. All bodies welded directly to bodies of the arm will be
 replaced with lumped masses.

 If @p gripper_info is provided, this function will use @p simulation_plant
 to measure the gripper's inertia and attach a single non-articulated rigid body
 that matches the gripper's inertia, including any bodies welded to the gripper.
 Additionally, if `grasp_frame` is present in the gripper model and welded to
 @p gripper_info.child_frame_name, a `grasp_frame` Frame will be added at the
 same pose as @p simulation_plant.

 This function requires that (a) most intermediate frames / bodies can be
 recreated by loading the URDF / SDFormat file dictated by ModelInstanceInfo and
 (b) the heuristics in ProcessModelDirectives reflect the actual construction of
 this plant.

 @pre @p arm_info.child_frame_name is welded to the world frame.
 @pre If @p gripper_info is given and `grasp_frame` exists, then `grasp_frame`
      is welded to @p gripper_info.child_frame_name. */
// TODO(eric.cousineau): Replace this with more generic (and robust) multibody
// plant subgraph (#7336).
std::unique_ptr<multibody::MultibodyPlant<double>> MakeArmControllerModel(
    const multibody::MultibodyPlant<double>& simulation_plant,
    const multibody::parsing::ModelInstanceInfo& arm_info,
    const std::optional<multibody::parsing::ModelInstanceInfo>& gripper_info =
        {});

}  // namespace internal
}  // namespace manipulation
}  // namespace drake
