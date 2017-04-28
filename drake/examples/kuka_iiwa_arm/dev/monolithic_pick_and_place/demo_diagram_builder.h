#pragma once

#include <memory>
#include <utility>

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/gripper_action.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/iiwa_move.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/state_machine_system.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/robot_plan_interpolator.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {

namespace examples {

namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

// The `z` coordinate of the top of the table in the world frame.
// The quantity 0.736 is the `z` coordinate of the frame associated with the
// 'surface' collision element in the SDF. This element uses a box of height
// 0.057m thus giving the surface height (`z`) in world coordinates as
// 0.736 + 0.057 / 2.
const double kTableTopZInWorld = 0.736 + 0.057 / 2;

// Coordinates for kRobotBase originally from iiwa_world_demo.cc.
// The intention is to center the robot on the table.
const Eigen::Vector3d kRobotBase(-0.243716, -0.625087, kTableTopZInWorld);

// TODO(naveen): refactor this to reduce duplicate code.
/**
 * A method for building a `systems::RigidBodyPlant` for the pick-and-place
 * demo.
 *
 * @tparam T  The vector element type, which must be a valid Eigen scalar.
 * @param iiwa_instance A pointer to the ModelInstanceInfo object to store
 * information on the IIWA (robot) within the constructed
 * `systems::RigidBodyPlant`.
 * @param wsg_instance A pointer to the ModelInstanceInfo object to store
 * information on the Schunk WSG (gripper) within the constructed
 * `systems::RigidBodyPlant`.
 * @param box_instance A pointer to the ModelInstanceInfo object to store
 * information on the box (target for manipulation) within the constructed
 * `systems::RigidBodyPlant`.
 * @param chosen_box An integer from 1-3 to indicate which of the 3 possible
 * boxes are to be added into this tree. The number 1 is the "small" sized
 * box, 2 is the "medium, and 3 is the large). The corresponding models can
 * be found in the /models folder.
 * @param box_position The position of the target box in world coordinates
 * as a Vector3 object.
 * @param box_orientation The orientation of the target box in RPY
 * parameterization as a Vector3 object.
 * @return A `std::unique_ptr` to the constructed `systems::RigidBodyPlant`.
 */
template <typename T>
std::unique_ptr<systems::RigidBodyPlant<T>> BuildCombinedPlant(
    ModelInstanceInfo<T>* iiwa_instance, ModelInstanceInfo<T>* wsg_instance,
    ModelInstanceInfo<T>* box_instance, const int chosen_box,
    Eigen::Vector3d box_position, Eigen::Vector3d box_orientation) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel("table",
                           "/examples/kuka_iiwa_arm/models/table/"
                           "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel(
      "box_small",
      "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf");

  tree_builder->StoreModel("box_medium",
                           "/examples/kuka_iiwa_arm/models/objects/"
                           "block_for_pick_and_place_mid_size.urdf");

  tree_builder->StoreModel("box_large",
                           "/examples/kuka_iiwa_arm/models/objects/"
                           "block_for_pick_and_place_large_size.urdf");
  tree_builder->StoreModel("wsg",
                           "/examples/schunk_wsg/models/schunk_wsg_50.sdf");

  // Builds a world with two fixed tables.  A box is placed one on
  // table, and the iiwa arm is fixed to the other.
  tree_builder->AddFixedModelInstance("table",
                                      Eigen::Vector3d::Zero() /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);
  tree_builder->AddFixedModelInstance("table",
                                      Eigen::Vector3d(0.8, 0, 0) /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);
  tree_builder->AddFixedModelInstance("table",
                                      Eigen::Vector3d(0, 0.85, 0) /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);

  tree_builder->AddGround();
  // Chooses an appropriate box.
  int box_id = 0;
  int iiwa_id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(iiwa_id);
  switch (chosen_box) {
    case 1:
      box_id = tree_builder->AddFloatingModelInstance("box_small", box_position,
                                                      box_orientation);
      break;
    case 2:
      box_id = tree_builder->AddFloatingModelInstance(
          "box_medium", box_position, box_orientation);
      break;
    case 3:
      box_id = tree_builder->AddFloatingModelInstance("box_large", box_position,
                                                      box_orientation);
      break;
    default: DRAKE_ABORT_MSG("Chosen box should be between 0 and 4.");
      break;
  }
  *box_instance = tree_builder->get_model_info_for_instance(box_id);

  int wsg_id = tree_builder->AddModelInstanceToFrame(
      "wsg", Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(wsg_id);

  auto plant =
      std::make_unique<systems::RigidBodyPlant<T>>(tree_builder->Build());
  return (std::move(plant));
}

/**
 * A `systems::Diagram` that encapsulates a `PickAndPlaceStateMachineSystem`
 * and a `IiwaMove` and a `GripperAction`. This `systems::Diagram` serves as
 * the high-level logic to execute a pick-and-place demo. All of the input
 * and output ports carry abstract data with custom lcm messages.
 *
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * Instantiated templates for the following kinds of T's are provided:
 * - double
 */
template <typename T>
class StateMachineAndPrimitives : public systems::Diagram<T> {
 public:
  StateMachineAndPrimitives(const RigidBodyTree<T>& iiwa_tree,
                            const double iiwa_action_primitive_rate = 0.01,
                            const double wsg_action_primitive_rate = 0.01);

  const systems::InputPortDescriptor<T>& get_input_port_iiwa_robot_state()
      const {
    return this->get_input_port(input_port_iiwa_robot_state_t_);
  }

  const systems::InputPortDescriptor<T>& get_input_port_box_robot_state()
      const {
    return this->get_input_port(input_port_box_robot_state_t_);
  }

  const systems::InputPortDescriptor<T>& get_input_port_wsg_status() const {
    return this->get_input_port(input_port_wsg_status_);
  }

  const systems::OutputPortDescriptor<T>& get_output_port_iiwa_command() const {
    return this->get_output_port(output_port_iiwa_command_);
  }

  const systems::OutputPortDescriptor<T>& get_output_port_wsg_command() const {
    return this->get_output_port(output_port_wsg_command_);
  }

 protected:
  IiwaMove* iiwa_move_{nullptr};
  GripperAction* gripper_action_{nullptr};
  PickAndPlaceStateMachineSystem* pick_and_place_state_machine_{nullptr};

  int input_port_iiwa_robot_state_t_{-1};
  int input_port_box_robot_state_t_{-1};
  int input_port_wsg_status_{-1};
  int output_port_iiwa_command_{-1};
  int output_port_wsg_command_{-1};
};

/**
 * A `systems::Diagram` that encapsulates a
 * `IiwaAndWsgPlanGeneratorsEstimatorsAndVisualizer`, a
 * `systems::DrakeVisualizer`, `RobotPlanInterpolator`,
 * `SchunkWsgTrajectoryGenerator` and  a `SchunkWsgStatusSender`. This
 * `systems::Diagram`system serves as the "low-level" logic needed to execute
 * a pick-and-place demo. All of the input and output ports carry abstract data
 * with custom lcm messages.
 *
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * Instantiated templates for the following kinds of T's are provided:
 * - double
 */
template <typename T>
class
IiwaWsgPlantGeneratorsEstimatorsAndVisualizer
    : public systems::Diagram<T> {
 public:
  /** Constructs the IiwaWsgPlantGeneratorsEstimatorsAndVisualizer.
   * @param lcm : A reference to the lcm object to be passed onto the
   * Visualizer
   * @param update_interval : The update interval of the unrestricted update of
   * `RobotPlanInterpolator`. This should be smaller than that of
   * components
   * commanding new plans.
   * @param chosen_box : The choice of which box ...
   */
  IiwaWsgPlantGeneratorsEstimatorsAndVisualizer(
      lcm::DrakeLcm* lcm, const int chosen_box = 1,
      const double update_interval = 0.001,
      Eigen::Vector3d box_position = Vector3<double>(1 + -0.43, -0.65,
                                                     kTableTopZInWorld + 0.1),
      Eigen::Vector3d box_orientation = Vector3<double>(0, 0, 1));
  const systems::InputPortDescriptor<T>& get_input_port_iiwa_plan() const {
    return this->get_input_port(input_port_iiwa_plan_);
  }

  const systems::InputPortDescriptor<T>& get_input_port_wsg_plan() const {
    return this->get_input_port(input_port_wsg_plan_);
  }

  const systems::OutputPortDescriptor<T>& get_output_port_wsg_status() const {
    return this->get_output_port(output_port_wsg_status_);
  }

  const systems::OutputPortDescriptor<T>&
  get_output_port_iiwa_robot_state_est_msg() const {
    return this->get_output_port(output_port_iiwa_robot_state_msg_);
  }

  const systems::OutputPortDescriptor<T>&
  get_output_port_box_robot_state_est_msg() const {
    return this->get_output_port(output_port_box_robot_state_msg_);
  }

  /**
   * Makes a plan for the iiwa to hold at the measured joint
   * configuration @p q0.  This function needs to be explicitly called
   * before any simulation. Otherwise this aborts in CalcOutput().
   */
  void InitializeIiwaPlan(const VectorX<T>& q0,
                          systems::Context<T>* context) const;

 private:
  IiwaAndWsgPlantWithStateEstimator<T>* plant_{nullptr};
  schunk_wsg::SchunkWsgStatusSender* wsg_status_sender_{nullptr};
  systems::DrakeVisualizer* drake_visualizer_{nullptr};
  RobotPlanInterpolator* iiwa_trajectory_generator_{nullptr};
  schunk_wsg::SchunkWsgTrajectoryGenerator* wsg_trajectory_generator_{nullptr};

  int input_port_iiwa_plan_{-1};
  int input_port_wsg_plan_{-1};
  int output_port_wsg_status_{-1};
  int output_port_iiwa_robot_state_msg_{-1};
  int output_port_box_robot_state_msg_{-1};
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
