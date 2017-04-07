#pragma once

#include <memory>
#include <utility>

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/iiwa_state_feedback_plan.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::DiagramBuilder;
using lcm::DrakeLcm;

namespace examples {
using schunk_wsg::SchunkWsgTrajectoryGenerator;
using schunk_wsg::SchunkWsgStatusSender;

namespace kuka_iiwa_arm {
namespace pick_and_place {

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
template <typename T>
std::unique_ptr<systems::RigidBodyPlant<T>> BuildCombinedPlant(
    ModelInstanceInfo<T>* iiwa_instance, ModelInstanceInfo<T>* wsg_instance,
    ModelInstanceInfo<T>* box_instance) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel("table",
                           "/examples/kuka_iiwa_arm/models/table/"
                           "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel(
      "box",
      "/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf");
  tree_builder->StoreModel("wsg",
                           "/examples/schunk_wsg/models/schunk_wsg_50.sdf");

  // Build a world with two fixed tables.  A box is placed one on
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

  // Start the box slightly above the table.  If we place it at
  // the table top exactly, it may start colliding the table (which is
  // not good, as it will likely shoot off into space).
  const Eigen::Vector3d kBoxBase(1 + -0.43, -0.65, kTableTopZInWorld + 0.1);

  int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(id);
  id = tree_builder->AddFloatingModelInstance("box", kBoxBase,
                                              Vector3<double>(0, 0, 1));

  *box_instance = tree_builder->get_model_info_for_instance(id);
  id = tree_builder->AddModelInstanceToFrame(
      "wsg", Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(id);

  auto plant = std::make_unique<RigidBodyPlant<T>>(tree_builder->Build());
  return (std::move(plant));
}

template <typename T>
class IiwaWsgPlantGeneratorsEstimatorsAndVisualizer
    : public systems::Diagram<T> {
 public:
  /// Constructs the IiwaWsgPlantGeneratorsEstimatorsAndVisualizer.
  /// This Diagram encapsulses a IiwaAndWsgPlantWithStateEstimator and adds a
  /// `systems::DrakeVisualizer`, `IiwaStateFeedbackPlanSource`,
  /// `SchunkWsgTrajectoryGenerator` and  a `SchunkWsgStatusSender` to it. This
  /// diagram
  /// is designed for usage within the monolithic pick and place demo.
  /// @param lcm : A reference to the lcm object to be passed onto the
  /// Visualizer
  IiwaWsgPlantGeneratorsEstimatorsAndVisualizer(
      DrakeLcm* lcm, const double update_interval = 0.001);

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

 private:
  IiwaAndWsgPlantWithStateEstimator<T>* plant_{nullptr};
  SchunkWsgStatusSender* wsg_status_sender_{nullptr};
  PassThrough<T>* pass_through_wsg_state_{nullptr};
  DrakeVisualizer* drake_visualizer_{nullptr};
  IiwaStateFeedbackPlanSource* iiwa_trajectory_generator_{nullptr};
  SchunkWsgTrajectoryGenerator* wsg_trajectory_generator_{nullptr};

  int input_port_iiwa_plan_{-1};
  int input_port_wsg_plan_{-1};
  int output_port_wsg_status_{-1};
  int output_port_iiwa_robot_state_msg_{-1};
  int output_port_box_robot_state_msg_{-1};
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
