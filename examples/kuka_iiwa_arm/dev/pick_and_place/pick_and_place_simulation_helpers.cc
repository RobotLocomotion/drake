#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/pick_and_place_simulation_helpers.h"

#include <map>
#include <string>
#include <utility>

#include "drake/manipulation/util/frame_pose_tracker.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/optitrack_sender.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

using manipulation::util::FramePoseTracker;
using manipulation::util::ModelInstanceInfo;
using manipulation::util::WorldSimTreeBuilder;
using systems::sensors::OptitrackLcmFrameSender;
using drake::math::RigidTransform;

namespace {

// TODO(sam.creasey) This calculation is duplicated in a number of
// places.  It would probably be better if it were part of the model
// rather than its own constant.

// The `z` coordinate of the top of the table in the world frame.
// The quantity 0.736 is the `z` coordinate of the frame associated with the
// 'surface' collision element in the SDF. This element uses a box of height
// 0.057m thus giving the surface height (`z`) in world coordinates as
// 0.736 + 0.057 / 2.
const double kTableTopZInWorld = 0.736 + 0.057 / 2;

}  // namespace

std::unique_ptr<systems::RigidBodyPlant<double>> BuildPickAndPlacePlant(
    const pick_and_place::SimulatedPlantConfiguration& configuration,
    std::vector<ModelInstanceInfo<double>>* arm_instances,
    std::vector<ModelInstanceInfo<double>>* wsg_instances,
    std::vector<ModelInstanceInfo<double>>* object_instances,
    std::vector<ModelInstanceInfo<double>>* table_instances) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Add models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreDrakeModel(
      "table",
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreDrakeModel("wsg",
                                "drake/manipulation/models/wsg_50_description"
                                "/sdf/schunk_wsg_50_ball_contact.sdf");

  tree_builder->AddGround();

  // Add the robots and the tables they sit on.
  const int num_robots(configuration.robot_models.size());
  DRAKE_THROW_UNLESS(num_robots ==
                     static_cast<int>(configuration.robot_poses.size()));
  for (int i = 0; i < num_robots; ++i) {
    const std::string robot_tag{"robot_" + std::to_string(i)};
    tree_builder->StoreDrakeModel(robot_tag, configuration.robot_models[i]);
    // Add the arm.
    const RigidTransform<double> X_WRobot(configuration.robot_poses[i]);
    const drake::math::RollPitchYaw<double> rpy_WRobot(X_WRobot.rotation());
    const int robot_base_id = tree_builder->AddFixedModelInstance(
        robot_tag, X_WRobot.translation(), rpy_WRobot.vector());
    arm_instances->push_back(
        tree_builder->get_model_info_for_instance(robot_base_id));
    if (wsg_instances) {
      // Add the gripper.
      std::shared_ptr<RigidBodyFrame<double>> frame_ee =
          tree_builder->tree().findFrame(
              "iiwa_frame_ee", arm_instances->back().instance_id);
      auto wsg_frame = std::make_shared<RigidBodyFrame<double>>(*frame_ee);
      wsg_frame->get_mutable_transform_to_body()->rotate(
          Eigen::AngleAxisd(-0.39269908, Eigen::Vector3d::UnitY()));
      wsg_frame->get_mutable_transform_to_body()->translate(
          0.04 * Eigen::Vector3d::UnitY());
      int wsg_id = tree_builder->AddModelInstanceToFrame(
          "wsg", wsg_frame, drake::multibody::joints::kFixed);
      wsg_instances->push_back(
          tree_builder->get_model_info_for_instance(wsg_id));
    }

    // Add the table that the arm sits on.
    const Eigen::Vector3d p_RobotTable_Robot(0.0, 0.0, -kTableTopZInWorld);
    const RigidTransform<double> X_RobotTable(
        drake::math::RotationMatrix<double>::Identity(), p_RobotTable_Robot);
    const RigidTransform<double> X_WTable = X_WRobot * X_RobotTable;
    const drake::math::RollPitchYaw<double> rpy_WTable(X_WTable.rotation());
    tree_builder->AddFixedModelInstance("table", X_WTable.translation(),
                                                 rpy_WTable.vector());
  }

  // Add the objects.
  const int num_objects(configuration.object_models.size());
  DRAKE_THROW_UNLESS(num_objects ==
                     static_cast<int>(configuration.object_poses.size()));
  for (int i = 0; i < num_objects; ++i) {
    const std::string object_tag{"object_" + std::to_string(i)};
    tree_builder->StoreDrakeModel(object_tag, configuration.object_models[i]);
    const RigidTransform<double> X(configuration.object_poses[i]);
    const drake::math::RollPitchYaw<double> rpy(X.rotation());
    const int object_id = tree_builder->AddFloatingModelInstance(object_tag,
                                                 X.translation(), rpy.vector());
    object_instances->push_back(
        tree_builder->get_model_info_for_instance(object_id));
  }

  // Add the tables that the objects sit on.
  const int num_tables(configuration.table_models.size());
  DRAKE_THROW_UNLESS(num_tables ==
                     static_cast<int>(configuration.table_poses.size()));
  for (int i = 0; i < num_tables; ++i) {
    const std::string table_tag{"table_" + std::to_string(i)};
    tree_builder->StoreDrakeModel(table_tag, configuration.table_models[i]);
    const RigidTransform<double> X(configuration.table_poses[i]);
    const drake::math::RollPitchYaw<double> rpy(X.rotation());
    const int table_id = tree_builder->AddFixedModelInstance(
        table_tag, X.translation(), rpy.vector());
    table_instances->push_back(
        tree_builder->get_model_info_for_instance(table_id));
  }

  return std::make_unique<systems::RigidBodyPlant<double>>(
      tree_builder->Build());
}

const systems::OutputPort<double>& AddOptitrackComponents(
    const pick_and_place::OptitrackConfiguration& optitrack_configuration,
    const RigidBodyTree<double>& tree,
    const std::vector<ModelInstanceInfo<double>>& arm_instances,
    const std::vector<ModelInstanceInfo<double>>& object_instances,
    const std::vector<ModelInstanceInfo<double>>& table_instances,
    const systems::OutputPort<double>& kinematics_port,
    systems::DiagramBuilder<double>* builder) {
  // Connect to "simulated" optitrack
  // Create the FramePoseTracker system.
  std::vector<std::unique_ptr<RigidBodyFrame<double>>> frames;
  std::map<std::string, int> frame_name_to_id_map;
  const int num_robot_bases =
      optitrack_configuration.robot_base_optitrack_info.size();
  for (int i = 0; i < num_robot_bases; ++i) {
    const int robot_base_instance_id = arm_instances[i].instance_id;
    const RigidBody<double>& body =
        tree.get_body(tree.FindBaseBodies(robot_base_instance_id).front());
    // The const_cast below is to accommodate the unfortunate API of
    // RigidBodyFrame.
    // TODO(avalenzu): Get rid of this const_cast when possible.
    frames.emplace_back(new RigidBodyFrame<double>(
        "robot_base_" + std::to_string(i),
        const_cast<RigidBody<double>*>(&body),
        optitrack_configuration.robot_base_optitrack_info[i].X_MF));
    frame_name_to_id_map[frames.back()->get_name()] =
        optitrack_configuration.robot_base_optitrack_info[i].id;
  }

  const int num_objects = optitrack_configuration.object_optitrack_info.size();
  for (int i = 0; i < num_objects; ++i) {
    const int object_instance_id = object_instances[i].instance_id;
    const RigidBody<double>& body =
        tree.get_body(tree.FindBaseBodies(object_instance_id).front());
    // The const_cast below is to accommodate the unfortunate API of
    // RigidBodyFrame.
    // TODO(avalenzu): Get rid of this const_cast when possible.
    frames.emplace_back(new RigidBodyFrame<double>(
        "object_" + std::to_string(i), const_cast<RigidBody<double>*>(&body),
        optitrack_configuration.object_optitrack_info[i].X_MF));
    frame_name_to_id_map[frames.back()->get_name()] =
        optitrack_configuration.object_optitrack_info[i].id;
  }

  const int num_tables = optitrack_configuration.table_optitrack_info.size();
  for (int i = 0; i < num_tables; ++i) {
    const int table_instance_id = table_instances[i].instance_id;
    const RigidBody<double>& body =
        tree.get_body(tree.FindBaseBodies(table_instance_id).front());
    // The const_cast below is to accommodate the unfortunate API of
    // RigidBodyFrame.
    // TODO(avalenzu): Get rid of this const_cast when possible.
    frames.emplace_back(new RigidBodyFrame<double>(
        "table_" + std::to_string(i), const_cast<RigidBody<double>*>(&body),
        optitrack_configuration.table_optitrack_info[i].X_MF));
    frame_name_to_id_map[frames.back()->get_name()] =
        optitrack_configuration.table_optitrack_info[i].id;
  }
  for (const auto& frame : frames) {
    drake::log()->info("Frame name: {}", frame->get_name());
  }
  auto pose_tracker = builder->AddSystem<FramePoseTracker>(tree, &frames);

  std::map<geometry::FrameId, std::pair<std::string, int>> frame_map;
  const std::map<std::string, geometry::FrameId>&
      frame_name_to_geometry_id_map =
      pose_tracker->get_frame_name_to_id_map();
  for (auto it = frame_name_to_id_map.begin();
       it != frame_name_to_id_map.end(); ++it) {
    frame_map[frame_name_to_geometry_id_map.at(it->first)] =
        std::pair<std::string, int>(it->first, it->second);
  }

  // Create the Optitrack sender.
  auto optitrack_sender =
      builder->AddSystem<OptitrackLcmFrameSender>(frame_map);

  // Connect the systems related to tracking bodies.
  builder->Connect(kinematics_port,
                   pose_tracker->get_kinematics_input_port());
  builder->Connect(pose_tracker->get_pose_vector_output_port(),
                   optitrack_sender->get_optitrack_input_port());

  return optitrack_sender->get_lcm_output_port();
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
