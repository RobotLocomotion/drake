#include <gflags/gflags.h>

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/perception/rigid_body_tree_removal.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/rgbd_camera.h"

namespace drake {
namespace perception {
namespace filtering {

using examples::kuka_iiwa_arm::IiwaAndWsgPlantWithStateEstimator;
using examples::kuka_iiwa_arm::kIiwaLcmStatusPeriod;
using examples::kuka_iiwa_arm::IiwaCommandReceiver;
using examples::kuka_iiwa_arm::IiwaStatusSender;
using examples::kuka_iiwa_arm::IiwaContactResultsToExternalTorque;
using manipulation::schunk_wsg::SchunkWsgController;
using manipulation::schunk_wsg::SchunkWsgStatusSender;
using manipulation::util::ModelInstanceInfo;
using manipulation::util::WorldSimTreeBuilder;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::LcmSubscriberSystem;
using systems::RigidBodyPlant;
using systems::sensors::ImageToLcmImageArrayT;
using systems::sensors::RgbdCamera;
using systems::sensors::RgbdCameraDiscrete;

// Information for constructing a simulated physical camera in a scene.
template <typename T>
struct PhysicalCamera {
  std::string name;
  std::shared_ptr<RigidBodyFrame<T>> fixture_frame;
  std::shared_ptr<RigidBodyFrame<T>> sensor_frame;
  Eigen::Isometry3d extrinsics;
};

void AddCamera(WorldSimTreeBuilder<double>* tree_builder,
               PhysicalCamera<double>* physical_camera,
               const Eigen::Vector3d& base, const Eigen::Vector3d& rotation) {
  const char* const kWsgFixtureUrdf =
        "drake/manipulation/models/xtion_description/urdf/"
        "xtion_wsg_fixture.urdf";
  tree_builder->StoreDrakeModel("xtion_wsg_fixture", kWsgFixtureUrdf);

  int fixture_id =
      tree_builder->AddFixedModelInstance("xtion_wsg_fixture", base, rotation);
  physical_camera->fixture_frame =
      tree_builder->tree().findFrame("xtion_wsg_fixture", fixture_id);

  drake::log()->info(" fixture_id: {}", fixture_id);
  drake::log()->info(
      "  fixture_frame:   {}",
      physical_camera->fixture_frame->get_transform_to_body().matrix());

  tree_builder->StoreDrakeModel(
          physical_camera->name,
          "drake/manipulation/models/xtion_description/urdf/xtion.urdf");
  int xtion_id = tree_builder->AddModelInstanceToFrame(
      "xtion", physical_camera->fixture_frame,
      drake::multibody::joints::kFixed);
  physical_camera->sensor_frame =
      tree_builder->tree().findFrame("rgbd_camera_frame", xtion_id);

  drake::log()->info(
      "  fixture_frame:   {}",
      physical_camera->fixture_frame->get_transform_to_body().matrix());
}

template <typename T>
std::unique_ptr<RigidBodyPlant<T>> BuildCombinedPlant(
    ModelInstanceInfo<T>* iiwa_instance, ModelInstanceInfo<T>* wsg_instance,
    ModelInstanceInfo<T>* box_instance, PhysicalCamera<T>* physical_camera) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  const char* const kIiwaUrdf =
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf";
  tree_builder->StoreDrakeModel("iiwa", kIiwaUrdf);
  tree_builder->StoreDrakeModel(
      "table",
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreDrakeModel(
      "box",
      "drake/examples/kuka_iiwa_arm/models/objects/"
      "block_for_pick_and_place.urdf");
  tree_builder->StoreDrakeModel(
      "wsg",
      "drake/manipulation/models/wsg_50_description"
      "/sdf/schunk_wsg_50_ball_contact.sdf");

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

  // The `z` coordinate of the top of the table in the world frame.
  // The quantity 0.736 is the `z` coordinate of the frame associated with the
  // 'surface' collision element in the SDF. This element uses a box of height
  // 0.057m thus giving the surface height (`z`) in world coordinates as
  // 0.736 + 0.057 / 2.
  const double kTableTopZInWorld = 0.736 + 0.057 / 2;

  // Coordinates for kRobotBase originally from iiwa_world_demo.cc.
  // The intention is to center the robot on the table.
  const Eigen::Vector3d kRobotBase(0, 0, kTableTopZInWorld);
  // Start the box slightly above the table.  If we place it at
  // the table top exactly, it may start colliding the table (which is
  // not good, as it will likely shoot off into space).
  const Eigen::Vector3d kBoxBase(0.8, 0., kTableTopZInWorld + 0.1);

  int id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(id);
  id = tree_builder->AddFloatingModelInstance("box", kBoxBase,
                                              Vector3<double>(0, 0, 0));
  *box_instance = tree_builder->get_model_info_for_instance(id);
  id = tree_builder->AddModelInstanceToFrame(
      "wsg", tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(id);

  // Add the camera.
  const Eigen::Vector3d kCamBase(0, 0, 3.0);
  const Eigen::Vector3d kCamRotation(-M_PI_2, 0, 0);
  AddCamera(tree_builder.get(), physical_camera, kCamBase, kCamRotation);

  auto plant = std::make_unique<RigidBodyPlant<T>>(tree_builder->Build());

  return plant;
}

int do_main() {
  std::unique_ptr<PhysicalCamera<double>> physical_camera;
  physical_camera.reset(new PhysicalCamera<double>());
  physical_camera->name = "xtion";

  ModelInstanceInfo<double> iiwa_instance, wsg_instance, box_instance;
  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>(&iiwa_instance, &wsg_instance, &box_instance, physical_camera.get());
  model_ptr->set_name("plant");

  systems::DiagramBuilder<double> builder;
  auto model =
        builder.template AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
            std::move(model_ptr), iiwa_instance, wsg_instance, box_instance);
  model->set_name("plant_with_state_estimator");

  const RigidBodyTree<double>& tree = model->get_plant().get_rigid_body_tree();

  // Add visualizer.
  drake::lcm::DrakeLcm lcm;
  systems::DrakeVisualizer* visualizer = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);
  visualizer->set_name("visualizer");
  builder.Connect(model->get_output_port_plant_state(),
                  visualizer->get_input_port(0));
  visualizer->set_publish_period(examples::kuka_iiwa_arm::kIiwaLcmStatusPeriod);

  const double depth_range_near = 0.2;
  const double depth_range_far = 1.5;
  const double fov_y = M_PI_4;
  const double period = 1. / 300;  // Should this be 60FPS?
  const double render_label_image = false;

  auto* camera = builder.AddSystem<RgbdCameraDiscrete>(
      std::make_unique<RgbdCamera>(physical_camera->name, model->get_tree(),
                                   *physical_camera->sensor_frame,
                                   depth_range_near, depth_range_far, fov_y),
      period, render_label_image);
  camera->set_name(physical_camera->name + "_rgbd_camera");

  builder.Connect(model->get_output_port_plant_state(),
                  camera->state_input_port());

  const std::string camera_name = "simulated_xtion";
  const std::string lcm_image_channel =
      "DRAKE_RGBD_CAMERA_IMAGES_" + camera_name;

//  // Image to LCM.
//  auto* image_to_lcm_message =
//      builder.AddSystem<ImageToLcmImageArrayT>("color", "depth", "label");
//  image_to_lcm_message->set_name(physical_camera->name + "_lcm_converter");
//
//  builder.Connect(camera->color_image_output_port(),
//                  image_to_lcm_message->color_image_input_port());
//
//  builder.Connect(camera->depth_image_output_port(),
//                  image_to_lcm_message->depth_image_input_port());
//
//  if (render_label_image) {
//    builder.Connect(camera->label_image_output_port(),
//                    image_to_lcm_message->label_image_input_port());
//  }
//
//  // Camera image publisher.
//  auto* image_lcm_pub = builder.AddSystem(
//      LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
//          lcm_image_channel, &lcm));
//  image_lcm_pub->set_name(physical_camera->name + "_lcm_publisher");
//  image_lcm_pub->set_publish_period(period);
//
//  builder.Connect(image_to_lcm_message->image_array_t_msg_output_port(),
//                  image_lcm_pub->get_input_port());
//
//  // Make the camera description message.
//  anzu::camera_description_t desc{};
//  desc.camera_name = camera_name;
//  desc.lcm_channel_name = lcm_image_channel;
//  desc.num_image_types = 2;
//
//  auto& rgb_info = camera->camera().color_camera_info();
//  auto& depth_info = camera->camera().depth_camera_info();
//
//  std::vector<rgbd_bridge::ImageType> types = {
//      rgbd_bridge::ImageType::RGB, rgbd_bridge::ImageType::DEPTH};
//  std::map<rgbd_bridge::ImageType, rgbd_bridge::Intrinsics> intrinsics;
//  // Color
//  intrinsics[rgbd_bridge::ImageType::RGB] = rgbd_bridge::Intrinsics(
//      rgb_info.width(), rgb_info.height(), rgb_info.focal_x(),
//      rgb_info.focal_y(), rgb_info.center_x(), rgb_info.center_y());
//  // Depth
//  intrinsics[rgbd_bridge::ImageType::DEPTH] = rgbd_bridge::Intrinsics(
//      depth_info.width(), depth_info.height(), depth_info.focal_x(),
//      depth_info.focal_y(), depth_info.center_x(), depth_info.center_y());
//
//  for (rgbd_bridge::ImageType type : types) {
//    anzu::image_description_t image_desc{};
//    image_desc.frame_name = rgbd_bridge::ImageTypeToFrameName(type);
//    image_desc.type = rgbd_bridge::ImageTypeToDescriptionType(type);
//
//    image_desc.intrinsics =
//        rgbd_bridge::SerializeIntrinsics(intrinsics.at(type));
//
//    for (rgbd_bridge::ImageType to_type : types) {
//      image_desc.extrinsics.push_back(rgbd_bridge::SerializeExtrinsics(
//          type, to_type, Eigen::Isometry3f::Identity()));
//    }
//    image_desc.num_extrinsics = image_desc.extrinsics.size();
//    desc.image_types.push_back(image_desc);
//  }
//
//  // make a publisher
//  auto camera_info_pub = builder.AddSystem(
//      drake::systems::lcm::LcmPublisherSystem::Make<
//          anzu::camera_description_t>("DRAKE_RGBD_CAMERAS", &lcm));
//  camera_info_pub->set_publish_period(period);
//
//  auto camera_info_msg_source = builder.template AddSystem<
//      drake::systems::ConstantValueSource<double>>(
//      drake::systems::AbstractValue::Make<anzu::camera_description_t>(
//          desc));
//  builder.Connect(camera_info_msg_source->get_output_port(0),
//                  camera_info_pub->get_input_port());

  // Create the command subscriber and status publisher.
    auto iiwa_command_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND",
                                                                   &lcm));
    iiwa_command_sub->set_name("iiwa_command_subscriber");
    auto iiwa_command_receiver = builder.AddSystem<IiwaCommandReceiver>();
    iiwa_command_receiver->set_name("iwwa_command_receiver");

    auto iiwa_status_pub = builder.AddSystem(
        LcmPublisherSystem::Make<lcmt_iiwa_status>("IIWA_STATUS",
                                                                 &lcm));
    iiwa_status_pub->set_name("iiwa_status_publisher");
    iiwa_status_pub->set_publish_period(kIiwaLcmStatusPeriod);
    auto iiwa_status_sender = builder.AddSystem<IiwaStatusSender>();
    iiwa_status_sender->set_name("iiwa_status_sender");

    std::vector<int> instance_ids = {iiwa_instance.instance_id};
    auto external_torque_converter =
        builder.AddSystem<IiwaContactResultsToExternalTorque>(
            tree, instance_ids);

    // TODO(siyuan): Connect this to kuka_planner runner once it generates
    // reference acceleration.
    auto iiwa_zero_acceleration_source =
        builder.template AddSystem<systems::ConstantVectorSource<double>>(
            Eigen::VectorXd::Zero(7));
    iiwa_zero_acceleration_source->set_name("zero_acceleration");

    builder.Connect(iiwa_command_sub->get_output_port(),
                    iiwa_command_receiver->get_input_port(0));
    builder.Connect(iiwa_command_receiver->get_commanded_state_output_port(),
                    model->get_input_port_iiwa_state_command());
    builder.Connect(iiwa_zero_acceleration_source->get_output_port(),
                    model->get_input_port_iiwa_acceleration_command());

    builder.Connect(model->get_output_port_iiwa_state(),
                    iiwa_status_sender->get_state_input_port());
    builder.Connect(iiwa_command_receiver->get_output_port(0),
                    iiwa_status_sender->get_command_input_port());
    builder.Connect(model->get_output_port_computed_torque(),
                    iiwa_status_sender->get_commanded_torque_input_port());
    builder.Connect(model->get_output_port_iiwa_measured_torque(),
                    iiwa_status_sender->get_measured_torque_input_port());
    builder.Connect(model->get_output_port_contact_results(),
                    external_torque_converter->get_input_port(0));
    builder.Connect(external_torque_converter->get_output_port(0),
                    iiwa_status_sender->get_external_torque_input_port());
    builder.Connect(iiwa_status_sender->get_output_port(0),
                    iiwa_status_pub->get_input_port());

    auto wsg_command_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<lcmt_schunk_wsg_command>(
            "SCHUNK_WSG_COMMAND", &lcm));
    wsg_command_sub->set_name("wsg_command_subscriber");
    auto wsg_controller = builder.AddSystem<SchunkWsgController>();

    auto wsg_status_pub = builder.AddSystem(
        LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
            "SCHUNK_WSG_STATUS", &lcm));
    wsg_status_pub->set_name("wsg_status_publisher");
    wsg_status_pub->set_publish_period(
        manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod);

    auto wsg_status_sender = builder.AddSystem<SchunkWsgStatusSender>(
        model->get_output_port_wsg_state().size(),
        model->get_output_port_wsg_measured_torque().size(),
        manipulation::schunk_wsg::kSchunkWsgPositionIndex,
        manipulation::schunk_wsg::kSchunkWsgVelocityIndex);
    wsg_status_sender->set_name("wsg_status_sender");

    builder.Connect(wsg_command_sub->get_output_port(),
                    wsg_controller->get_command_input_port());
    builder.Connect(wsg_controller->get_output_port(0),
                    model->get_input_port_wsg_command());
    builder.Connect(model->get_output_port_wsg_state(),
                    wsg_status_sender->get_input_port_wsg_state());
    builder.Connect(model->get_output_port_wsg_measured_torque(),
                    wsg_status_sender->get_input_port_measured_torque());
    builder.Connect(model->get_output_port_wsg_state(),
                    wsg_controller->get_state_input_port());
    builder.Connect(*wsg_status_sender, *wsg_status_pub);

    auto iiwa_state_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
            "EST_ROBOT_STATE", &lcm));
    iiwa_state_pub->set_name("iiwa_state_publisher");
    iiwa_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

    builder.Connect(model->get_output_port_iiwa_robot_state_msg(),
                    iiwa_state_pub->get_input_port());
    iiwa_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

    auto box_state_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
            "OBJECT_STATE_EST", &lcm));
    box_state_pub->set_name("box_state_publisher");
    box_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

    builder.Connect(model->get_output_port_object_robot_state_msg(),
                    box_state_pub->get_input_port());
    box_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

  auto sys = builder.Build();
  systems::Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.set_publish_every_time_step(false);
  double simulation_sec = 2.0;
  simulator.StepTo(simulation_sec);

  // TODO: fix segmentation fault
  const systems::AbstractValue& output = camera->depth_image_output_port().EvalAbstract(simulator.get_context());
  const auto& output_image = output.GetValue<systems::sensors::ImageDepth32F>();
  drake::log()->info(
        "  depth image:   {}",
        output_image.width());

//  const auto& depth_image = camera->depth_image_output_port<double>().Eval(simulator.get_context());
//  auto depth_image = camera->ConvertDepthImageToPointCloud(const ImageDepth32F& depth_image,
//                                              const CameraInfo& camera_info,
//                                              Eigen::Matrix3Xf* point_cloud)

  return 0;
}

}  // namespace filtering
}  // namespace perception
}  // namespace drake


int main(int argc, char* argv[]) {
//  gflags::SetUsageMessage(
//      "A simple acrobot demo using Drake's MultibodyTree,"
//      "with SceneGraph visualization. "
//      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
//  drake::logging::HandleSpdlogGflags();
  return drake::perception::filtering::do_main();
}
