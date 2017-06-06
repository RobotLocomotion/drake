#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

#include <string>

#include <gflags/gflags.h>
#include "robotlocomotion/image_array_t.hpp"

#include "drake/common/unused.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"
#include "drake/systems/sensors/rgbd_camera.h"

using std::cout;
using std::endl;
using std::string;

using drake::multibody::joints::kQuaternion;

namespace drake {
namespace systems {
namespace sensors {
namespace {

bool ValidateSdf(const char* flagname, const std::string& sdf) {
  if (sdf.empty()) {
    cout << "Invalid filename for --" << flagname << ": " << sdf << endl;
    return false;
  }
  return true;
}

DEFINE_double(duration, 5., "Total duration of the simulation in secondes.");
DEFINE_string(sdf, "", "The filename for SDF.");
DEFINE_validator(sdf, &ValidateSdf);

constexpr double kCameraPosePublishPeriod{0.01};
constexpr double kImageArrayPublishPeriod{0.01};

constexpr char kCameraBaseFrameName[] = "camera_base_frame";
constexpr char kColorCameraFrameName[] = "color_camera_optical_frame";
constexpr char kDepthCameraFrameName[] = "depth_camera_optical_frame";
constexpr char kLabelCameraFrameName[] = "label_camera_optical_frame";

constexpr char kImageArrayLcmChannelName[] = "DRAKE_RGBD_CAMERA_IMAGES";
constexpr char kPoseLcmChannelName[] = "DRAKE_RGBD_CAMERA_POSE";
}  // anonymous namespace

int main() {
  drake::unused(sdf_validator_registered);

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      FLAGS_sdf, kQuaternion, tree.get());

  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem<RigidBodyPlant<double>>(move(tree));
  plant->set_name("rigid_body_plant");
  plant->set_normal_contact_parameters(3000, 10);
  plant->set_friction_contact_parameters(0.9, 0.5, 0.01);

  // Adds an RgbdCamera at a fixed pose.
  auto rgbd_camera =
      builder.template AddSystem<RgbdCamera>(
          "rgbd_camera", plant->get_rigid_body_tree(),
          Eigen::Vector3d(-1., 0., 1.),
          Eigen::Vector3d(0., M_PI_4, 0.), M_PI_4, true);

  auto image_to_lcm_image_array =
      builder.template AddSystem<ImageToLcmImageArrayT>(
          kColorCameraFrameName, kDepthCameraFrameName, kLabelCameraFrameName);
  image_to_lcm_image_array->set_name("converter");

  ::drake::lcm::DrakeLcm lcm;
  auto image_array_lcm_publisher = builder.template AddSystem(
      lcm::LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
          kImageArrayLcmChannelName, &lcm));
  image_array_lcm_publisher->set_name("publisher");
  image_array_lcm_publisher->set_publish_period(kImageArrayPublishPeriod);

  rendering::PoseStampedTPoseVectorTranslator translator(kCameraBaseFrameName);
  auto pose_lcm_publisher = builder.template AddSystem<lcm::LcmPublisherSystem>(
      kPoseLcmChannelName, translator, &lcm);
  pose_lcm_publisher->set_name("pose_lcm_publisher");
  pose_lcm_publisher->set_publish_period(kCameraPosePublishPeriod);

  builder.Connect(
      plant->get_output_port(0),
      rgbd_camera->state_input_port());

  builder.Connect(
      rgbd_camera->color_image_output_port(),
      image_to_lcm_image_array->color_image_input_port());

  builder.Connect(
      rgbd_camera->depth_image_output_port(),
      image_to_lcm_image_array->depth_image_input_port());

  builder.Connect(
      rgbd_camera->label_image_output_port(),
      image_to_lcm_image_array->label_image_input_port());

  builder.Connect(
      image_to_lcm_image_array->image_array_t_msg_output_port(),
      image_array_lcm_publisher->get_input_port(0));

  builder.Connect(
      rgbd_camera->camera_base_pose_output_port(),
      pose_lcm_publisher->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto simulator = std::make_unique<systems::Simulator<double>>(
      *diagram, std::move(context));

  simulator->set_publish_at_initialization(true);
  simulator->set_publish_every_time_step(false);
  simulator->Initialize();
  simulator->StepTo(FLAGS_duration);

  return 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::systems::sensors::main();
}
