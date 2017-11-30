#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "robotlocomotion/image_array_t.hpp"

#include "drake/common/unused.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/rgbd_camera.h"

using std::cout;
using std::endl;
using std::string;

using drake::multibody::joints::kQuaternion;
using drake::multibody::joints::kFixed;

namespace drake {
namespace systems {
namespace sensors {
namespace {

bool ValidateSdf(const char* flagname, const std::string& filename) {
  if (filename.substr(filename.find_last_of(".") + 1) == "sdf") {
    return true;
  }
  cout << "Invalid filename for --" << flagname << ": " << filename << endl;
  return false;
}

bool ValidateDir(const char* flagname, const std::string& dir) {
  if (dir.empty()) {
    cout << "Invalid directory for --" << flagname << ": " << dir << endl;
    return false;
  }
  return true;
}

DEFINE_bool(lookup, true,
            "If true, RgbdCamera faces a direction normal to the "
            "terrain plane.");
DEFINE_double(duration, 5., "Total duration of the simulation in secondes.");
DEFINE_string(sdf_dir, "",
              "The full path of directory where SDFs are located.");
DEFINE_string(sdf_fixed, "sphere.sdf",
              "The filename for a SDF that contains fixed base objects.");
DEFINE_string(sdf_floating, "box.sdf",
              "The filename for a SDF that contains floating base objects.");
DEFINE_validator(sdf_dir, &ValidateDir);
DEFINE_validator(sdf_fixed, &ValidateSdf);
DEFINE_validator(sdf_floating, &ValidateSdf);

constexpr double kCameraUpdatePeriod{0.01};

constexpr char kCameraBaseFrameName[] = "camera_base_frame";
constexpr char kColorCameraFrameName[] = "color_camera_optical_frame";
constexpr char kDepthCameraFrameName[] = "depth_camera_optical_frame";
constexpr char kLabelCameraFrameName[] = "label_camera_optical_frame";

constexpr char kImageArrayLcmChannelName[] = "DRAKE_RGBD_CAMERA_IMAGES";
constexpr char kPoseLcmChannelName[] = "DRAKE_RGBD_CAMERA_POSE";

struct CameraConfig {
  Eigen::Vector3d pos;
  Eigen::Vector3d rpy;
  double fov_y{};
  double depth_range_near{};
  double depth_range_far{};
};

}  // anonymous namespace

int main() {
  drake::unused(sdf_dir_validator_registered);
  drake::unused(sdf_fixed_validator_registered);
  drake::unused(sdf_floating_validator_registered);

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      FLAGS_sdf_dir + "/" + FLAGS_sdf_fixed, kFixed, tree.get());

  drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      FLAGS_sdf_dir + "/" + FLAGS_sdf_floating, kQuaternion, tree.get());

  drake::multibody::AddFlatTerrainToWorld(tree.get());

  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem<RigidBodyPlant<double>>(move(tree));
  plant->set_name("rigid_body_plant");

  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(1e8)  // Pa
      .set_dissipation(1)  // s/m
      .set_friction(0.9, 0.5);
  plant->set_default_compliant_material(default_material);

  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_area = 2e-4;  // m^2
  model_parameters.v_stiction_tolerance = 0.01;  // m/s
  plant->set_contact_model_parameters(model_parameters);

  // Adds an RgbdCamera at a fixed pose.
  CameraConfig config;
  if (FLAGS_lookup) {
    config.pos = Eigen::Vector3d(0., -0.02, 0.05);
    config.rpy = Eigen::Vector3d(0., -M_PI_2, 0.);
    config.fov_y = 130. / 180 * M_PI;
    config.depth_range_near = 0.01;
    config.depth_range_far = 1.;
  } else {
    config.pos = Eigen::Vector3d(-1., 0., 1.);
    config.rpy = Eigen::Vector3d(0., M_PI_4, 0.);
    config.fov_y = M_PI_4;
    config.depth_range_near = 0.5;
    config.depth_range_far = 5.;
  }

  auto rgbd_camera =
      builder.AddSystem<RgbdCameraDiscrete>(
          std::make_unique<RgbdCamera>(
              "rgbd_camera", plant->get_rigid_body_tree(),
              config.pos, config.rpy,
              config.depth_range_near, config.depth_range_far,
              config.fov_y),
      kCameraUpdatePeriod);

  auto image_to_lcm_image_array =
      builder.template AddSystem<ImageToLcmImageArrayT>(
          kColorCameraFrameName, kDepthCameraFrameName, kLabelCameraFrameName);
  image_to_lcm_image_array->set_name("converter");

  ::drake::lcm::DrakeLcm lcm;
  auto drake_viz = builder.template AddSystem<DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm);
  drake_viz->set_publish_period(kCameraUpdatePeriod);

  auto image_array_lcm_publisher = builder.template AddSystem(
      lcm::LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
          kImageArrayLcmChannelName, &lcm));
  image_array_lcm_publisher->set_name("publisher");
  image_array_lcm_publisher->set_publish_period(kCameraUpdatePeriod);

  rendering::PoseStampedTPoseVectorTranslator translator(kCameraBaseFrameName);
  auto pose_lcm_publisher = builder.template AddSystem<lcm::LcmPublisherSystem>(
      kPoseLcmChannelName, translator, &lcm);
  pose_lcm_publisher->set_name("pose_lcm_publisher");
  pose_lcm_publisher->set_publish_period(kCameraUpdatePeriod);

  builder.Connect(
      plant->get_output_port(0),
      rgbd_camera->state_input_port());

  builder.Connect(
      plant->get_output_port(0),
      drake_viz->get_input_port(0));

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
