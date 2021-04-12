/// @file This program is an example of using the LcmImageArrayToImages and
/// ImageToLcmImageArrayT systems.  It receives an incoming stream of
/// lcmt_image_array messages, unpacks and repacks the color and
/// depth frames, and retransmits the images.  The output can be viewed in
/// drake_visualizer.

#include <gflags/gflags.h>

#include "drake/lcmt_image_array.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/lcm_image_array_to_images.h"

DEFINE_string(channel_name, "DRAKE_RGBD_CAMERA_IMAGES_IN",
              "Channel name to receive images on");
DEFINE_string(publish_name, "DRAKE_RGBD_CAMERA_IMAGES",
              "Channel name to publish images on");
DEFINE_double(duration, 5., "Total duration of the simulation in secondes.");

namespace drake {
namespace systems {
namespace sensors {
namespace {

int DoMain() {
  DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  auto image_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_image_array>(
          FLAGS_channel_name, lcm));
  auto array_to_images = builder.AddSystem<LcmImageArrayToImages>();
  builder.Connect(image_sub->get_output_port(),
                  array_to_images->image_array_t_input_port());

  auto image_to_lcm_image_array =
      builder.AddSystem<ImageToLcmImageArrayT>(true);
  builder.Connect(
      array_to_images->color_image_output_port(),
      image_to_lcm_image_array->DeclareImageInputPort<PixelType::kRgba8U>(
          "color"));
  builder.Connect(array_to_images->depth_image_output_port(),
      image_to_lcm_image_array->DeclareImageInputPort<PixelType::kDepth32F>(
          "depth"));

  auto image_array_lcm_publisher = builder.AddSystem(
      lcm::LcmPublisherSystem::Make<lcmt_image_array>(
          FLAGS_publish_name, lcm, 0.1 /* publish period */));
  builder.Connect(
      image_to_lcm_image_array->image_array_t_msg_output_port(),
      image_array_lcm_publisher->get_input_port());

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto simulator = std::make_unique<systems::Simulator<double>>(
      *diagram, std::move(context));

  simulator->set_publish_at_initialization(true);
  simulator->set_publish_every_time_step(false);
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();
  simulator->AdvanceTo(FLAGS_duration);

  return 0;
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::systems::sensors::DoMain();
}
