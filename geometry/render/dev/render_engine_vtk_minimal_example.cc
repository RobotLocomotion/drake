#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/pixel_types.h"
#include "drake/systems/sensors/rgbd_sensor.h"

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_bool(render_on, true, "Sets rendering generally enabled (or not)");
DEFINE_bool(color, true, "Sets the enabled camera to render color");
DEFINE_bool(depth, true, "Sets the enabled camera to render depth");
DEFINE_bool(label, true, "Sets the enabled camera to render label");
DEFINE_double(render_fps, 10, "Frames per simulation second to render");

namespace drake {
namespace geometry {
namespace render {
namespace minimal_example {
namespace {

using Eigen::Vector3d;
using geometry::DrakeVisualizerd;
using geometry::SceneGraph;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderEngineVtkParams;
using geometry::render::RenderLabel;
using geometry::SceneGraph;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::MultibodyPlant;
using multibody::Parser;
using systems::InputPort;
using systems::Context;
using systems::sensors::PixelType;
using systems::sensors::RgbdSensor;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;

int do_main() {
  // Declare builder, plant, and scene_graph.
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>* plant = nullptr;
  SceneGraph<double>* scene_graph = nullptr;

  std::tie(plant, scene_graph) =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

  // Add RenderEngineVtk with default params.
  scene_graph->set_name("scene_graph");
  const std::string render_name("renderer");
  scene_graph->AddRenderer(render_name,
                           MakeRenderEngineVtk(RenderEngineVtkParams()));

  // Add ycb models into a MultibodyPlant.
  Parser parser{plant};
  const auto craker_box = parser.AddModelFromFile(
      FindResourceOrThrow(
          "drake/manipulation/models/ycb/sdf/003_cracker_box.sdf"),
          "cracker_box");
  const RigidTransformd X_WCrackerBox(
      RollPitchYawd{0, 0, 0}, Vector3d(0, 0.2, 0));

  plant->WeldFrames(
      plant->world_frame(),
      plant->GetFrameByName("base_link_cracker", craker_box),
      X_WCrackerBox);

  const auto sugar_box = parser.AddModelFromFile(
      FindResourceOrThrow(
          "drake/manipulation/models/ycb/sdf/004_sugar_box.sdf"),
          "sugar_box");
  const RigidTransformd X_WSugarBox(
      RollPitchYawd{0, 0, M_PI / 2}, Vector3d(-0.2, 0.1, 0));

  plant->WeldFrames(
      plant->world_frame(),
      plant->GetFrameByName("base_link_sugar", sugar_box),
      X_WSugarBox);

  const auto soup_can = parser.AddModelFromFile(
      FindResourceOrThrow(
          "drake/manipulation/models/ycb/sdf/005_tomato_soup_can.sdf"),
          "tomato_soup_can");
  const RigidTransformd X_WSoupCan(
      RollPitchYawd{0, M_PI / 2, 0}, Vector3d(0.3, -0.05, 0));

  plant->WeldFrames(
      plant->world_frame(),
      plant->GetFrameByName("base_link_soup", soup_can),
      X_WSoupCan);

  const auto mustard_bottle = parser.AddModelFromFile(
      FindResourceOrThrow(
          "drake/manipulation/models/ycb/sdf/006_mustard_bottle.sdf"),
          "mustard_bottle");
  const RigidTransformd X_WMustardBottle(
      RollPitchYawd{-M_PI / 2, 0, -M_PI / 2}, Vector3d(0, -0.2, 0));

  plant->WeldFrames(
      plant->world_frame(),
      plant->GetFrameByName("base_link_mustard", mustard_bottle),
      X_WMustardBottle);


  DrakeLcm lcm;
  DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, &lcm);
  if (FLAGS_render_on) {
    // Create the camera and pose it to face the objects.
    const ColorRenderCamera color_camera{
        {render_name, {640, 480, M_PI_4}, {0.01, 10.0}, {}}, false};
    const DepthRenderCamera depth_camera{color_camera.core(), {0.01, 10.0}};
    const RigidTransformd X_WB(
        RollPitchYawd{-M_PI *0.7, 0, M_PI / 2}, Vector3d(0.8, 0, 0.5));

    auto world_id = plant->GetBodyFrameIdOrThrow(plant->world_body().index());
    auto camera = builder.AddSystem<RgbdSensor>(
        world_id, X_WB, color_camera, depth_camera);
    builder.Connect(scene_graph->get_query_output_port(),
                    camera->query_object_input_port());

    // Broadcast the images.
    // Publishing images to drake visualizer
    auto image_to_lcm_image_array =
        builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>();
    image_to_lcm_image_array->set_name("converter");

    systems::lcm::LcmPublisherSystem* image_array_lcm_publisher{nullptr};
    if ((FLAGS_color || FLAGS_depth || FLAGS_label)) {
      image_array_lcm_publisher =
          builder.template AddSystem(systems::lcm::LcmPublisherSystem::Make<
              lcmt_image_array>(
              "DRAKE_RGBD_CAMERA_IMAGES", &lcm,
              1. / FLAGS_render_fps /* publish period */));
      image_array_lcm_publisher->set_name("publisher");

      builder.Connect(
          image_to_lcm_image_array->image_array_t_msg_output_port(),
          image_array_lcm_publisher->get_input_port());
    }

    if (FLAGS_color) {
      const auto& port =
          image_to_lcm_image_array->DeclareImageInputPort<PixelType::kRgba8U>(
              "color");
      builder.Connect(camera->color_image_output_port(), port);
    }

    if (FLAGS_depth) {
      const auto& port =
          image_to_lcm_image_array
              ->DeclareImageInputPort<PixelType::kDepth32F>("depth");
      builder.Connect(camera->depth_image_32F_output_port(), port);
    }

    if (FLAGS_label) {
      const auto& port =
          image_to_lcm_image_array
              ->DeclareImageInputPort<PixelType::kLabel16I>("label");
      builder.Connect(camera->label_image_output_port(), port);
    }
  }

  plant->Finalize();
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.get_mutable_integrator().set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(1.f);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace minimal_example
}  // namespace render
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::render::minimal_example::do_main();
}

