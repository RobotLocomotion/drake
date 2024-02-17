#include <thread>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/meshcat/contact_visualizer_params.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event_status.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"


using drake::geometry::Meshcat;
using drake::geometry::SceneGraph;
using drake::geometry::SceneGraphConfig;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::ContactResults;
using drake::multibody::meshcat::ContactVisualizer;
using drake::multibody::meshcat::ContactVisualizerParams;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;
using drake::systems::Simulator;
using drake::visualization::ApplyVisualizationConfig;
using drake::visualization::VisualizationConfig;

// import time
// from pydrake.common.value import Value
// from pydrake.geometry import StartMeshcat, SceneGraphConfig
// from pydrake.math import RigidTransform
// from pydrake.multibody.meshcat import (
//     ContactVisualizer, ContactVisualizerParams)
// from pydrake.multibody.parsing import Parser
// from pydrake.multibody.plant import (
//     AddMultibodyPlant, MultibodyPlantConfig, ContactModel, ContactResults)
// from pydrake.systems.analysis import Simulator
// from pydrake.systems.framework import DiagramBuilder, LeafSystem
// from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig

namespace drake {
namespace tmp {
namespace {

// Create a table top.
const char kTableTopSdf[] = R"""(
<sdf version="1.7">
  <model name="TableTop">
    <link name="table_top_link">
      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.6 1.0 0.05</size>
          </box>
        </geometry>
        <material>
         <diffuse>0.9 0.8 0.7 0.5</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.6 1.0 0.05</size>
          </box>
        </geometry>
      </collision>
    </link>
    <frame name="top_surface">
      <pose relative_to="table_top_link">0 0 0.025 0 0 0</pose>
    </frame>
  </model>
</sdf>
)""";

const char kFreeBodySdf[] = R"""(
<sdf version='1.9'>
<model name='free'>
<include>
  <uri>package://drake/manipulation/models/ycb/sdf/006_mustard_bottle.sdf</uri>
</include>
</model>
</sdf>
)""";


class ContactReporter : public LeafSystem<double> {
 public:
  ContactReporter() {
    DeclareAbstractInputPort(
        "contact_results",
        Value(
            // Input port will take ContactResults from MultibodyPlant
            ContactResults<double>()));
    // Calling `ForcedPublish()` will trigger the callback.
    this->DeclareForcedPublishEvent(&ContactReporter::Publish);
  }

  EventStatus Publish(const Context<double>& context) const {
    drake::log()->info(
        "\nContactReporter::Publish() called at time={}", context.get_time());

    ContactResults<double> contact_results =
        get_input_port().Eval<ContactResults<double>>(context);
    int num_hydroelastic_contacts = contact_results.num_hydroelastic_contacts();
    drake::log()->info(
        "num_hydroelastic_contacts() = {}", num_hydroelastic_contacts);

    for (int c = 0; c < num_hydroelastic_contacts; ++c) {
    drake::log()->info(
        "\nhydroelastic_contact_info({}): {}-th hydroelastic contact patch", c, c);

      const auto& hydroelastic_contact_info = contact_results.hydroelastic_contact_info(c);
      const auto& spatial_force = hydroelastic_contact_info.F_Ac_W();
      drake::log()->info(
          "F_Ac_W(): spatial force (on body A, at centroid of contact surface,"
          " in World frame) = {}", spatial_force);

      const auto& contact_surface = hydroelastic_contact_info.contact_surface();
      int num_faces = contact_surface.num_faces();
      double total_area = contact_surface.total_area();
      const auto& centroid = contact_surface.centroid();
      drake::log()->info(
          "contact_surface()\n"
          "total_area(): area of contact surface in m^2 = {}\n"
          "num_faces(): number of polygons or triangles = {}\n"
          "centroid(): centroid (in World frame) = [{} {} {}]\n",
          total_area, num_faces, centroid[0], centroid[1], centroid[2]);
    }
    return EventStatus::Succeeded();
  }
};


class BoxDrop {
 public:
  BoxDrop() {
    meshcat_ = std::make_shared<Meshcat>();
  }

  void ClearMeshcat() {
    meshcat_->Delete();
    meshcat_->DeleteAddedControls();
  }

  void AddScene(double time_step, const std::string& free_body_name) {
    MultibodyPlantConfig plant_config;
    plant_config.time_step = time_step;
    plant_config.discrete_contact_approximation = "sap";
    auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder_);
    plant_ = &plant;
    scene_graph_ = &scene_graph;
    Parser parser(plant_);

    // Load the table top and the box we created.
    parser.AddModelsFromString(kFreeBodySdf, "sdf");
    parser.AddModelsFromString(kTableTopSdf, "sdf");

    // Weld the rigid box to the world so that it's fixed during simulation.
    // The top surface passes the world's origin.
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("top_surface"));

    // Finalize the plant after loading the scene.
    plant.set_contact_model(multibody::ContactModel::kHydroelastic);
    SceneGraphConfig config;
    config.default_proximity_properties.compliance_type = "compliant";
    scene_graph.set_config(config);
    plant.Finalize();

    // Set how high the center of the compliant box is from the world's origin.
    // W = the world's frame
    // C = frame at the center of the compliant box
    RigidTransform<double> X_WC(Vector3<double>{0, 0, 1});
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName(free_body_name), X_WC);
  }

  void AddViz() {
    VisualizationConfig config;
    config.publish_period = 1 / 256.0;
    config.publish_contacts = false;
    ApplyVisualizationConfig(config, &builder_, {}, {}, {}, meshcat_);
  }

  void AddContactReport() {
    auto contact_reporter = builder_.AddSystem(std::make_unique<ContactReporter>());
    builder_.Connect(plant_->get_contact_results_output_port(),
                    contact_reporter->get_input_port(0));
  }

  void AddContactViz() {
    ContactVisualizerParams params;
    params.publish_period = 1.0 / 256.0;
    params.newtons_per_meter = 2e1;
    params.newton_meters_per_meter = 1e-1;
    ContactVisualizer<double>::AddToBuilder(&builder_, *plant_, meshcat_, params);
  }

  void RunSimulationWithContactReportAndViz(
      double sim_time, const std::string& free_body_name) {
    ClearMeshcat();

    AddScene(0.001, free_body_name);
    AddViz();
    AddContactReport();
    AddContactViz();

    Simulator<double> simulator(builder_.Build());

    simulator.Initialize();
    simulator.set_target_realtime_rate(1.0);

    meshcat_->StartRecording(256.0);
    simulator.AdvanceTo(sim_time);
    meshcat_->StopRecording();

    // Numerically report contact results at the end of simulation.
    simulator.get_system().ForcedPublish(simulator.get_context());
  }

  std::shared_ptr<Meshcat> meshcat() {
    return meshcat_;
  }

 private:
  DiagramBuilder<double> builder_;
  std::shared_ptr<Meshcat> meshcat_;
  MultibodyPlant<double>* plant_;
  SceneGraph<double>* scene_graph_;
};

int do_main(int argc, const char* argv[]) {
  unused(argc, argv);
  BoxDrop box_drop;
  while (box_drop.meshcat()->GetNumActiveConnections() <= 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  box_drop.RunSimulationWithContactReportAndViz(2, "base_link_mustard");
  box_drop.meshcat()->PublishRecording();

  while (box_drop.meshcat()->GetNumActiveConnections() > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
}  // namespace
}  // namespace tmp
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::tmp::do_main(argc, argv);
}
