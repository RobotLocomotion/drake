#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/multibody/deformable/parallel_gripper_controller.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/camera_config.h"
#include "drake/systems/sensors/camera_config_functions.h"

DEFINE_double(simulation_time, 15.0, "Desired duration of the simulation [s].");
DEFINE_double(realtime_rate, 0.0, "Desired real time rate.");
DEFINE_double(discrete_time_step, 2.5e-2,
              "Discrete time step for the system [s].");
DEFINE_bool(render_bubble, true,
            "Renders the dot pattern inside the bubble gripper if true.");

using drake::examples::deformable::ParallelGripperController;
using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::Mesh;
using drake::geometry::PerceptionProperties;
using drake::geometry::ProximityProperties;
using drake::geometry::RenderEngineVtkParams;
using drake::geometry::Rgba;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::Body;
using drake::multibody::CoulombFriction;
using drake::multibody::DeformableBodyId;
using drake::multibody::DeformableModel;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::PackageMap;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::fem::DeformableBodyConfig;
using drake::schema::Transform;
using drake::systems::Context;
using drake::systems::sensors::ApplyCameraConfig;
using drake::systems::sensors::CameraConfig;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace drake {
namespace examples {
namespace multibody {
namespace bubble_gripper {
namespace {

int do_main() {
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  DRAKE_DEMAND(FLAGS_discrete_time_step > 0.0);
  plant_config.time_step = FLAGS_discrete_time_step;

  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  if (FLAGS_render_bubble) {
    /* Add a renderer to render the inside dot pattern of the bubble gripper. */
    const RenderEngineVtkParams vtk_params{.backend = "GLX"};
    scene_graph.AddRenderer("renderer",
                            geometry::MakeRenderEngineVtk(vtk_params));
  }

  /* Minimum required proximity properties for rigid bodies to interact with
   deformable bodies.
   1. A valid Coulomb friction coefficient, and
   2. A resolution hint. (Rigid bodies need to be tessellated so that collision
   queries can be performed against deformable geometries.)
   3. A hydroelastic modulus (which is added by default through scene graph
   config). */
  ProximityProperties rigid_proximity_props;
  const CoulombFriction<double> surface_friction(1.0, 1.0);
  const double resolution_hint = 0.01;
  AddContactMaterial({}, {}, surface_friction, &rigid_proximity_props);
  rigid_proximity_props.AddProperty(geometry::internal::kHydroGroup,
                                    geometry::internal::kRezHint,
                                    resolution_hint);

  /* Set up a ground. */
  Box ground{1, 1, 1};
  const RigidTransformd X_WG(Eigen::Vector3d{0, 0, -0.505});
  plant.RegisterCollisionGeometry(plant.world_body(), X_WG, ground,
                                  "ground_collision", rigid_proximity_props);
  IllustrationProperties illustration_props;
  illustration_props.AddProperty("phong", "diffuse",
                                 Vector4d(0.95, 0.80, 0.65, 0.9));
  /* Avoid rendering the ground as it clutters the background.
   Currently, all visual geometries added through MultibodyPlant are
   automatically assigned perception properties. When that automatic assignment
   is no longer done, we can remove this and simply not assign a perception
   property. */
  illustration_props.AddProperty("renderer", "accepting",
                                 std::set<std::string>{"nothing"});
  plant.RegisterVisualGeometry(plant.world_body(), X_WG, ground,
                               "ground_visual", illustration_props);

  /* Parse the gripper model (without the bubbles). */
  Parser parser(&builder);
  ModelInstanceIndex gripper_instance = parser.AddModelsFromUrl(
      "package://drake_models/wsg_50_description/sdf/"
      "schunk_wsg_50_deformable_bubble.sdf")[0];

  /* Add in the bubbles. */
  const auto model_instances = parser.AddModelsFromUrl(
      "package://drake/examples/multibody/deformable/models/bubbles.sdf");
  // TODO(xuchenhan-tri): Use name to retrieve deformable bodies for more
  // robustness.
  const std::vector<DeformableBodyId> bubble_ids =
      plant.deformable_model().GetBodyIds(model_instances[0]);
  const DeformableBodyId left_bubble = bubble_ids[0];
  const DeformableBodyId right_bubble = bubble_ids[1];
  auto& deformable_model = plant.mutable_deformable_model();

  /* Now we attach the bubble to the WSG finger using a fixed constraint. To do
   that, we specify a box geometry and put all vertices of the bubble geometry
   under fixed constraint with the rigid finger if they fall inside the box.
   Refer to DeformableModel::AddFixedConstraint for details. */
  const Body<double>& left_finger = plant.GetBodyByName("left_finger");
  /* Pose of the bubble in the left finger body frame. */
  const RigidTransformd X_FlBl = RigidTransformd(
      math::RollPitchYawd(M_PI_2, M_PI_2, 0), Vector3d(0.0, -0.03, -0.1125));
  /* All vertices of the deformable bubble mesh inside this box will be subject
   to fixed constraints. */
  const Box box(0.1, 0.004, 0.15);
  deformable_model.AddFixedConstraint(
      left_bubble, left_finger, X_FlBl, box,
      /* The pose of the box in the left finger's frame. */
      RigidTransformd(Vector3d(0.0, -0.03, -0.1)));

  const Body<double>& right_finger = plant.GetBodyByName("right_finger");
  /* Pose of the right finger body (at initialization) in the world frame. */
  const RigidTransformd X_FrBr = RigidTransformd(
      math::RollPitchYawd(-M_PI_2, M_PI_2, 0), Vector3d(0.0, 0.03, -0.1125));
  deformable_model.AddFixedConstraint(
      right_bubble, right_finger, X_FrBr, box,
      /* The pose of the box in the right finger's frame. */
      RigidTransformd(Vector3d(0.0, 0.03, -0.1)));

  /* Add in a deformable manipuland. */
  parser.AddModelsFromUrl(
      "package://drake/examples/multibody/deformable/models/"
      "deformable_teddy.sdf");

  /* All rigid and deformable models have been added. Finalize the plant. */
  plant.Finalize();

  /* Set the width between the fingers for open and closed states as well as the
   height to which the gripper lifts the manipuland. */
  const double open_width = 0.12;
  const double closed_width = 0.04;
  const double lifted_height = 0.12;
  const auto& control = *builder.AddSystem<ParallelGripperController>(
      open_width, closed_width, lifted_height);
  builder.Connect(control.get_output_port(),
                  plant.get_desired_state_input_port(gripper_instance));

  /* Add a visualizer that emits LCM messages for visualization. */
  geometry::DrakeVisualizerParams params;
  params.role = geometry::Role::kIllustration;
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, nullptr,
                                           params);
  /* We want to look in the -Py direction so we line up Bz with -Py.*/
  const Vector3d Bz_P = -Vector3d::UnitY();
  const Vector3d Bx_P = -Vector3d::UnitZ();
  const Vector3d By_P = Bz_P.cross(Bx_P);  // Already a unit vector.
  const Vector3d p_PB(0, 0.06, -0.11);
  const RotationMatrixd R_PB =
      RotationMatrixd::MakeFromOrthonormalColumns(Bx_P, By_P, Bz_P);
  Transform schema_X_PB(RigidTransformd(R_PB, p_PB));
  schema_X_PB.base_frame = "right_finger";

  /* Create the camera if rendering is requested. */
  if (FLAGS_render_bubble) {
    const CameraConfig camera_config{.width = 1280,
                                     .height = 720,
                                     .focal{CameraConfig::FovDegrees{.y = 90}},
                                     .clipping_near = 0.001,
                                     .X_PB = schema_X_PB,
                                     .renderer_name = "renderer",
                                     .show_rgb = true};
    ApplyCameraConfig(camera_config, &builder);
  }

  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  /* Build the simulator and run! */
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  /* Set the initial conditions for the spatula pose and the gripper finger
   positions. */
  Context<double>& mutable_root_context = simulator.get_mutable_context();
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, &mutable_root_context);

  /* Set initial finger joint positions to the "open" state. */
  const PrismaticJoint<double>& right_joint =
      plant.GetMutableJointByName<PrismaticJoint>("right_finger_sliding_joint");
  const PrismaticJoint<double>& left_joint =
      plant.GetMutableJointByName<PrismaticJoint>("left_finger_sliding_joint");
  left_joint.set_translation(&plant_context, -open_width / 2.0);
  right_joint.set_translation(&plant_context, open_width / 2.0);

  simulator.Initialize();
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace bubble_gripper
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is a demo used to showcase the following features in deformable "
      "body simulation in Drake:\n"
      "  1. frictional contact resolution among deformable bodies;\n"
      "  2. deformable geometry rendering;\n"
      "  3. fixed constraints between rigid bodies and deformable bodies.\n"
      "Note that this example only runs on Linux systems.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::bubble_gripper::do_main();
}
