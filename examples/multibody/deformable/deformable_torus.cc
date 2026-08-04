#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/examples/multibody/deformable/deformable_common.h"
#include "drake/examples/multibody/deformable/parallel_gripper_controller.h"
#include "drake/examples/multibody/deformable/point_source_force_field.h"
#include "drake/examples/multibody/deformable/suction_cup_controller.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(simulation_time, 12.0, "Desired duration of the simulation [s].");
DEFINE_double(realtime_rate, 1.0, "Desired real time rate.");
DEFINE_double(time_step, 1e-2,
              "Discrete time step for the system [s]. Must be positive.");
DEFINE_string(gripper, "parallel",
              "Type of gripper used to pick up the deformable torus. Options "
              "are: 'parallel' and 'suction'.");
DEFINE_string(contact_approximation, "lagged",
              "Type of convex contact approximation. See "
              "multibody::DiscreteContactApproximation for details. Options "
              "are: 'sap', 'lagged', and 'similar'.");

using drake::examples::deformable::ParallelGripperController;
using drake::examples::deformable::PointSourceForceField;
using drake::examples::deformable::SuctionCupController;
using drake::geometry::AddContactMaterial;
using drake::geometry::Capsule;
using drake::geometry::IllustrationProperties;
using drake::geometry::ProximityProperties;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::systems::Context;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace drake {
namespace examples {
namespace deformable {
namespace {

/* Adds a suction gripper to the given MultibodyPlant and assign
 `proximity_props` to all the registered collision geometries. Returns the
 ModelInstanceIndex of the gripper model. */
ModelInstanceIndex AddSuctionGripper(
    MultibodyPlant<double>* plant, const ProximityProperties& proximity_props) {
  const double radius = 0.02;
  const double length = 0.1;
  const auto M = SpatialInertia<double>::SolidCapsuleWithMass(
      0.1, radius, length, Vector3d::UnitZ());
  ModelInstanceIndex model_instance = plant->AddModelInstance("instance");
  const auto& body = plant->AddRigidBody("cup_body", model_instance, M);
  const Capsule capsule{radius, length};
  IllustrationProperties cup_illustration_props;
  cup_illustration_props.AddProperty("phong", "diffuse",
                                     Vector4d(0.9, 0.1, 0.1, 0.8));
  plant->RegisterVisualGeometry(body, RigidTransformd::Identity(), capsule,
                                "cup_visual", cup_illustration_props);
  /* Add a visual hint for the center of the suction force source. */
  const Sphere sphere{0.0075};
  IllustrationProperties source_illustration_props;
  source_illustration_props.AddProperty("phong", "diffuse",
                                        Vector4d(0.1, 0.9, 0.1, 0.8));
  plant->RegisterVisualGeometry(body, RigidTransformd(Vector3d(0, 0, -0.07)),
                                sphere, "source_visual",
                                source_illustration_props);
  plant->RegisterCollisionGeometry(body, RigidTransformd::Identity(), capsule,
                                   "cup_collision", proximity_props);
  /* Adds an actuated joint between the suction cup body and the world. */
  const RigidTransformd X_WF(Vector3d(0.04, 0, -0.05));
  const auto& prismatic_joint = plant->AddJoint<PrismaticJoint>(
      "translate_z_joint", plant->world_body(), X_WF, body, std::nullopt,
      Vector3d::UnitZ());
  plant->GetMutableJointByName<PrismaticJoint>("translate_z_joint")
      .set_default_translation(0.5);
  const auto actuator_index =
      plant->AddJointActuator("prismatic joint actuator", prismatic_joint)
          .index();
  plant->get_mutable_joint_actuator(actuator_index)
      .set_controller_gains({1e4, 1});

  return model_instance;
}

/* Adds a parallel gripper to the given MultibodyPlant. Returns the
 ModelInstanceIndex of the gripper model. */
ModelInstanceIndex AddParallelGripper(MultibodyPlant<double>* plant) {
  // TODO(xuchenhan-tri): Consider using a schunk gripper from the manipulation
  // station instead.
  Parser parser(plant);
  ModelInstanceIndex model_instance = parser.AddModelsFromUrl(
      "package://drake/examples/multibody/deformable/models/simple_gripper.sdf")
                                          [0];
  /* Get joints so that we can set initial conditions. */
  PrismaticJoint<double>& left_slider =
      plant->GetMutableJointByName<PrismaticJoint>("left_slider");
  PrismaticJoint<double>& right_slider =
      plant->GetMutableJointByName<PrismaticJoint>("right_slider");
  /* Initialize the gripper in an "open" position. */
  const double kInitialWidth = 0.085;
  left_slider.set_default_translation(-kInitialWidth / 2.0);
  right_slider.set_default_translation(kInitialWidth / 2.0);

  return model_instance;
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_time_step;
  plant_config.discrete_contact_approximation = FLAGS_contact_approximation;

  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);
  RegisterRigidGround(&plant);

  /* Minimum required proximity properties for rigid bodies to interact with
   deformable bodies.
   1. A valid Coulomb friction coefficient, and
   2. A resolution hint. (Rigid bodies need to be tessellated so that collision
   queries can be performed against deformable geometries.) The value dictates
   how fine the mesh used to represent the rigid collision geometry is. */
  ProximityProperties rigid_proximity_props;
  /* Set the friction coefficient close to that of rubber against rubber. */
  const CoulombFriction<double> surface_friction(1.15, 1.15);
  AddContactMaterial({}, {}, surface_friction, &rigid_proximity_props);
  rigid_proximity_props.AddProperty(geometry::internal::kHydroGroup,
                                    geometry::internal::kRezHint, 0.01);

  /* Add a parallel gripper or a suction gripper depending on the runtime flag.
   */
  const bool use_suction = FLAGS_gripper == "suction";
  ModelInstanceIndex gripper_instance =
      use_suction ? AddSuctionGripper(&plant, rigid_proximity_props)
                  : AddParallelGripper(&plant);

  /* Set up a deformable torus. */
  Parser parser(&plant);
  parser.AddModelsFromUrl(
      "package://drake/examples/multibody/deformable/models/"
      "deformable_torus.sdf");

  /* Add an external suction force if using a suction gripper. */
  const PointSourceForceField* suction_force_ptr{nullptr};
  if (use_suction) {
    auto suction_force = std::make_unique<PointSourceForceField>(
        plant, plant.GetBodyByName("cup_body"), Vector3d(0, 0, -0.07), 0.1);
    suction_force_ptr = suction_force.get();
    plant.mutable_deformable_model().AddExternalForce(std::move(suction_force));
  }

  /* All rigid and deformable models have been added. Finalize the plant. */
  plant.Finalize();

  /* Add a visualizer that emits LCM messages for visualization. */
  geometry::DrakeVisualizerParams params;
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, nullptr,
                                           params);

  /* Add a controller appropriate for the type of gripper. */
  if (use_suction) {
    const double kInitialHeight = 0.5;
    const double kStartSuctionHeight =
        0.15;  // The height at which to turn on suction.
    const double kApproachTime = 3.0;      // Time to start the action
    const double kStartSuctionTime = 4.0;  // Time to turn on suction
    const double kRetrieveTime = 6.0;      // Time to retrieve the gripper
    const double kDropTime = 9.0;          // Time to turn off suction
    const auto& suction = *builder.AddSystem<SuctionCupController>(
        kInitialHeight, kStartSuctionHeight, kApproachTime, kStartSuctionTime,
        kRetrieveTime, kDropTime);
    builder.Connect(suction.maximum_force_density_port(),
                    suction_force_ptr->maximum_force_density_input_port());
    builder.Connect(suction.desired_state_output_port(),
                    plant.get_desired_state_input_port(gripper_instance));
  } else {
    /* Set the width between the fingers for open and closed states as well as
     the height to which the gripper lifts the deformable torus. */
    const double kOpenWidth = 0.088;
    const double kClosedWidth = 0.023;
    const double kLiftedHeight = 0.18;
    const auto& control = *builder.AddSystem<ParallelGripperController>(
        kOpenWidth, kClosedWidth, kLiftedHeight);
    builder.Connect(control.get_output_port(),
                    plant.get_desired_state_input_port(gripper_instance));
  }

  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  /* Build the simulator and run! */
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.Initialize();
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace deformable
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is a demo used to showcase deformable body simulations in Drake. "
      "A parallel (or suction) gripper grasps a deformable torus on the "
      "ground, lifts it up, and then drops it back on the ground. "
      "Launch meldis before running this example. "
      "Refer to README for instructions on meldis as well as optional flags.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::deformable::do_main();
}
