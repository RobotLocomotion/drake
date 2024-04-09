#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
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
DEFINE_double(E, 3e4, "Young's modulus of the deformable body [Pa].");
DEFINE_double(nu, 0.4, "Poisson's ratio of the deformable body, unitless.");
DEFINE_double(density, 1e3,
              "Mass density of the deformable body [kg/m³]. We observe that "
              "density above 2400 kg/m³ makes the torus too heavy to be picked "
              "up by the suction gripper.");
DEFINE_double(beta, 0.01,
              "Stiffness damping coefficient for the deformable body [1/s].");
DEFINE_string(gripper, "parallel",
              "Type of gripper used to pick up the deformable torus. Options "
              "are: 'parallel' and 'suction'.");
DEFINE_string(contact_approximation, "lagged",
              "Type of convex contact approximation. See "
              "multibody::DiscreteContactApproximation for details. Options "
              "are: 'sap', 'lagged', and 'similar'.");
DEFINE_double(
    contact_damping, 10.0,
    "Hunt and Crossley damping for the deformable body, only used when "
    "'contact_approximation' is set to 'lagged' or 'similar' [s/m].");

using drake::examples::deformable::ParallelGripperController;
using drake::examples::deformable::PointSourceForceField;
using drake::examples::deformable::SuctionCupController;
using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::Ellipsoid;
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::Mesh;
using drake::geometry::ProximityProperties;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::CoulombFriction;
using drake::multibody::DeformableBodyId;
using drake::multibody::DeformableModel;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::fem::DeformableBodyConfig;
using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace drake {
namespace examples {
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

/* Adds a parallel gripper to the given MultibodyPlant and assign
 `proximity_props` to all the registered collision geometries. Returns the
 ModelInstanceIndex of the gripper model. */
ModelInstanceIndex AddParallelGripper(
    MultibodyPlant<double>* plant, const ProximityProperties& proximity_props) {
  // TODO(xuchenhan-tri): Consider using a schunk gripper from the manipulation
  // station instead.
  Parser parser(plant);
  ModelInstanceIndex model_instance = parser.AddModelsFromUrl(
      "package://drake/examples/multibody/deformable/models/simple_gripper.sdf")
                                          [0];
  /* Add collision geometries. */
  const RigidTransformd X_BG =
      RigidTransformd(math::RollPitchYawd(M_PI_2, 0, 0), Vector3d::Zero());
  const RigidBody<double>& left_finger = plant->GetBodyByName("left_finger");
  const RigidBody<double>& right_finger = plant->GetBodyByName("right_finger");
  /* The size of the fingers is set to match the visual geometries in
   simple_gripper.sdf. */
  Capsule capsule(0.01, 0.08);
  plant->RegisterCollisionGeometry(left_finger, X_BG, capsule,
                                   "left_finger_collision", proximity_props);
  plant->RegisterCollisionGeometry(right_finger, X_BG, capsule,
                                   "right_finger_collision", proximity_props);
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
  /* Set up a ground. */
  Box ground{4, 4, 4};
  const RigidTransformd X_WG(Eigen::Vector3d{0, 0, -2});
  plant.RegisterCollisionGeometry(plant.world_body(), X_WG, ground,
                                  "ground_collision", rigid_proximity_props);
  IllustrationProperties illustration_props;
  illustration_props.AddProperty("phong", "diffuse",
                                 Vector4d(0.7, 0.5, 0.4, 0.8));
  plant.RegisterVisualGeometry(plant.world_body(), X_WG, ground,
                               "ground_visual", std::move(illustration_props));

  /* Add a parallel gripper or a suction gripper depending on the runtime flag.
   */
  const bool use_suction = FLAGS_gripper == "suction";
  ModelInstanceIndex gripper_instance =
      use_suction ? AddSuctionGripper(&plant, rigid_proximity_props)
                  : AddParallelGripper(&plant, rigid_proximity_props);

  /* Set up a deformable torus. */
  auto owned_deformable_model =
      std::make_unique<DeformableModel<double>>(&plant);

  DeformableBodyConfig<double> deformable_config;
  deformable_config.set_youngs_modulus(FLAGS_E);
  deformable_config.set_poissons_ratio(FLAGS_nu);
  deformable_config.set_mass_density(FLAGS_density);
  deformable_config.set_stiffness_damping_coefficient(FLAGS_beta);

  const std::string torus_vtk = FindResourceOrThrow(
      "drake/examples/multibody/deformable/models/torus.vtk");
  /* Load the geometry and scale it down to 65% (to showcase the scaling
   capability and to make the torus suitable for grasping by the gripper). */
  const double scale = 0.65;
  auto torus_mesh = std::make_unique<Mesh>(torus_vtk, scale);
  /* Minor diameter of the torus inferred from the vtk file. */
  const double kL = 0.09 * scale;
  /* Set the initial pose of the torus such that its bottom face is touching the
   ground. */
  const RigidTransformd X_WB(Vector3<double>(0.0, 0.0, kL / 2.0));
  auto torus_instance = std::make_unique<GeometryInstance>(
      X_WB, std::move(torus_mesh), "deformable_torus");

  /* Minimally required proximity properties for deformable bodies: A valid
   Coulomb friction coefficient. */
  ProximityProperties deformable_proximity_props;
  AddContactMaterial(FLAGS_contact_damping, {}, surface_friction,
                     &deformable_proximity_props);
  torus_instance->set_proximity_properties(deformable_proximity_props);

  /* Registration of all deformable geometries ostensibly requires a resolution
   hint parameter that dictates how the shape is tessellated. In the case of a
   `Mesh` shape, the resolution hint is unused because the shape is already
   tessellated. */
  // TODO(xuchenhan-tri): Though unused, we still asserts the resolution hint is
  // positive. Remove the requirement of a resolution hint for meshed shapes.
  const double unused_resolution_hint = 1.0;
  owned_deformable_model->RegisterDeformableBody(
      std::move(torus_instance), deformable_config, unused_resolution_hint);

  /* Add an external suction force if using a suction gripper. */
  const PointSourceForceField* suction_force_ptr{nullptr};
  if (use_suction) {
    auto suction_force = std::make_unique<PointSourceForceField>(
        plant, plant.GetBodyByName("cup_body"), Vector3d(0, 0, -0.07), 0.1);
    suction_force_ptr = suction_force.get();
    owned_deformable_model->AddExternalForce(std::move(suction_force));
  }

  const DeformableModel<double>* deformable_model =
      owned_deformable_model.get();
  plant.AddPhysicalModel(std::move(owned_deformable_model));

  /* All rigid and deformable models have been added. Finalize the plant. */
  plant.Finalize();

  /* It's essential to connect the vertex position port in DeformableModel to
   the source configuration port in SceneGraph when deformable bodies are
   present in the plant. */
  builder.Connect(
      deformable_model->vertex_positions_port(),
      scene_graph.get_source_configuration_port(plant.get_source_id().value()));

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
    const double kOpenWidth = kL * 1.5;
    const double kClosedWidth = kL * 0.4;
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
  return drake::examples::do_main();
}
