/// @file
///
/// This demo simulates a planar-gripper (three two-degree-of-freedom fingers
/// moving in a plane) which reorients a brick through contact-interactions.
/// The simulation interpolates a sequence of joint poses (keyframes) for the
/// fingers, where each keyframe represents a static-equilibrium condition for
/// the brick (i.e., the net wrench on the brick is zero). In this example, we
/// simulate a playback of these keyframes at an arbitrarily set speed.
///
/// The planar-gripper coordinate frame is illustrated in
/// `planar_gripper_common.h`. Users have the option to either orient the
/// gravity vector to point along the -Gz axis, i.e., simulating the case when
/// the planar-gripper is stood up vertically, or have gravity point along the
/// -Gx axis, i.e., simulating the case when the planar-gripper is laid down
/// flat on the floor.
///
/// To support the vertical case (gravity acting along -Gz) in hardware, we use
/// a plexiglass lid (ceiling) that is opposite the planar-gripper floor in
/// order to keep the brick's motion constrained to the Gy-Gz plane. That is,
/// when the lid is closed the brick is "squeezed" between the ceiling and floor
/// and is physically fixed in the Gx-axis due to contact with these surfaces.
/// For simulation, we mimic this contact interaction by fixing the amount by
/// which the brick geometry penetrates the floor geometry (without considering
/// the ceiling), and can specify this penetration depth via the flag
/// `brick_floor_penetration'. To enforce zero contact between the brick and
/// floor, set this flag to zero.
///
/// For the horizontal case in hardware, gravity (acting along -Gx) keeps the
/// brick's motion constrained to lie in the Gy-Gz plane (no ceiling required),
/// and therefore the plexiglass lid is left open. This means surface contact
/// only occurs between the brick and the floor. In simulation, we define an
/// additional prismatic degree of freedom for the brick along the Gx axis, such
/// that the brick's position along Gx is determined by gravitational and floor
/// contact forces acting on the brick. In this case, the
/// `brick_floor_penetration` flag specifies only the initial brick/floor
/// penetration depth.
///
/// @Note: The keyframes contained in `postures.txt` are strictly for simulating
///        the vertical case. Using these keyframes to simulate the horizontal
///        case may cause the simulation to fail.

// TODO(rcory) Include a README.md that explains the use cases for this
//  example.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/planar_gripper/planar_gripper_common.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace planar_gripper {
namespace {

using geometry::SceneGraph;
using multibody::ConnectContactResultsToDrakeVisualizer;
using multibody::JointActuatorIndex;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using Eigen::Vector3d;

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_time, 4.5,
              "Desired duration of the simulation in seconds.");
DEFINE_double(time_step, 1e-3,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");
DEFINE_double(
    keyframe_dt, 0.1,
    "Defines the time step to take between keyframes. Note that keyframe "
    "data in postures.txt contains static equilibrium poses and we "
    "play these back at an arbitrary speed for this simulation.");
DEFINE_double(penetration_allowance, 1e-3,
              "The contact penetration allowance.");
DEFINE_double(floor_coef_static_friction, 0.5,
              "The floor's coefficient of static friction");
DEFINE_double(floor_coef_kinetic_friction, 0.5,
              "The floor's coefficient of kinetic friction");
DEFINE_double(brick_floor_penetration, 1e-5,
              "Determines how much the brick should penetrate the floor "
              "(in meters). When simulating the vertical case this penetration "
              "distance will remain fixed.");
DEFINE_string(orientation, "vertical",
              "The orientation of the planar gripper. Options are {vertical, "
              "horizontal}.");
DEFINE_bool(visualize_contacts, false,
            "Visualize contacts in Drake visualizer.");

/// Adds a floor to the simulation, modeled as a thin cylinder.
void AddFloor(MultibodyPlant<double>* plant,
              const SceneGraph<double>& scene_graph) {
  // Get info for the brick from the SceneGraph inspector. This is used to
  // determine placement of the floor in order to achieve the specified
  // brick/floor penetration.
  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph.model_inspector();

  // The brick model includes four small sphere collisions at the bottom four
  // corners of the box collision. These four spheres (and not the box) are
  // intended to make contact with the floor. Here we extract the height of
  // these spheres in order to weld the floor at the appropriate height, such
  // that the initial box/floor penetration is given by the flag
  // brick_floor_penetration.
  const geometry::Shape& sphere_shape =
      inspector.GetShape(inspector.GetGeometryIdByName(
          plant->GetBodyFrameIdOrThrow(
              plant->GetBodyByName("brick_link").index()),
          geometry::Role::kProximity, "brick::sphere1_collision"));
  const double sphere_radius =
      dynamic_cast<const geometry::Sphere&>(sphere_shape).get_radius();
  const math::RigidTransformd X_WS =
      inspector.GetPoseInFrame(inspector.GetGeometryIdByName(
          plant->GetBodyFrameIdOrThrow(
              plant->GetBodyByName("brick_link").index()),
          geometry::Role::kProximity, "brick::sphere1_collision"));

  const double kFloorHeight = 0.001;
  const double kSphereTipXOffset = X_WS.translation()(0) - sphere_radius;
  const drake::multibody::CoulombFriction<double> coef_friction_floor(
      FLAGS_floor_coef_static_friction, FLAGS_floor_coef_kinetic_friction);
  const math::RigidTransformd X_WF(
      Eigen::AngleAxisd(M_PI_2, Vector3d::UnitY()),
      Vector3d(kSphereTipXOffset - (kFloorHeight / 2.0) +
                   FLAGS_brick_floor_penetration, 0, 0));
  const Vector4<double> black(0.2, 0.2, 0.2, 1.0);
  plant->RegisterVisualGeometry(plant->world_body(), X_WF,
                                geometry::Cylinder(.125, kFloorHeight),
                                "FloorVisualGeometry", black);
  plant->RegisterCollisionGeometry(
      plant->world_body(), X_WF, geometry::Cylinder(.125, kFloorHeight),
      "FloorCollisionGeometry", coef_friction_floor);
}

/// Converts the generalized force output of the ID controller (internally using
/// a control plant with only the gripper) to the generalized force input for
/// the full simulation plant (containing gripper and brick).
class ControlToSimPlantForceConverter : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ControlToSimPlantForceConverter);
  ControlToSimPlantForceConverter(const MultibodyPlant<double>* plant,
                                 ModelInstanceIndex gripper_instance)
      : plant_(plant), gripper_instance_(gripper_instance) {
    DRAKE_DEMAND(plant != nullptr);
    this->DeclareVectorInputPort(
        "input", systems::BasicVector<double>(plant->num_actuators()));
    this->DeclareVectorOutputPort(
        "output", systems::BasicVector<double>(plant->num_velocities()),
        &ControlToSimPlantForceConverter::remap_output);
  }

  void remap_output(const systems::Context<double>& context,
                    systems::BasicVector<double>* output_vector) const {
    auto output_value = output_vector->get_mutable_value();
    auto input_value = this->EvalVectorInput(context, 0)->get_value();

    output_value.setZero();
    plant_->SetVelocitiesInArray(gripper_instance_, input_value, &output_value);
  }

 private:
  const MultibodyPlant<double>* plant_;
  const ModelInstanceIndex gripper_instance_;
};

int DoMain() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  Vector3d gravity;

  // Make and add the planar_gripper model.
  const std::string full_name =
      FindResourceOrThrow("drake/examples/planar_gripper/planar_gripper.sdf");
  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant>(FLAGS_time_step);
  const ModelInstanceIndex gripper_index =
      Parser(&plant, &scene_graph).AddModelFromFile(full_name);
  WeldGripperFrames<double>(&plant);

  // Adds the brick to be manipulated.
  const std::string brick_file_name =
      FindResourceOrThrow("drake/examples/planar_gripper/planar_brick.sdf");
  const ModelInstanceIndex brick_index =
      Parser(&plant).AddModelFromFile(brick_file_name, "brick");

  // When the planar-gripper is welded via WeldGripperFrames(), motion always
  // lies in the world Y-Z plane (because the planar-gripper frame is aligned
  // with the world frame). Therefore, gravity can either point along the world
  // -Z axis (vertical case), or world -X axis (horizontal case).
  if (FLAGS_orientation == "vertical") {
    const multibody::Frame<double>& brick_base_frame =
        plant.GetFrameByName("brick_base", brick_index);
    plant.WeldFrames(plant.world_frame(), brick_base_frame);
    gravity = Vector3d(
        0, 0, -multibody::UniformGravityFieldElement<double>::kDefaultStrength);
  } else if (FLAGS_orientation == "horizontal") {
    plant.AddJoint<PrismaticJoint>(
        "brick_translate_x_joint",
        plant.world_body(), std::nullopt,
        plant.GetBodyByName("brick_base"), std::nullopt,
        Vector3d::UnitX());
    gravity = Vector3d(
        -multibody::UniformGravityFieldElement<double>::kDefaultStrength, 0, 0);
  } else {
    throw std::logic_error("Unrecognized 'orientation' flag.");
  }

  // Create the controlled plant. Contains only the fingers (no bricks).
  MultibodyPlant<double> control_plant(FLAGS_time_step);
  Parser(&control_plant).AddModelFromFile(full_name);
  WeldGripperFrames<double>(&control_plant);

  // Adds a thin floor that can provide friction against the brick.
  AddFloor(&plant, scene_graph);

  // Finalize the simulation and control plants.
  plant.Finalize();
  control_plant.Finalize();

  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.mutable_gravity_field().set_gravity_vector(gravity);
  control_plant.mutable_gravity_field().set_gravity_vector(gravity);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(plant.geometry_source_is_registered());

  // Create the gains for the inverse dynamics controller. These gains were
  // chosen arbitrarily.
  const int kNumJoints = 6;
  Vector<double, kNumJoints> Kp, Kd, Ki;
  Kp.setConstant(1500); Kd.setConstant(500); Ki.setConstant(500);

  auto id_controller =
      builder.AddSystem<systems::controllers::InverseDynamicsController>(
          control_plant, Kp, Ki, Kd, false);

  // Connect the ID controller.
  builder.Connect(plant.get_state_output_port(gripper_index),
                  id_controller->get_input_port_estimated_state());

  // Parse the keyframes from a file and also return initial brick pose. The
  // brick's pose consists of {y_position, z_position, x_rotation_angle}.
  const std::string keyframe_path =
      "drake/examples/planar_gripper/postures.txt";
  MatrixX<double> keyframes;
  std::map<std::string, int> finger_joint_name_to_col_index_map;
  Vector3<double>
      brick_initial_2D_pose_G;  // (y, z, theta) where (y,z) are expressed in G.
  std::tie(keyframes, finger_joint_name_to_col_index_map) =
      ParseKeyframes(keyframe_path, &brick_initial_2D_pose_G);
  // Here we assume the gripper frame G is aligned with the world frame W, e.g.,
  // as given by calling WeldGripperFrames(). We enforce this by checking here.
  DRAKE_DEMAND(
      X_WGripper().IsExactlyEqualTo(math::RigidTransform<double>::Identity()));

  // Creates the time vector for the plan interpolator.
  Eigen::VectorXd times = Eigen::VectorXd::Zero(keyframes.cols());
  for (int i = 1; i < keyframes.cols(); ++i) {
    times(i) = i * FLAGS_keyframe_dt;
  }
  const auto pp =
      trajectories::PiecewisePolynomial<double>::Pchip(times, keyframes);
  auto traj_src = builder.AddSystem<systems::TrajectorySource<double>>(
      pp, 1 /* with one derivative */);
  builder.Connect(traj_src->get_output_port(),
                  id_controller->get_input_port_desired_state());

  // The inverse dynamics controller internally uses a "controlled plant", which
  // contains the gripper model *only* (i.e., no brick). Therefore, its output
  // must be re-mapped to the input of the full "simulation plant", which
  // contains both gripper and brick. The system
  // ControlToSimPlantForceConverter fills this role.
  auto generalized_force_map =
      builder.AddSystem<ControlToSimPlantForceConverter>(&plant, gripper_index);
  builder.Connect(*id_controller, *generalized_force_map);
  builder.Connect(generalized_force_map->get_output_port(0),
                  plant.get_applied_generalized_force_input_port());

  // Connect a constant zero vector to the actuation input port of MBP since we
  // don't use it (we use the generalized forces input).
  auto const_src = builder.AddSystem<systems::ConstantVectorSource>(
      VectorX<double>::Zero(kNumJoints));
  builder.Connect(const_src->get_output_port(),
                  plant.get_actuation_input_port());

  // Connect MBP snd SG.
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  lcm::DrakeLcm lcm;
  geometry::ConnectDrakeVisualizer(&builder, scene_graph, &lcm);

  // Publish contact results for visualization.
  if (FLAGS_visualize_contacts) {
    ConnectContactResultsToDrakeVisualizer(&builder, plant, &lcm);
  }

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Create the initial condition vector. Set initial joint velocities to zero.
  VectorX<double> gripper_initial_conditions =
      VectorX<double>::Zero(kNumJoints * 2);
  gripper_initial_conditions.head(kNumJoints) =
      keyframes.block(0, 0, kNumJoints, 1);

  // All fingers consist of two joints: a base joint and a mid joint.
  // Set the initial finger joint positions.
  const int kNumFingers = 3;
  for (int i = 0; i < kNumFingers; i++) {
    std::string finger = "finger" + std::to_string(i+1);
    const RevoluteJoint<double>& base_pin =
        plant.GetJointByName<RevoluteJoint>(finger + "_BaseJoint");
    const RevoluteJoint<double>& mid_pin =
        plant.GetJointByName<RevoluteJoint>(finger + "_MidJoint");
    int base_index = finger_joint_name_to_col_index_map[finger + "_BaseJoint"];
    int mid_index = finger_joint_name_to_col_index_map[finger + "_MidJoint"];
    base_pin.set_angle(&plant_context, gripper_initial_conditions(base_index));
    mid_pin.set_angle(&plant_context, gripper_initial_conditions(mid_index));
  }

  // Set the brick's initial conditions.
  const PrismaticJoint<double>& y_translate =
      plant.GetJointByName<PrismaticJoint>("brick_translate_y_joint");
  const PrismaticJoint<double>& z_translate =
      plant.GetJointByName<PrismaticJoint>("brick_translate_z_joint");
  const RevoluteJoint<double>& x_revolute =
      plant.GetJointByName<RevoluteJoint>("brick_revolute_x_joint");
  y_translate.set_translation(&plant_context, brick_initial_2D_pose_G(0));
  z_translate.set_translation(&plant_context, brick_initial_2D_pose_G(1));
  x_revolute.set_angle(&plant_context, brick_initial_2D_pose_G(2));

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("A simple planar gripper example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::planar_gripper::DoMain();
}
