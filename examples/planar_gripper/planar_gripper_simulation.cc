#include <memory>

#include <gflags/gflags.h>
#include "Eigen/src/Core/Matrix.h"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/planar_gripper/planar_gripper_common.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/manipulation/planner/robot_plan_interpolator.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace planar_gripper {
namespace {

using geometry::SceneGraph;

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::multibody::PrismaticJoint;
using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::manipulation::planner::RobotPlanInterpolator;
using drake::multibody::JointActuatorIndex;
using drake::multibody::ModelInstanceIndex;

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_time, 4.5,
              "Desired duration of the simulation in seconds.");
DEFINE_double(time_step, 1e-3,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");
DEFINE_double(keyframe_dt, 0.1, "Defines the time step between keyframes.");


/// Converts the generalized force output of the ID controller (internally using
/// a control plant with only the gripper) to the generalized force input for
/// the full simulation plant (containing gripper and object).
class MakePlantGeneralizedForceArray : public systems::LeafSystem<double> {
 public:
  MakePlantGeneralizedForceArray(const MultibodyPlant<double>& plant,
                                 ModelInstanceIndex gripper_instance)
      : plant_(plant), gripper_instance_(gripper_instance) {
    this->DeclareVectorInputPort(
        "input", systems::BasicVector<double>(plant.num_actuators()));
    this->DeclareVectorOutputPort(
        "output", systems::BasicVector<double>(plant.num_velocities()),
        &MakePlantGeneralizedForceArray::remap_output);
  }

  void remap_output(const systems::Context<double>& context,
                    systems::BasicVector<double>* output_vector) const {
    auto output_value = output_vector->get_mutable_value();
    auto input_value = this->EvalVectorInput(context, 0)->get_value();

    output_value.setZero();
    plant_.SetVelocitiesInArray(gripper_instance_, input_value, &output_value);
  }

 private:
  const MultibodyPlant<double>& plant_;
  ModelInstanceIndex gripper_instance_;
};

int DoMain() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Make and add the planar_gripper model.
  const std::string full_name =
      FindResourceOrThrow("drake/examples/planar_gripper/planar_gripper.sdf");
  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant>(FLAGS_time_step);
  auto plant_id = Parser(&plant, &scene_graph).AddModelFromFile(full_name);
  examples::planar_gripper::WeldGripperFrames<double>(&plant);

  // Adds the object to be manipulated.
  auto brick_file_name =
      FindResourceOrThrow("drake/examples/planar_gripper/planar_brick.sdf");
  auto brick_id = Parser(&plant).AddModelFromFile(brick_file_name, "object");
  const multibody::Frame<double>& brick_base_frame =
      plant.GetFrameByName("brick_base", brick_id);
  plant.WeldFrames(plant.world_frame(), brick_base_frame);

  // Create the controlled plant. Contains only the fingers (no objects).
  auto control_plant_ptr =
      std::make_shared<multibody::MultibodyPlant<double>>(FLAGS_time_step);
  MultibodyPlant<double>& control_plant = *control_plant_ptr;
  Parser(&control_plant).AddModelFromFile(full_name);
  drake::examples::planar_gripper::WeldGripperFrames<double>(&control_plant);

  plant.Finalize();
  control_plant.Finalize();

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(plant.geometry_source_is_registered());

  // Create the gains for the inverse dynamics controller. These gains were
  // chosen arbitrarily.
  const int kNumJoints = 6;
  VectorX<double> Kp(kNumJoints), Kd(kNumJoints), Ki(kNumJoints);
  Kp = VectorX<double>::Ones(kNumJoints) * 1500;
  Ki = VectorX<double>::Ones(kNumJoints) * 500;
  Kd = VectorX<double>::Ones(kNumJoints) * 500;

  auto id_controller =
      builder.AddSystem<systems::controllers::InverseDynamicsController>(
          control_plant, Kp, Ki, Kd, false);

  // Connect the ID controller.
  builder.Connect(plant.get_state_output_port(plant_id),
                  id_controller->get_input_port_estimated_state());

  // Parse the keyframes from a file and also return initial conditions.
  const std::string keyframe_path =
      "drake/examples/planar_gripper/postures.txt";
  Vector3<double> brick_ics;
  auto keyframes =
      examples::planar_gripper::ParseKeyframes(keyframe_path, &brick_ics);
  keyframes.transposeInPlace();

  // Creates the time vector for the plan interpolator.
  std::vector<double> times(keyframes.cols());
  times[0] = 0.0;
  for (int i = 1; i < keyframes.cols(); ++i) {
    times[i] = times[i - 1] + FLAGS_keyframe_dt;
  }
  std::vector<int> info(times.size(), 1);
  robotlocomotion::robot_plan_t plan{};
  plan = manipulation::planner::EncodeKeyFrames(control_plant, times, info,
                                                keyframes);

  // Publish the plan for inspection.
  examples::planar_gripper::PublishRobotPlan(plan);

  // Add the plan interpolator to the diagram.
  auto interp = builder.AddSystem<RobotPlanInterpolator>(
      control_plant_ptr, manipulation::planner::InterpolatorType::Pchip,
      FLAGS_keyframe_dt);
  auto const_value_src =
      builder.AddSystem<systems::ConstantValueSource<double>>(
          *AbstractValue::Make(plan));
  builder.Connect(const_value_src->get_output_port(0),
                  interp->get_plan_input_port());
  builder.Connect(interp->get_state_output_port(),
                  id_controller->get_input_port_desired_state());

  // The inverse dynamics controller internally uses a "controlled plant", which
  // contains the gripper model *only* (i.e., no object). Therefore, it's output
  // must be re-mapped to the input of the full "simulation plant", which
  // contains both gripper and object. The system MakePlantGeneralizedForceArray
  // fills this role.
  auto generalized_force_map =
      builder.AddSystem<MakePlantGeneralizedForceArray>(plant, plant_id);
  builder.Connect(id_controller->get_output_port_control(),
                  generalized_force_map->get_input_port(0));
  builder.Connect(generalized_force_map->get_output_port(0),
                  plant.get_applied_generalized_force_input_port());

  // Connect a constant zero vector to the actuation input port of MBP since we
  // don't use it (we use the generalized forces input).
  auto const_src = builder.AddSystem<systems::ConstantVectorSource>(
      VectorX<double>::Zero(6));
  builder.Connect(const_src->get_output_port(),
                  plant.get_actuation_input_port());

  // Connect MBP snd SG.
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions.
  VectorX<double> gripper_ics = VectorX<double>::Zero(kNumJoints * 2);
  gripper_ics.head(kNumJoints) = keyframes.block(0, 0, kNumJoints, 1);

  // Finger 1
  const RevoluteJoint<double>& sh_pin1 =
      plant.GetJointByName<RevoluteJoint>("finger1_BaseJoint");
  const RevoluteJoint<double>& el_pin1 =
      plant.GetJointByName<RevoluteJoint>("finger1_MidJoint");
  sh_pin1.set_angle(&plant_context, gripper_ics(0));
  el_pin1.set_angle(&plant_context, gripper_ics(3));

  // Finger 2
  const RevoluteJoint<double>& sh_pin2 =
      plant.GetJointByName<RevoluteJoint>("finger2_BaseJoint");
  const RevoluteJoint<double>& el_pin2 =
      plant.GetJointByName<RevoluteJoint>("finger2_MidJoint");
  sh_pin2.set_angle(&plant_context, gripper_ics(1));
  el_pin2.set_angle(&plant_context, gripper_ics(4));

  // Finger 3
  const RevoluteJoint<double>& sh_pin3 =
      plant.GetJointByName<RevoluteJoint>("finger3_BaseJoint");
  const RevoluteJoint<double>& el_pin3 =
      plant.GetJointByName<RevoluteJoint>("finger3_MidJoint");
  sh_pin3.set_angle(&plant_context, gripper_ics(2));
  el_pin3.set_angle(&plant_context, gripper_ics(5));

  // Set the box initial conditions.
  const PrismaticJoint<double>& y_translate =
      plant.GetJointByName<PrismaticJoint>("brick_translate_y_joint");
  const PrismaticJoint<double>& z_translate =
      plant.GetJointByName<PrismaticJoint>("brick_translate_z_joint");
  const RevoluteJoint<double>& x_revolute =
      plant.GetJointByName<RevoluteJoint>("brick_revolute_x_joint");
  y_translate.set_translation(&plant_context, brick_ics(0));
  z_translate.set_translation(&plant_context, brick_ics(1));
  x_revolute.set_angle(&plant_context, brick_ics(2));

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  // Initialize the interpolator.
  auto& dcontext = simulator.get_mutable_context();
  auto& interpolator_context =
      diagram->GetMutableSubsystemContext(*interp, &dcontext);
  interp->Initialize(0 /* plan start time */, gripper_ics.head(6),
                     &interpolator_context.get_mutable_state());

  simulator.set_publish_every_time_step(false);
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
