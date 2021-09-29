/* @file
A Strandbeest demo demonstrating the use of a linear bushing as
a way to model kinematic loops. It shows:
  - How to model a parameterized Strandbeest  in SDF.
  - Use the `multibody::Parser` to load a model from an SDF file into a
    MultibodyPlant.
  - Model revolute joints with a `multibody::LinearBushingRollPitchYaw` to
    model closed kinematic chains.
  - Parsing custom drake:linear_bushing_rpy tags.
  - Computing inverse kinematics for the Strandbeest model.

  Refer to README.md for more details on how to run and modify this example.
*/
#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using multibody::BodyIndex;
using multibody::ForceElementIndex;
using multibody::InverseKinematics;
using multibody::Joint;
using multibody::LinearBushingRollPitchYaw;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::RevoluteJoint;
using solvers::Solve;
using systems::BasicVector;
using systems::Context;
using systems::DiagramBuilder;
using systems::LeafSystem;
using systems::OutputPort;
using systems::OutputPortIndex;
using systems::Simulator;

namespace examples {
namespace multibody {
namespace strandbeest {
namespace {

DEFINE_double(simulation_time, 5.0, "Duration of the simulation in seconds.");

DEFINE_double(initial_velocity, 1.0,
              "Initial velocity of the crossbar_crank joint.");

DEFINE_double(mbt_dt, 0.0,
              "Discrete time step. Defaults to 0.0 for a continuous system.");

DEFINE_double(penetration_allowance, 1.0e-2, "MBP penetration allowance.");

DEFINE_double(stiction_tolerance, 5.0e-2, "MBP stiction tolerance.");

// A simple proportional controller to keep the angular velocity of the joint
// at a desired rate.
template <typename T>
class DesiredVelocityMotor final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DesiredVelocityMotor)

  DesiredVelocityMotor(const MultibodyPlant<T>& plant, const Joint<T>& joint,
                       double omega_desired, double proportional)
      : velocity_index_(plant.num_positions() + joint.velocity_start()),
        omega_desired_(omega_desired),
        kProportional_(proportional) {
    this->DeclareInputPort("Plant state", systems::kVectorValued,
                           plant.num_multibody_states());
    output_index_ = this->DeclareVectorOutputPort(
                            "Torque", 1, &DesiredVelocityMotor<T>::CalcTorque)
                        .get_index();
  }

  /// Returns the output port on which the sum is presented.
  const OutputPort<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(output_index_);
  }

 private:
  int velocity_index_;
  double omega_desired_;
  OutputPortIndex output_index_;
  double kProportional_;

  // Calculates the torque on the motor proportional to the difference in
  // desired angular velocity and given angular velocity.
  void CalcTorque(const Context<T>& context, BasicVector<T>* torque) const {
    const T omega = this->get_input_port(0).Eval(context)[velocity_index_];
    const T tau = kProportional_ * (omega_desired_ - omega);
    (*torque)[0] = tau;
  }
};

int do_main() {
  // Build a generic MultibodyPlant and SceneGraph.
  DiagramBuilder<double> builder;

  auto [strandbeest, scene_graph] = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(FLAGS_mbt_dt));

  // Make and add the strandbeest model from an SDF model.
  const std::string relative_name =
      "drake/examples/multibody/strandbeest/Strandbeest.urdf";
  const std::string full_name = FindResourceOrThrow(relative_name);

  Parser parser(&strandbeest);
  parser.AddModelFromFile(full_name);

  // We are done defining the model. Finalize.
  strandbeest.Finalize();

  // Calculate the total mass of the model. Do not include the mass of
  // BodyIndex(0) a.k.a. the world.
  double total_mass = 0;
  for (BodyIndex body_index(1); body_index < strandbeest.num_bodies();
       ++body_index) {
    const auto& body = strandbeest.get_body(body_index);
    total_mass += body.get_default_mass();
  }

  // Set the penetration allowance and stiction tolerance to values that make
  // sense for the scale of our simulation.
  strandbeest.set_penetration_allowance(FLAGS_penetration_allowance);
  strandbeest.set_stiction_tolerance(FLAGS_stiction_tolerance);

  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  ConnectContactResultsToDrakeVisualizer(&builder, strandbeest, scene_graph);

  // Create a DesiredVelocityMotor where the proportional term is directly
  // proportional to the mass of the model.
  RevoluteJoint<double>& crank_joint_actuated =
      strandbeest.GetMutableJointByName<RevoluteJoint>("joint_crossbar_crank");
  auto torque_source = builder.AddSystem<DesiredVelocityMotor>(
      strandbeest, crank_joint_actuated, FLAGS_initial_velocity, total_mass);

  torque_source->set_name("Applied Torque");
  builder.Connect(torque_source->get_output_port(),
                  strandbeest.get_actuation_input_port());
  builder.Connect(strandbeest.get_state_output_port(),
                  torque_source->get_input_port(0));
  auto diagram = builder.Build();

  // Create a context for the diagram and extract the context for the
  // strandbeest model.
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& strandbeest_context =
      strandbeest.GetMyMutableContextFromRoot(diagram_context.get());

  // Set up an initial condition to fix the floating body (crossbar)
  // for inverse kinematics.
  VectorXd lower = strandbeest.GetPositionLowerLimits();
  VectorXd upper = strandbeest.GetPositionUpperLimits();

  // Fix the orientation of the floating body (crossbar) to the unit
  // quaternion.
  lower.head<4>() = Eigen::Vector4d(1, 0, 0, 0);
  upper.head<4>() = Eigen::Vector4d(1, 0, 0, 0);

  // Fix the translation of the floating body (crossbar) to (-2, 0, 1.35).
  // Place the Strandbeest model slightly above the ground plane box so it is
  // not in collision and also does not have to fall far to land.
  lower.segment<3>(4) = Eigen::Vector3d(-2, 0, 1.35);
  upper.segment<3>(4) = Eigen::Vector3d(-2, 0, 1.35);

  // Fix the crank shaft to top dead center.
  const RevoluteJoint<double>& joint_crossbar_crank =
      strandbeest.GetJointByName<RevoluteJoint>("joint_crossbar_crank");
  const int start_position = joint_crossbar_crank.position_start();

  lower[start_position] = 0;
  upper[start_position] = 0;

  // Create an InverseKinematics program with our strandbeest context but
  // without it parsing the default position limits as constraints.
  InverseKinematics ik(strandbeest, &strandbeest_context, false);

  // Add our custom position constraints that fix the floating body (crossbar).
  ik.get_mutable_prog()->AddBoundingBoxConstraint(lower, upper, ik.q());

  // Add a position constraint for each bushing element. The origins of the two
  // frames defining the bushing should be coincident, so we add an equality
  // constraint for those poses. Skip the 0th force element (UniformGravity).
  for (ForceElementIndex bushing_index(1);
       bushing_index < strandbeest.num_force_elements(); ++bushing_index) {
    const LinearBushingRollPitchYaw<double>& bushing =
        strandbeest.GetForceElement<LinearBushingRollPitchYaw>(bushing_index);

    ik.AddPointToPointDistanceConstraint(
        bushing.frameA(), Eigen::Vector3d(0, 0, 0), bushing.frameC(),
        Eigen::Vector3d(0, 0, 0), 0, 0);
  }

  // Solve the IK. The solved positions will be stored in the context passed
  // to the InverseKinematics constructor.
  Solve(ik.prog());

  // Create a simulator and run the simulation.
  std::unique_ptr<Simulator<double>> simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  simulator->AdvanceTo(FLAGS_simulation_time);

  // Print some useful statistics.
  PrintSimulatorStatistics(*simulator);

  return 0;
}  // namespace

}  // namespace
}  // namespace strandbeest
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A demo showing the Strandbeest walking forward with a proportionally "
      "controlled motor set to a desired crank velocity. Launch "
      "drake-visualizer before running this example.");

  FLAGS_simulator_accuracy = 1e-2;
  FLAGS_simulator_max_time_step = 1e-1;
  FLAGS_simulator_integration_scheme = "implicit_euler";

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::strandbeest::do_main();
}
