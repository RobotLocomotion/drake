#include "drake/multibody/cenic/cenic_integrator.h"

#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/analysis/test_utilities/spring_mass_system.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace multibody {
namespace cenic {
namespace {

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

using common::MaybePauseForUser;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using geometry::Meshcat;
using geometry::SceneGraph;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;
using systems::SimulatorConfig;
using systems::SpringMassSystem;
using systems::controllers::PidController;
using visualization::ApplyVisualizationConfig;
using visualization::VisualizationConfig;

class DoublePendulum : public testing::Test {
 public:
  DoublePendulum() {
    meshcat_ = std::make_shared<Meshcat>();
    builder_ = std::make_unique<DiagramBuilder<double>>();
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(builder_.get(), 0.0);
    Parser(plant_, scene_graph_).AddModelsFromString(xml_, "xml");
  }

  /* Creates the system diagram and sets initial conditions. */
  Context<double>& Finalize() {
    plant_->Finalize();

    VisualizationConfig vis_config;
    // Don't create visualizer events that interfere with the integrator's step
    // size selection.
    vis_config.publish_period = 1e9;
    ApplyVisualizationConfig(vis_config, builder_.get(), nullptr, plant_,
                             scene_graph_, meshcat_);

    diagram_ = builder_->Build();
    context_ = diagram_->CreateDefaultContext();
    Context<double>& plant_context =
        plant_->GetMyMutableContextFromRoot(context_.get());
    plant_->SetPositions(&plant_context, Eigen::Vector2d(M_PI / 4, M_PI / 2));
    plant_->SetVelocities(&plant_context, Eigen::Vector2d(0.1, 0.2));
    return plant_context;
  }

 protected:
  std::string xml_ = R"""(
      <?xml version="1.0"?>
      <mujoco model="double_pendulum">
      <worldbody>
        <body>
        <joint type="hinge" name="joint1" axis="0 1 0" pos="0 0 0.1" damping="0.001"/>
        <geom type="capsule" size="0.01 0.1"/>
        <body>
          <joint type="hinge" name="joint2" axis="0 1 0" pos="0 0 -0.1" damping="0.001"/>
          <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
        </body>
        </body>
      </worldbody>
      </mujoco>
      )""";
  std::unique_ptr<DiagramBuilder<double>> builder_;
  MultibodyPlant<double>* plant_;
  SceneGraph<double>* scene_graph_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  std::shared_ptr<Meshcat> meshcat_;
};

/* Checks that a double pendulum simulation in fixed-step mode matches the
discrete solver. */
TEST_F(DoublePendulum, FixedStep) {
  const double time_step = 1e-2;
  const double simulation_time = 0.5;
  Context<double>& plant_context = Finalize();
  const VectorXd x0 = plant_->GetPositionsAndVelocities(plant_context);

  // Simulate with the integrator in fixed-step mode
  Simulator<double> simulator(*diagram_, std::move(context_));
  CenicIntegrator<double>& integrator =
      simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(time_step);
  integrator.set_fixed_step_mode(true);
  simulator.set_target_realtime_rate(0.0);
  simulator.AdvanceTo(simulation_time);
  EXPECT_EQ(integrator.get_num_steps_taken(),
            static_cast<int>(simulation_time / time_step));

  const VectorXd xT = plant_->GetPositionsAndVelocities(plant_context);

  // Simulate in discrete mode with the same time step.
  MultibodyPlant<double> reference_plant(time_step);
  Parser(&reference_plant).AddModelsFromString(xml_, "xml");
  reference_plant.Finalize();
  auto reference_context = reference_plant.CreateDefaultContext();
  reference_plant.SetPositionsAndVelocities(reference_context.get(), x0);
  Simulator<double> reference_simulator(reference_plant,
                                        std::move(reference_context));
  reference_simulator.Initialize();
  reference_simulator.AdvanceTo(simulation_time);

  const VectorXd xT_reference = reference_plant.GetPositionsAndVelocities(
      reference_simulator.get_context());

  EXPECT_TRUE(CompareMatrices(xT, xT_reference, 100 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Verifies that energy conservation for an undamped double pendulum is improved
with tightened accuracy. */
TEST_F(DoublePendulum, EnergyConservation) {
  const double simulation_time = 2.0;

  // Remove joint damping for this test.
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    joint.set_default_damping_vector(Eigen::VectorXd::Zero(1));
  }
  Context<double>& plant_context = Finalize();

  // Record the initial system energy (kinetic + potential).
  const VectorXd x0 = plant_->GetPositionsAndVelocities(plant_context);
  const double e0 = plant_->CalcPotentialEnergy(plant_context) +
                    plant_->CalcKineticEnergy(plant_context);

  // Simulate with error control at loose accuracy.
  Simulator<double> simulator(*diagram_, std::move(context_));
  CenicIntegrator<double>& integrator =
      simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(false);
  integrator.set_target_accuracy(1e-1);  // Very loose accuracy.
  simulator.set_target_realtime_rate(0.0);
  simulator.AdvanceTo(simulation_time);

  const int num_steps_loose = integrator.get_num_steps_taken();
  const double eT_loose = plant_->CalcPotentialEnergy(plant_context) +
                          plant_->CalcKineticEnergy(plant_context);

  // Simulate again with a tighter accuracy.
  simulator.get_mutable_context().SetTime(0.0);
  plant_->SetPositionsAndVelocities(&plant_context, x0);
  integrator.set_target_accuracy(1e-5);  // Much tighter accuracy.
  simulator.Initialize();
  simulator.AdvanceTo(simulation_time);

  const int num_steps_tight = integrator.get_num_steps_taken();
  const double eT_tight = plant_->CalcPotentialEnergy(plant_context) +
                          plant_->CalcKineticEnergy(plant_context);

  const double energy_error_tight = std::abs(eT_tight - e0) / e0;
  const double energy_error_loose = std::abs(eT_loose - e0) / e0;
  EXPECT_LT(energy_error_tight, energy_error_loose);
  EXPECT_GT(num_steps_tight, num_steps_loose);
}

/* Simulates the double pendulum with PID actuation. */
TEST_F(DoublePendulum, ExternalActuation) {
  const double simulation_time = 10.0;

  // Add joint actuators to the plant.
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    plant_->AddJointActuator(joint.name() + "_actuator", joint);
  }
  plant_->Finalize();
  EXPECT_EQ(plant_->num_actuators(), 2);

  // Set up a PID controller
  const Eigen::Vector2d q_nom(-M_PI / 4, M_PI / 2);
  const Eigen::Vector2d Kp(1.24, 0.79);
  const Eigen::Vector2d Kd(0.15, 0.03);
  const Eigen::Vector2d Ki(0.5, 0.4);
  const Eigen::Vector4d x_nom(M_PI_2, -M_PI_2, 0.0, 0.0);

  auto target_sender = builder_->AddSystem<ConstantVectorSource<double>>(x_nom);
  auto pid_controller = builder_->AddSystem<PidController<double>>(Kp, Ki, Kd);

  builder_->Connect(target_sender->get_output_port(),
                    pid_controller->get_input_port_desired_state());
  builder_->Connect(plant_->get_state_output_port(),
                    pid_controller->get_input_port_estimated_state());
  builder_->Connect(pid_controller->get_output_port(),
                    plant_->get_actuation_input_port());

  VisualizationConfig vis_config;
  vis_config.publish_period = 1e9;
  ApplyVisualizationConfig(vis_config, builder_.get(), nullptr, plant_,
                           scene_graph_, meshcat_);

  // Finalize the diagram and set initial conditions.
  diagram_ = builder_->Build();
  context_ = diagram_->CreateDefaultContext();
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(context_.get());
  plant_->SetPositions(&plant_context, Eigen::Vector2d(M_PI_4, M_PI_2));
  plant_->SetVelocities(&plant_context, Eigen::Vector2d(0.1, 0.2));

  // Run the simulation with error-control.
  Simulator<double> simulator(*diagram_, std::move(context_));
  CenicIntegrator<double>& integrator =
      simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(false);
  integrator.set_target_accuracy(1e-3);
  simulator.set_target_realtime_rate(0.0);
  simulator.set_publish_every_time_step(true);

  meshcat_->StartRecording();
  simulator.AdvanceTo(simulation_time);
  meshcat_->StopRecording();
  meshcat_->PublishRecording();
  MaybePauseForUser();

  // We should get close-ish to the target state.
  const VectorXd xT = plant_->GetPositionsAndVelocities(plant_context);
  EXPECT_TRUE(CompareMatrices(xT, x_nom, 1e-2, MatrixCompareType::absolute));

  // The PID controller's external state (due to integral terms) should have
  // advanced.
  const VectorXd z = simulator.get_context()
                         .get_continuous_state()
                         .get_misc_continuous_state()
                         .CopyToVector();
  EXPECT_TRUE(z(0) != 0.0 && z(1) != 0.0);
}

/* Checks integration of an external (non-plant) system with second-order (q, v)
continuous state.*/
TEST_F(DoublePendulum, PositionVelocityExternalSystem) {
  auto spring_mass_system = builder_->AddSystem<SpringMassSystem>(100.0, 1.0);
  Finalize();

  const double q0 = 0.2;
  const double v0 = 0.1;
  Context<double>& spring_mass_context =
      spring_mass_system->GetMyMutableContextFromRoot(context_.get());
  spring_mass_system->set_position(&spring_mass_context, q0);
  spring_mass_system->set_velocity(&spring_mass_context, v0);

  Simulator<double> simulator(*diagram_, std::move(context_));
  SimulatorConfig config;
  config.target_realtime_rate = 0.0;
  config.publish_every_time_step = true;
  ApplySimulatorConfig(config, &simulator);

  CenicIntegrator<double>& integrator =
      simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.001);
  integrator.set_fixed_step_mode(true);

  simulator.Initialize();
  simulator.AdvanceTo(0.1);

  const double qf = spring_mass_system->get_position(spring_mass_context);
  const double vf = spring_mass_system->get_velocity(spring_mass_context);
  double qf_expected;
  double vf_expected;
  spring_mass_system->GetClosedFormSolution(q0, v0, 0.1, &qf_expected,
                                            &vf_expected);

  EXPECT_NEAR(qf, qf_expected, 1e-2);
  EXPECT_NEAR(vf, vf_expected, 1e-2);
}

/* Checks that the integrator can enforce joint limits. */
TEST_F(DoublePendulum, JointLimits) {
  // Add joint limits to the plant.
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    joint.set_position_limits(VectorXd::Constant(1, 0.2),
                              VectorXd::Constant(1, 3.1));
  }
  Context<double>& plant_context = Finalize();

  // Run the simulation with error-control.
  Simulator<double> simulator(*diagram_, std::move(context_));
  CenicIntegrator<double>& integrator =
      simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(false);
  integrator.set_target_accuracy(1e-3);
  simulator.set_target_realtime_rate(0.0);
  simulator.set_publish_every_time_step(true);

  meshcat_->StartRecording();
  simulator.AdvanceTo(1.0);
  meshcat_->StopRecording();
  meshcat_->PublishRecording();
  MaybePauseForUser();

  // The final configuration should be resting on the lower joint limits.
  const VectorXd q = plant_->GetPositions(plant_context);
  EXPECT_NEAR(q(0), 0.2, 1e-5);
  EXPECT_NEAR(q(1), 0.2, 1e-5);
}

/* Checks that effort limits are enforced by the integrator. */
TEST_F(DoublePendulum, EffortLimits) {
  // Add joint actuators with effort limits to the plant.
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    plant_->AddJointActuator(joint.name() + "_actuator", joint, 0.1);
    joint.set_default_damping_vector(Eigen::VectorXd::Zero(1));
  }
  Context<double>& plant_context = Finalize();
  const VectorXd q0 = plant_->GetPositions(plant_context);
  const VectorXd v0 = plant_->GetVelocities(plant_context);

  // Apply a large constant torque to both joints.
  plant_->get_actuation_input_port().FixValue(&plant_context,
                                              VectorXd::Constant(2, 10.0));

  // Set up the integrator.
  CenicIntegrator<double> integrator(*diagram_, context_.get());
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(true);
  integrator.Initialize();

  // Compute some dynamics terms.
  MatrixXd M(2, 2);
  VectorXd k(2);
  MultibodyForces<double> f_ext(*plant_);
  plant_->CalcMassMatrix(plant_context, &M);
  plant_->CalcForceElementsContribution(plant_context, &f_ext);
  k = plant_->CalcInverseDynamics(plant_context, VectorXd::Zero(2), f_ext);

  // Simulate for a single step.
  const double h = 0.01;
  EXPECT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(h));

  // Requested actuation should exceed effort limits.
  const VectorXd u_req = plant_->get_actuation_input_port().Eval(plant_context);
  EXPECT_TRUE(u_req[0] > 0.1);
  EXPECT_TRUE(u_req[1] > 0.1);

  // Applied actuation should be clamped to effort limits.
  // TODO(vincekurtz): report u_app in plant.get_net_actuation_output_port().
  const VectorXd v = plant_->GetVelocities(plant_context);
  const VectorXd u_app = M * (v - v0) / h + k;

  EXPECT_TRUE(u_app[0] <= 0.1 + 100 * kEpsilon);
  EXPECT_TRUE(u_app[1] <= 0.1 + 100 * kEpsilon);
}

/* Checks that coupler constraints are handled correctly by the integrator. */
TEST_F(DoublePendulum, CoupledJoints) {
  const double gear_ratio = 0.5;
  plant_->AddCouplerConstraint(plant_->GetJointByName("joint1"),
                               plant_->GetJointByName("joint2"), gear_ratio);
  Context<double>& plant_context = Finalize();
  const VectorXd q0 = plant_->GetPositions(plant_context);

  // Initial conditions should satisfy the coupler constraint.
  EXPECT_NEAR(q0(0), gear_ratio * q0(1), 1e-5);

  // Run the simulation briefly.
  Simulator<double> simulator(*diagram_, std::move(context_));
  CenicIntegrator<double>& integrator =
      simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(false);
  integrator.set_target_accuracy(1e-3);
  simulator.set_target_realtime_rate(0.0);
  simulator.set_publish_every_time_step(true);
  meshcat_->StartRecording();
  simulator.AdvanceTo(1.0);
  meshcat_->StopRecording();
  meshcat_->PublishRecording();
  MaybePauseForUser();

  // Verify that the constraint is satisfied at the end of the simulation.
  const VectorXd q = plant_->GetPositions(plant_context);
  EXPECT_NEAR(q(0), gear_ratio * q(1), 1e-5);
}

class BallOnTable : public testing::Test {
 public:
  BallOnTable() {
    meshcat_ = std::make_shared<Meshcat>();
    builder_ = std::make_unique<DiagramBuilder<double>>();
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(builder_.get(), 0.0);
    Parser(plant_, scene_graph_).AddModelsFromString(xml_, "xml");
  }

  /* Creates the system diagram and sets initial conditions. */
  Context<double>& Finalize() {
    plant_->Finalize();

    VisualizationConfig vis_config;
    // Don't create visualizer events that interfere with the integrator's step
    // size selection.
    vis_config.publish_period = 1e9;
    ApplyVisualizationConfig(vis_config, builder_.get(), nullptr, plant_,
                             scene_graph_, meshcat_);

    diagram_ = builder_->Build();
    context_ = diagram_->CreateDefaultContext();
    Context<double>& plant_context =
        plant_->GetMyMutableContextFromRoot(context_.get());
    return plant_context;
  }

 protected:
  std::string xml_ = R"""(
    <?xml version="1.0"?>
    <mujoco model="robot">
      <worldbody>
        <geom name="floor" type="plane" pos="0 0 0" size="5 5 0.1" />
        <body name="link1" pos="0 0 1.0">
          <joint name="joint1" type="free"/>
          <geom type="sphere" size="0.1" pos="0 0 0"/>
        </body>
      </worldbody>
    </mujoco>
    )""";
  std::unique_ptr<DiagramBuilder<double>> builder_;
  MultibodyPlant<double>* plant_;
  SceneGraph<double>* scene_graph_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  std::shared_ptr<Meshcat> meshcat_;
};

/* Checks that a ball will fall and roll along the table. */
TEST_F(BallOnTable, FreeFall) {
  // Set a non-zero initial velocity.
  Context<double>& plant_context = Finalize();
  VectorXd v0(6);
  v0 << 0.0, 0.0, 0.0, 0.8, 0.0, 0.0;
  plant_->SetVelocities(&plant_context, v0);

  // Run the simulation briefly.
  Simulator<double> simulator(*diagram_, std::move(context_));
  CenicIntegrator<double>& integrator =
      simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(false);
  integrator.set_target_accuracy(1e-3);
  simulator.set_target_realtime_rate(0.0);
  simulator.set_publish_every_time_step(true);
  meshcat_->StartRecording();
  simulator.AdvanceTo(1.0);
  meshcat_->StopRecording();
  meshcat_->PublishRecording();
  MaybePauseForUser();

  // The ball should be on the table, and have moved in the +x direction.
  const VectorXd q = plant_->GetPositions(plant_context);
  EXPECT_TRUE(0.09 <= q[6] && q[6] <= 0.1);  // height
  EXPECT_GT(q[4], 0.5);                      // x position

  // Hitting the ground should have triggered at least one step size reduction.
  EXPECT_GT(integrator.get_num_step_shrinkages_from_error_control(), 0);
  EXPECT_EQ(integrator.get_num_step_shrinkages_from_substep_failures(), 0);
}

/* Checks that quaternions always remain normalized. */
TEST_F(BallOnTable, QuaternionNormalization) {
  // Set initial velocities that will yeet the ball into the distance while
  // spinning wildly.
  Context<double>& plant_context = Finalize();
  VectorXd v0(6);
  v0 << 100.0, -218.0, -82.0, 10.0, 20.0, 500.0;
  plant_->SetVelocities(&plant_context, v0);

  // Run the simulation briefly with fixed (large) steps.
  Simulator<double> simulator(*diagram_, std::move(context_));
  CenicIntegrator<double>& integrator =
      simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(true);
  simulator.set_target_realtime_rate(0.0);
  simulator.set_publish_every_time_step(true);
  meshcat_->StartRecording();
  simulator.AdvanceTo(1.0);
  meshcat_->StopRecording();
  meshcat_->PublishRecording();
  MaybePauseForUser();

  // Quaternions should remain normalized.
  const VectorXd q = plant_->GetPositions(plant_context);
  const Eigen::Vector4d quat = q.segment<4>(0);
  const double quat_norm = quat.norm();
  EXPECT_NEAR(quat_norm, 1.0, 10 * kEpsilon);
}

/* Verifies that we can simulate non-actuator external forces. */
TEST_F(BallOnTable, ExternalForces) {
  Context<double>& plant_context = Finalize();
  const double initial_height = plant_->GetPositions(plant_context)[6];

  // Apply a constant upward force.
  VectorXd f_ext = VectorXd::Zero(plant_->num_velocities());
  f_ext[5] = 50.0;  // +z direction
  plant_->get_applied_generalized_force_input_port().FixValue(&plant_context,
                                                              f_ext);

  // Run the simulation briefly with fixed (large) steps.
  Simulator<double> simulator(*diagram_, std::move(context_));
  CenicIntegrator<double>& integrator =
      simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(false);
  integrator.set_target_accuracy(1e-3);
  simulator.set_target_realtime_rate(0.0);
  simulator.set_publish_every_time_step(true);
  meshcat_->StartRecording();
  simulator.AdvanceTo(1.0);
  meshcat_->StopRecording();
  meshcat_->PublishRecording();
  MaybePauseForUser();

  // The ball should have moved upward, overcoming gravity.
  const double final_height = plant_->GetPositions(plant_context)[6];
  EXPECT_GT(final_height, initial_height);
}

}  // namespace
}  // namespace cenic
}  // namespace multibody
}  // namespace drake
