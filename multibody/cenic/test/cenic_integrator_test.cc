#include "drake/multibody/cenic/cenic_integrator.h"

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/test_utilities/spring_mass_system.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace multibody {
namespace cenic {
namespace {

using contact_solvers::icf::IcfSolverParameters;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::NamedStatistic;
using systems::Simulator;
using systems::SpringMassSystem;
using systems::controllers::PidController;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
constexpr char kDoublePendulumMjcf[] = R"""(
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
constexpr char kBallOnTableMjcf[] = R"""(
  <?xml version="1.0"?>
  <mujoco model="robot">
    <worldbody>
      <geom name="floor" type="box" pos="0 0 -0.1" size="50 50 0.1" />
      <body name="link1" pos="0 0 1.0">
        <joint name="joint1" type="free"/>
        <geom type="sphere" size="0.1" pos="0 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )""";

/* A base test case with utilities for setting up a CENIC simulation. */
class SimulationTestScenario : public testing::Test {
 protected:
  void SetUp() override { AddModels(); }

  /* Add models to the plant. */
  virtual void AddModels() = 0;

  /* Set defualt initial conditions. */
  virtual void SetInitialConditions() = 0;

  /* Creates the system diagram and sets up a simulation with CENIC. */
  void Build() {
    if (!plant_.is_finalized()) {
      plant_.Finalize();
    }

    diagram_ = builder_->Build();
    std::unique_ptr<Context<double>> context = diagram_->CreateDefaultContext();

    // Set initial conditions.
    plant_context_ = &plant_.GetMyMutableContextFromRoot(context.get());
    SetInitialConditions();

    // Set up a simulator with the CENIC integrator in error-control mode.
    simulator_ =
        std::make_unique<Simulator<double>>(*diagram_, std::move(context));
    integrator_ = &simulator_->reset_integrator<CenicIntegrator<double>>();
    integrator_->set_maximum_step_size(0.1);
    EXPECT_EQ(integrator_->get_fixed_step_mode(), false);
    integrator_->set_target_accuracy(1e-3);
  }

  // Only available prior to Build().
  std::unique_ptr<DiagramBuilder<double>> builder_{
      std::make_unique<DiagramBuilder<double>>()};

  // Only available after Build().
  std::unique_ptr<Diagram<double>> diagram_;
  Context<double>* plant_context_{nullptr};
  std::unique_ptr<Simulator<double>> simulator_;
  CenicIntegrator<double>* integrator_{nullptr};

  // Always available.
  MultibodyPlant<double>& plant_{
      AddMultibodyPlantSceneGraph(builder_.get(), 0.0).plant};
};

class DoublePendulum : public SimulationTestScenario {
 protected:
  void AddModels() override {
    Parser(builder_.get()).AddModelsFromString(kDoublePendulumMjcf, "xml");
  }

  void SetInitialConditions() override {
    plant_.SetPositions(plant_context_, Vector2d(M_PI / 4, M_PI / 2));
    plant_.SetVelocities(plant_context_, Vector2d(0.1, 0.2));
  }
};

class BallOnTable : public SimulationTestScenario {
 protected:
  void AddModels() override {
    Parser(builder_.get()).AddModelsFromString(kBallOnTableMjcf, "xml");
  }

  void SetInitialConditions() override {
    plant_.SetVelocities(plant_context_, VectorXd::Zero(6));
  }
};

/* Checks that the integrator complains when no initial step size is
configured. */
TEST_F(DoublePendulum, NoStep) {
  Build();
  integrator_ = &simulator_->reset_integrator<CenicIntegrator<double>>();
  DRAKE_EXPECT_THROWS_MESSAGE(simulator_->AdvanceTo(1e-2),
                              ".*Neither.*step size.*set.*");
}

/* Checks that the integrator reports expected statistics, and that they may be
reset. */
TEST_F(DoublePendulum, Stats) {
  Build();

  const std::set<std::string> expected_keys{
      "cenic_total_hessian_factorizations",
      "cenic_total_ls_iterations",
      "cenic_total_solver_iterations",
      "integrator_actual_initial_step_size_taken",
      "integrator_largest_step_size_taken",
      "integrator_num_derivative_evaluations",
      "integrator_num_step_shrinkages_from_error_control",
      "integrator_num_steps_taken",
      "integrator_smallest_adapted_step_size_taken"};

  auto validate_empty_stats = [&](const std::vector<NamedStatistic>& stats,
                                  std::string_view label) {
    EXPECT_EQ(ssize(stats), 9);
    std::set<std::string> found_keys;
    for (const auto& [key, value] : stats) {
      SCOPED_TRACE(fmt::format("{} stat for '{}'", label, key));
      found_keys.insert(key);
      if (std::holds_alternative<int64_t>(value)) {
        EXPECT_EQ(std::get<int64_t>(value), 0);
      } else {
        EXPECT_TRUE(std::isnan(std::get<double>(value)));
      }
    }
    EXPECT_EQ(found_keys, expected_keys);
  };

  validate_empty_stats(integrator_->GetStatisticsSummary(), "initial");

  simulator_->AdvanceTo(1.0);

  std::vector<NamedStatistic> later_stats = integrator_->GetStatisticsSummary();
  EXPECT_EQ(ssize(later_stats), 9);
  std::set<std::string> later_keys;
  for (const auto& [key, value] : later_stats) {
    later_keys.insert(key);
    if (key == "cenic_total_hessian_factorizations") {
      EXPECT_EQ(std::get<int64_t>(value), 569);
    } else if (key == "cenic_total_ls_iterations") {
      // The system simulated here doesn't require using line search. See
      // icf_solver_test for test case that exercises this statistic.
      EXPECT_EQ(std::get<int64_t>(value), 0);
    } else if (key == "cenic_total_solver_iterations") {
      EXPECT_EQ(std::get<int64_t>(value), 569);
    } else if (key == "integrator_actual_initial_step_size_taken") {
      EXPECT_NEAR(std::get<double>(value), 0.01, 1e-9);
    } else if (key == "integrator_largest_step_size_taken") {
      EXPECT_NEAR(std::get<double>(value), 0.012669, 1e-6);
    } else if (key == "integrator_num_derivative_evaluations") {
      // CENIC doesn't ever call CalcTimeDerivatives(), which is what is being
      // counted here.
      EXPECT_EQ(std::get<int64_t>(value), 0);
    } else if (key == "integrator_num_step_shrinkages_from_error_control") {
      EXPECT_EQ(std::get<int64_t>(value), 23);
    } else if (key == "integrator_num_steps_taken") {
      EXPECT_EQ(std::get<int64_t>(value), 167);
    } else if (key == "integrator_smallest_adapted_step_size_taken") {
      EXPECT_NEAR(std::get<double>(value), 0.003819, 1e-6);
    } else {
      GTEST_FAIL() << "Unrecognized statistic name: '" << key << "'.";
    }
  }
  EXPECT_EQ(later_keys, expected_keys);

  integrator_->ResetStatistics();

  validate_empty_stats(integrator_->GetStatisticsSummary(), "reset");
}

/* Checks that solver parameter setting is effective. Since the methods tested
here only forward to the ICF solver, only one field is checked. Detailed
testing of parameter effects can be found in icf_solver_test. */
TEST_F(DoublePendulum, Parameters) {
  Build();
  EXPECT_NO_THROW(simulator_->AdvanceTo(1e-2));
  IcfSolverParameters params = integrator_->get_solver_parameters();
  params.max_iterations = 1;  // Force solver failure.
  integrator_->SetSolverParameters(params);
  DRAKE_EXPECT_THROWS_MESSAGE(simulator_->AdvanceTo(2e-2),
                              ".*optimization failed.*");
}

/* Checks that a double pendulum simulation in fixed-step mode matches the
discrete solver. Simulation time and stepping are carefully chosen to
facilitate matching of results. */
TEST_F(DoublePendulum, FixedStep) {
  const double time_step = 1e-2;
  const double simulation_time = 0.5;
  Build();

  // Simulate with CENIC in fixed-step mode.
  const VectorXd x_initial = plant_.GetPositionsAndVelocities(*plant_context_);
  integrator_->set_maximum_step_size(time_step);
  integrator_->set_fixed_step_mode(true);
  simulator_->AdvanceTo(simulation_time);
  EXPECT_EQ(integrator_->get_num_steps_taken(),
            static_cast<int>(simulation_time / time_step));
  const VectorXd x_final = plant_.GetPositionsAndVelocities(*plant_context_);

  // Simulate a discrete-time plant with the same time step.
  MultibodyPlant<double> reference_plant(time_step);
  Parser(&reference_plant).AddModelsFromString(kDoublePendulumMjcf, "xml");
  reference_plant.Finalize();
  auto reference_context = reference_plant.CreateDefaultContext();
  reference_plant.SetPositionsAndVelocities(reference_context.get(), x_initial);
  Simulator<double> reference_simulator(reference_plant,
                                        std::move(reference_context));
  reference_simulator.Initialize();
  reference_simulator.AdvanceTo(simulation_time);

  const VectorXd x_final_reference = reference_plant.GetPositionsAndVelocities(
      reference_simulator.get_context());

  EXPECT_TRUE(CompareMatrices(x_final, x_final_reference, 100 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Verifies that energy conservation and error metrics for an undamped double
pendulum are improved with tightened accuracy. */
TEST_F(DoublePendulum, AccuracyTrends) {
  const double simulation_time = 2.0;

  // Remove joint damping for this test.
  for (multibody::JointIndex i : plant_.GetJointIndices()) {
    multibody::Joint<double>& joint = plant_.get_mutable_joint(i);
    joint.set_default_damping_vector(VectorXd::Zero(1));
  }
  Build();

  // Record the initial system energy (kinetic + potential).
  const VectorXd x_initial = plant_.GetPositionsAndVelocities(*plant_context_);
  const double initial_energy = plant_.CalcPotentialEnergy(*plant_context_) +
                                plant_.CalcKineticEnergy(*plant_context_);

  // Simulate with error control at a very loose accuracy.
  integrator_->set_target_accuracy(1e-1);
  simulator_->AdvanceTo(simulation_time);

  // Calculate a norm of the error estimate for use in trend checking.
  auto error_norm = [](const systems::ContinuousState<double>* estimate) {
    return estimate->CopyToVector().norm();
  };

  const int num_steps_loose = integrator_->get_num_steps_taken();
  const double final_energy_loose =
      plant_.CalcPotentialEnergy(*plant_context_) +
      plant_.CalcKineticEnergy(*plant_context_);
  const double error_loose = error_norm(integrator_->get_error_estimate());

  // Simulate again with a much tighter accuracy.
  simulator_->get_mutable_context().SetTime(0.0);
  plant_.SetPositionsAndVelocities(plant_context_, x_initial);
  integrator_->set_target_accuracy(1e-5);
  simulator_->Initialize();
  simulator_->AdvanceTo(simulation_time);

  const int num_steps_tight = integrator_->get_num_steps_taken();
  const double final_energy_tight =
      plant_.CalcPotentialEnergy(*plant_context_) +
      plant_.CalcKineticEnergy(*plant_context_);
  const double error_tight = error_norm(integrator_->get_error_estimate());

  const double energy_error_tight =
      std::abs(final_energy_tight - initial_energy) / initial_energy;
  const double energy_error_loose =
      std::abs(final_energy_loose - initial_energy) / initial_energy;
  EXPECT_LT(energy_error_tight, energy_error_loose);
  EXPECT_GT(num_steps_tight, num_steps_loose);
  EXPECT_LT(error_tight, error_loose);
}

/* Simulates the double pendulum with PID actuation. */
TEST_F(DoublePendulum, ExternalActuation) {
  // Add some actuators to the plant.
  for (multibody::JointIndex i : plant_.GetJointIndices()) {
    multibody::Joint<double>& joint = plant_.get_mutable_joint(i);
    plant_.AddJointActuator(joint.name() + "_actuator", joint);
  }
  plant_.Finalize();
  EXPECT_EQ(plant_.num_actuators(), 2);

  // Set up a PID controller.
  const Vector2d Kp(1.24, 0.79);
  const Vector2d Kd(0.15, 0.03);
  const Vector2d Ki(0.5, 0.4);
  const Vector4d x_nom(M_PI_2, -M_PI_2, 0.0, 0.0);

  auto target_sender = builder_->AddSystem<ConstantVectorSource<double>>(x_nom);
  auto pid_controller = builder_->AddSystem<PidController<double>>(Kp, Ki, Kd);

  builder_->Connect(target_sender->get_output_port(),
                    pid_controller->get_input_port_desired_state());
  builder_->Connect(plant_.get_state_output_port(),
                    pid_controller->get_input_port_estimated_state());
  builder_->Connect(pid_controller->get_output_port(),
                    plant_.get_actuation_input_port());

  Build();

  // Simulate for a few seconds.
  simulator_->AdvanceTo(20.0);

  // We should get close-ish to the target state.
  const VectorXd x_final = plant_.GetPositionsAndVelocities(*plant_context_);
  EXPECT_TRUE(
      CompareMatrices(x_final, x_nom, 1e-3, MatrixCompareType::absolute));

  // The PID controller's external state (due to integral terms) should have
  // advanced. In particular, we should be close to the steady-state, where
  // integral terms -Ki * z balance gravity.
  const VectorXd z = simulator_->get_context()
                         .get_continuous_state()
                         .get_misc_continuous_state()
                         .CopyToVector();
  const VectorXd tau_g = plant_.CalcGravityGeneralizedForces(*plant_context_);
  const VectorXd tau_integral = -(Ki.asDiagonal() * z);
  EXPECT_TRUE(
      CompareMatrices(tau_integral, tau_g, 1e-3, MatrixCompareType::absolute));
}

/* Checks integration of an external (non-plant) system with second-order (q, v)
continuous state.*/
// TODO(#23921): Revisit this test when implementing an error estimate that
// accounts for external systems.
TEST_F(DoublePendulum, PositionVelocityExternalSystem) {
  auto spring_mass_system = builder_->AddSystem<SpringMassSystem>(100.0, 1.0);
  Build();

  const double q0 = 0.2;
  const double v0 = 0.1;
  Context<double>& spring_mass_context =
      spring_mass_system->GetMyMutableContextFromRoot(
          &simulator_->get_mutable_context());
  spring_mass_system->set_position(&spring_mass_context, q0);
  spring_mass_system->set_velocity(&spring_mass_context, v0);

  // For now, error control only acts on the plant's configurations and does not
  // consider the external system state. Therefore, we run this test in
  // fixed-step mode. See issue #23921 for further details.
  integrator_->set_maximum_step_size(0.001);
  integrator_->set_fixed_step_mode(true);
  simulator_->AdvanceTo(0.1);

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
  for (multibody::JointIndex i : plant_.GetJointIndices()) {
    multibody::Joint<double>& joint = plant_.get_mutable_joint(i);
    joint.set_position_limits(VectorXd::Constant(1, 0.2 * (i + 1)),
                              VectorXd::Constant(1, 3.1));
  }
  Build();

  simulator_->AdvanceTo(1.0);

  // The final configuration should be resting on the lower joint limits.
  const VectorXd q = plant_.GetPositions(*plant_context_);
  EXPECT_NEAR(q(0), 0.2, 1e-5);
  EXPECT_NEAR(q(1), 0.4, 1e-5);
}

/* Checks that effort limits are enforced by the integrator. */
TEST_F(DoublePendulum, EffortLimits) {
  // Add some actuators with tight effort limits to the plant.
  for (multibody::JointIndex i : plant_.GetJointIndices()) {
    multibody::Joint<double>& joint = plant_.get_mutable_joint(i);
    plant_.AddJointActuator(joint.name() + "_actuator", joint, 0.1 * (i + 1));

    // Disable joint damping so it's easier to derive the applied torques
    // analytically.
    joint.set_default_damping_vector(VectorXd::Zero(1));
  }
  Build();

  // Apply large torques that would exceed the effort limits.
  plant_.get_actuation_input_port().FixValue(plant_context_,
                                             VectorXd::Constant(2, 10.0));

  // Compute some dynamics terms.
  const VectorXd q0 = plant_.GetPositions(*plant_context_);
  const VectorXd v0 = plant_.GetVelocities(*plant_context_);
  MatrixXd M(2, 2);
  VectorXd k(2);
  MultibodyForces<double> f_ext(plant_);
  plant_.CalcMassMatrix(*plant_context_, &M);
  plant_.CalcForceElementsContribution(*plant_context_, &f_ext);
  k = plant_.CalcInverseDynamics(*plant_context_, VectorXd::Zero(2), f_ext);

  // Simulate for a single step.
  const double h = 0.01;
  integrator_->set_fixed_step_mode(true);
  integrator_->set_maximum_step_size(h);
  simulator_->AdvanceTo(h);
  EXPECT_EQ(integrator_->get_num_steps_taken(), 1);

  // Requested actuation should exceed effort limits.
  const VectorXd u_requested =
      plant_.get_actuation_input_port().Eval(*plant_context_);
  EXPECT_TRUE(u_requested[0] > 0.1);
  EXPECT_TRUE(u_requested[1] > 0.2);

  // Applied actuation should be clamped to effort limits.
  // TODO(#24074): report this in plant.get_net_actuation_output_port().
  const VectorXd v = plant_.GetVelocities(*plant_context_);
  const VectorXd u_applied = M * (v - v0) / h + k;

  EXPECT_NEAR(u_applied[0], 0.1, 10 * kEpsilon);
  EXPECT_NEAR(u_applied[1], 0.2, 10 * kEpsilon);
}

/* Checks that coupler constraints are handled correctly by the integrator. */
TEST_F(DoublePendulum, CoupledJoints) {
  const double gear_ratio = 0.5;
  plant_.AddCouplerConstraint(plant_.GetJointByName("joint1"),
                              plant_.GetJointByName("joint2"), gear_ratio);
  Build();
  const VectorXd q0 = plant_.GetPositions(*plant_context_);

  // Initial conditions should satisfy the coupler constraint.
  EXPECT_NEAR(q0(0), gear_ratio * q0(1), kEpsilon);

  simulator_->AdvanceTo(1.0);

  // Verify that the constraint is satisfied at the end of the simulation.  The
  // constraint is essentially enforced by a very stiff spring, so we shouldn't
  // expect exact satisfaction.
  const VectorXd q = plant_.GetPositions(*plant_context_);
  EXPECT_NEAR(q(0), gear_ratio * q(1), 1e-5);
}

/* Checks that a ball rolling along the ground produces the expected result. */
TEST_F(BallOnTable, RollingContact) {
  Build();

  // Place the ball on the ground, with initial linear and angular velocity
  // for rolling without slipping.
  const double radius = 0.1;
  const double roll_speed = 0.8;
  VectorXd q0 = plant_.GetPositions(*plant_context_);
  q0[4] = 0.0;     // x position
  q0[6] = radius;  // z position
  plant_.SetPositions(plant_context_, q0);
  VectorXd v0 = VectorXd::Zero(6);
  v0[1] = roll_speed / radius;
  v0[3] = roll_speed;
  plant_.SetVelocities(plant_context_, v0);

  // Compare a simulation with the analytical solution.
  const double simulation_time = 1.0;
  const double x_expected = roll_speed * simulation_time;
  const double z_expected = radius;

  simulator_->AdvanceTo(simulation_time);

  const VectorXd q = plant_.GetPositions(*plant_context_);
  const double x_actual = q[4];
  const double z_actual = q[6];

  EXPECT_NEAR(x_actual, x_expected, 1e-3);
  EXPECT_NEAR(z_actual, z_expected, 1e-3);
}

/* Checks that quaternions always remain normalized. */
TEST_F(BallOnTable, QuaternionNormalization) {
  Build();

  // Set initial velocities that will yeet the ball into the distance while
  // spinning wildly.
  VectorXd v0(6);
  v0 << 100.0, -218.0, -82.0, 10.0, 20.0, 500.0;
  plant_.SetVelocities(plant_context_, v0);

  // Run the simulation briefly with fixed (large) steps.
  integrator_->set_maximum_step_size(0.1);
  integrator_->set_fixed_step_mode(true);
  simulator_->AdvanceTo(1.0);

  // Quaternions should remain normalized.
  const VectorXd q = plant_.GetPositions(*plant_context_);
  const Vector4d quat = q.segment<4>(0);
  const double quat_norm = quat.norm();
  EXPECT_NEAR(quat_norm, 1.0, 10 * kEpsilon);
}

/* Verifies that we can simulate non-actuator external forces. */
TEST_F(BallOnTable, ExternalForces) {
  const double simulation_time = 0.3;
  Build();
  const double initial_height = plant_.GetPositions(*plant_context_)[6];

  // Apply a constant upward force.
  VectorXd f_ext = VectorXd::Zero(plant_.num_velocities());
  f_ext[5] = 50.0;  // +z direction
  plant_.get_applied_generalized_force_input_port().FixValue(plant_context_,
                                                             f_ext);
  integrator_->set_target_accuracy(1e-5);
  simulator_->AdvanceTo(simulation_time);

  // Compare with the analytical solution.
  const double m = plant_.CalcTotalMass(*plant_context_);
  const double g = 9.81;
  const double net_acceleration = -g + 50.0 / m;
  const double expected_height = initial_height + 0.5 * net_acceleration *
                                                      simulation_time *
                                                      simulation_time;
  const double final_height = plant_.GetPositions(*plant_context_)[6];
  EXPECT_NEAR(final_height, expected_height, 1e-3);
}

}  // namespace
}  // namespace cenic
}  // namespace multibody
}  // namespace drake
