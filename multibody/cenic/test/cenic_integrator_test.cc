#include "drake/multibody/cenic/cenic_integrator.h"

#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
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
using math::RigidTransformd;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::InputPort;
using systems::NamedStatistic;
using systems::OutputPort;
using systems::Simulator;
using systems::SpringMassSystem;
using systems::System;
using systems::controllers::PidController;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
constexpr char kDoublePendulumMjcf[] = R"""(
  <?xml version="1.0"?>
  <mujoco model="double_pendulum">
    <worldbody>
      <body name="link1">
        <joint type="hinge" name="joint1" axis="0 1 0" pos="0 0 0.1" damping="0.001"/>
        <geom type="capsule" size="0.01 0.1"/>
        <body name="link2">
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

// TestParam uses tuple for compatibility with ::testing::Combine.
// get<0>() is nesting depth for the plant and scene graph.
// get<1>() is nesting depth for external systems.
// get<2>() is true if we have a scene graph.
using TestParam = std::tuple<int, int, bool>;

/* A base test case with utilities for setting up a CENIC simulation. */
class SimulationTestScenario : public testing::TestWithParam<TestParam> {
 protected:
  void SetUp() override {
    const TestParam params = GetParam();
    plant_depth_ = std::get<0>(params);
    external_depth_ = std::get<1>(params);
    have_scene_graph_ = std::get<2>(params);
    if (have_scene_graph_) {
      auto result = AddMultibodyPlantSceneGraph(builder_.get(), 0.0);
      plant_ = &result.plant;
      scene_graph_ = &result.scene_graph;
    } else {
      plant_ = builder_->AddNamedSystem<MultibodyPlant<double>>("plant", 0.0);
    }
    plant_nest_ = plant_;
    AddModels();
  }

  /* Adds models to the plant. */
  virtual void AddModels() = 0;

  /* Sets default initial conditions. */
  virtual void SetInitialConditions() = 0;

  /* Prepares a `system` to be nested within the diagram eventually produced by
  `builder`, by exporting all of its ports with their original unqualified
  names. */
  void ReExportAllPorts(DiagramBuilder<double>* builder,
                        const System<double>& system) {
    for (int m = 0; m < system.num_input_ports(); ++m) {
      const InputPort<double>& input = system.get_input_port(m);
      if (builder->IsConnectedOrExported(input)) {
        continue;
      }
      builder->ExportInput(input, input.get_name());
    }
    for (int m = 0; m < system.num_output_ports(); ++m) {
      const OutputPort<double>& output = system.get_output_port(m);
      builder->ExportOutput(output, output.get_name());
    }
  }

  /* Wraps `my_system` in extra nested diagrams, according to the "external
  depth" test parameter, and adds the entire thing to the top-level builder.
  Returns a tuple of [0] the derived-class pointer to the wrapped system (for
  type-specific access), and [1] a system-typed pointer to the new wrapping
  diagram. */
  template <typename MySystem>
  std::tuple<MySystem*, System<double>*> BurySystem(
      std::unique_ptr<MySystem> my_system) {
    MySystem* alias = my_system.get();
    std::unique_ptr<System<double>> owned_system(std::move(my_system));
    for (int k = 0; k < external_depth_; ++k) {
      DiagramBuilder<double> builder;
      auto* system = builder.AddSystem(std::move(owned_system));
      ReExportAllPorts(&builder, *system);
      owned_system = builder.Build();
    }
    auto* nest = builder_->AddSystem(std::move(owned_system));
    return std::tie(alias, nest);
  }

  void Finalize() {
    plant_->Finalize();

    // Wrapping the plant requires re-exporting of selected ports. The logic
    // here is different from BurySystem because it needs to preserve the
    // plant, scene graph, and their port connections.
    if (plant_depth_ > 0) {
      ReExportAllPorts(builder_.get(), *plant_);
      auto wrapper = builder_->Build();
      for (int k = 1; k < plant_depth_; ++k) {
        DiagramBuilder<double> builder;
        auto* system = builder.AddSystem(std::move(wrapper));
        ReExportAllPorts(&builder, *system);
        wrapper = builder.Build();
      }
      builder_ = std::make_unique<DiagramBuilder<double>>();
      plant_nest_ = builder_->AddSystem(std::move(wrapper));
    }
  }

  /* Creates the system diagram and sets up a simulation with CENIC. */
  void Build() {
    if (!plant_->is_finalized()) {
      Finalize();
    }

    diagram_ = builder_->Build();
    std::unique_ptr<Context<double>> context = diagram_->CreateDefaultContext();

    // Set initial conditions.
    plant_context_ = &plant_->GetMyMutableContextFromRoot(context.get());
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

  // Always available (after Setup).
  int plant_depth_{0};
  int external_depth_{0};
  bool have_scene_graph_{false};
  // A direct reference to the plant. Use it for type-specific API access.
  MultibodyPlant<double>* plant_{nullptr};
  // The scene graph, iff have_scene_graph_.
  geometry::SceneGraph<double>* scene_graph_{nullptr};

  // Depending on the value of plant_depth_, plant_nest_ will either be a
  // pointer to the plant (depth 0), or a pointer to a stack of diagrams
  // wrapping plant and scene graph, with all of their available ports
  // re-exported. Use it for establishing top-level connections between wrapped
  // systems.
  System<double>* plant_nest_{nullptr};
};

class DoublePendulum : public SimulationTestScenario {
 protected:
  void AddModels() override {
    Parser(builder_.get()).AddModelsFromString(kDoublePendulumMjcf, "xml");
  }

  void SetInitialConditions() override {
    plant_->SetPositions(plant_context_, Vector2d(M_PI / 4, M_PI / 2));
    plant_->SetVelocities(plant_context_, Vector2d(0.1, 0.2));
  }
};

class BallOnTable : public SimulationTestScenario {
 protected:
  void AddModels() override {
    Parser(builder_.get()).AddModelsFromString(kBallOnTableMjcf, "xml");
  }

  void SetInitialConditions() override {
    plant_->SetVelocities(plant_context_, VectorXd::Zero(6));
  }
};

/* Checks that the integrator complains when no initial step size is
configured. */
TEST_P(DoublePendulum, NoStep) {
  Build();
  integrator_ = &simulator_->reset_integrator<CenicIntegrator<double>>();
  DRAKE_EXPECT_THROWS_MESSAGE(simulator_->AdvanceTo(1e-2),
                              ".*Neither.*step size.*set.*");
}

/* Checks that the integrator reports expected statistics, and that they may be
reset. */
TEST_P(DoublePendulum, Stats) {
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
TEST_P(DoublePendulum, Parameters) {
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
TEST_P(DoublePendulum, FixedStep) {
  const double time_step = 1e-2;
  const double simulation_time = 0.5;
  Build();

  // Simulate with CENIC in fixed-step mode.
  const VectorXd x_initial = plant_->GetPositionsAndVelocities(*plant_context_);
  integrator_->set_maximum_step_size(time_step);
  integrator_->set_fixed_step_mode(true);
  simulator_->AdvanceTo(simulation_time);
  EXPECT_EQ(integrator_->get_num_steps_taken(),
            static_cast<int>(simulation_time / time_step));
  const VectorXd x_final = plant_->GetPositionsAndVelocities(*plant_context_);

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
TEST_P(DoublePendulum, AccuracyTrends) {
  // This test is relatively slow; skip extra nesting variations.
  if (plant_depth_ + external_depth_ > 0) {
    return;
  }
  const double simulation_time = 2.0;

  // Remove joint damping for this test.
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    joint.set_default_damping_vector(VectorXd::Zero(1));
  }
  Build();

  // Record the initial system energy (kinetic + potential).
  const VectorXd x_initial = plant_->GetPositionsAndVelocities(*plant_context_);
  const double initial_energy = plant_->CalcPotentialEnergy(*plant_context_) +
                                plant_->CalcKineticEnergy(*plant_context_);

  // Simulate with error control at a very loose accuracy.
  integrator_->set_target_accuracy(1e-1);
  simulator_->AdvanceTo(simulation_time);

  // Calculate a norm of the error estimate for use in trend checking.
  auto error_norm = [](const systems::ContinuousState<double>* estimate) {
    return estimate->CopyToVector().norm();
  };

  const int num_steps_loose = integrator_->get_num_steps_taken();
  const double final_energy_loose =
      plant_->CalcPotentialEnergy(*plant_context_) +
      plant_->CalcKineticEnergy(*plant_context_);
  const double error_loose = error_norm(integrator_->get_error_estimate());

  // Simulate again with a much tighter accuracy.
  simulator_->get_mutable_context().SetTime(0.0);
  plant_->SetPositionsAndVelocities(plant_context_, x_initial);
  integrator_->set_target_accuracy(1e-5);
  simulator_->Initialize();
  simulator_->AdvanceTo(simulation_time);

  const int num_steps_tight = integrator_->get_num_steps_taken();
  const double final_energy_tight =
      plant_->CalcPotentialEnergy(*plant_context_) +
      plant_->CalcKineticEnergy(*plant_context_);
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
TEST_P(DoublePendulum, ExternalActuation) {
  // Add some actuators to the plant.
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    plant_->AddJointActuator(joint.name() + "_actuator", joint);
  }
  Finalize();
  EXPECT_EQ(plant_->num_actuators(), 2);

  // Set up a PID controller.
  const Vector2d Kp(1.24, 0.79);
  const Vector2d Kd(0.15, 0.03);
  const Vector2d Ki(0.5, 0.4);
  const Vector4d x_nom(M_PI_2, -M_PI_2, 0.0, 0.0);

  auto [target_sender, target_nest] =
      BurySystem(std::make_unique<ConstantVectorSource<double>>(x_nom));
  auto [pid_controller, pid_nest] =
      BurySystem(std::make_unique<PidController<double>>(Kp, Ki, Kd));

  builder_->Connect(target_nest->get_output_port(),
                    pid_nest->GetInputPort("desired_state"));
  builder_->Connect(plant_nest_->GetOutputPort("state"),
                    pid_nest->GetInputPort("estimated_state"));
  builder_->Connect(pid_nest->GetOutputPort("control"),
                    plant_nest_->GetInputPort("actuation"));

  Build();

  // Simulate for a few seconds.
  simulator_->AdvanceTo(20.0);

  // We should get close-ish to the target state.
  const VectorXd x_final = plant_->GetPositionsAndVelocities(*plant_context_);
  EXPECT_TRUE(
      CompareMatrices(x_final, x_nom, 1e-3, MatrixCompareType::absolute));

  // The PID controller's external state (due to integral terms) should have
  // advanced. In particular, we should be close to the steady-state, where
  // integral terms -Ki * z balance gravity.
  const VectorXd z = simulator_->get_context()
                         .get_continuous_state()
                         .get_misc_continuous_state()
                         .CopyToVector();
  const VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*plant_context_);
  const VectorXd tau_integral = -(Ki.asDiagonal() * z);
  EXPECT_TRUE(
      CompareMatrices(tau_integral, tau_g, 1e-3, MatrixCompareType::absolute));
}

/* Checks integration of an external (non-plant) system with second-order (q, v)
continuous state.*/
// TODO(#23921): Revisit this test when implementing an error estimate that
// accounts for external systems.
TEST_P(DoublePendulum, PositionVelocityExternalSystem) {
  auto [spring_mass_system, spring_mass_nest] =
      BurySystem(std::make_unique<SpringMassSystem<double>>(100.0, 1.0));
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

/* Checks that the integrator can properly handle zero-duration steps. */
TEST_P(DoublePendulum, ZeroStep) {
  Build();

  simulator_->AdvanceTo(1.0);

  // Generic integrator behavior: accept 0-duration step with no change to time
  // or continuous state, and with error estimate updated to all 0.
  const VectorXd q = plant_->GetPositions(*plant_context_);
  integrator_->IntegrateWithMultipleStepsToTime(
      simulator_->get_context().get_time());
  EXPECT_EQ(simulator_->get_context().get_time(), 1.0);
  const VectorXd q_after = plant_->GetPositions(*plant_context_);
  EXPECT_TRUE(CompareMatrices(q_after, q));
  VectorXd error_estimate(integrator_->get_error_estimate()->CopyToVector());
  EXPECT_EQ(error_estimate.norm(), 0);
}

/* Checks that the integrator can enforce joint limits. */
TEST_P(DoublePendulum, JointLimits) {
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    joint.set_position_limits(VectorXd::Constant(1, 0.2 * (i + 1)),
                              VectorXd::Constant(1, 3.1));
  }
  Build();

  simulator_->AdvanceTo(1.0);

  // The final configuration should be resting on the lower joint limits.
  const VectorXd q = plant_->GetPositions(*plant_context_);
  EXPECT_NEAR(q(0), 0.2, 1e-5);
  EXPECT_NEAR(q(1), 0.4, 1e-5);
}

/* Checks that the integrator with joint limits in force can resolve initial
conditions that are extreme violations of the constraints. See #24403. */
TEST_P(DoublePendulum, JointLimitsExtremeInitialConditions) {
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    joint.set_position_limits(VectorXd::Constant(1, 0.2 * (i + 1)),
                              VectorXd::Constant(1, 3.1));
  }
  Build();

  // An extreme constraint violation can push the integrator into a regime
  // where it may attempt very small internal time steps. Early implementations
  // of the limit constraint could fail to make progress and eventually throw
  // by attempting a prohibitively small step (#24403).
  const VectorXd q_bad = VectorXd::Zero(2);
  plant_->SetPositions(plant_context_, q_bad);
  EXPECT_NO_THROW(simulator_->AdvanceTo(1.0));
}

/* Checks that the integrator joint limit constraints also support joint
locking. */
TEST_P(DoublePendulum, JointLimitsAndJointLocking) {
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    joint.set_position_limits(VectorXd::Constant(1, 0.2 * (i + 1)),
                              VectorXd::Constant(1, 3.1));
  }
  Build();

  // Check joint locking support with joint limit constraint.
  const auto& joint1 = plant_->GetJointByName("joint1");
  joint1.Lock(plant_context_);
  EXPECT_NO_THROW(simulator_->AdvanceTo(1.0));
}

/* Checks that effort limits are enforced by the integrator. */
TEST_P(DoublePendulum, EffortLimits) {
  // Add some actuators with tight effort limits to the plant.
  for (multibody::JointIndex i : plant_->GetJointIndices()) {
    multibody::Joint<double>& joint = plant_->get_mutable_joint(i);
    plant_->AddJointActuator(joint.name() + "_actuator", joint, 0.1 * (i + 1));

    // Disable joint damping so it's easier to derive the applied torques
    // analytically.
    joint.set_default_damping_vector(VectorXd::Zero(1));
  }
  plant_->SetUseSampledOutputPorts(true);
  Build();

  // Apply large torques that would exceed the effort limits.
  plant_->get_actuation_input_port().FixValue(plant_context_,
                                              VectorXd::Constant(2, 10.0));

  // Compute some dynamics terms.
  const VectorXd q0 = plant_->GetPositions(*plant_context_);
  const VectorXd v0 = plant_->GetVelocities(*plant_context_);
  MatrixXd M(2, 2);
  VectorXd k(2);
  MultibodyForces<double> f_ext(*plant_);
  plant_->CalcMassMatrix(*plant_context_, &M);
  plant_->CalcForceElementsContribution(*plant_context_, &f_ext);
  k = plant_->CalcInverseDynamics(*plant_context_, VectorXd::Zero(2), f_ext);

  // Simulate for a single step.
  const double h = 0.01;
  integrator_->set_fixed_step_mode(true);
  integrator_->set_maximum_step_size(h);
  simulator_->AdvanceTo(h);
  EXPECT_EQ(integrator_->get_num_steps_taken(), 1);

  // Requested actuation should exceed effort limits.
  const VectorXd u_requested =
      plant_->get_actuation_input_port().Eval(*plant_context_);
  EXPECT_TRUE(u_requested[0] > 0.1);
  EXPECT_TRUE(u_requested[1] > 0.2);

  // Applied actuation should be clamped to effort limits.
  const VectorXd v = plant_->GetVelocities(*plant_context_);
  const VectorXd u_applied = M * (v - v0) / h + k;

  EXPECT_NEAR(u_applied[0], 0.1, 10 * kEpsilon);
  EXPECT_NEAR(u_applied[1], 0.2, 10 * kEpsilon);

  // The sampled net_actuation port reports the clamped actuation, and the
  // sampled generalized_acceleration port reports the step's acceleration.
  const VectorXd u_net =
      plant_->get_net_actuation_output_port().Eval(*plant_context_);
  EXPECT_TRUE(CompareMatrices(u_net, Vector2d(0.1, 0.2), 1e-6));
  const VectorXd vdot =
      plant_->get_generalized_acceleration_output_port().Eval(*plant_context_);
  EXPECT_TRUE(CompareMatrices(vdot, (v - v0) / h, 1e-6));
}

/* Checks that coupler constraints are handled correctly by the integrator. */
TEST_P(DoublePendulum, CoupledJoints) {
  const double gear_ratio = 0.5;
  const auto& joint1 = plant_->GetJointByName("joint1");
  const auto& joint2 = plant_->GetJointByName("joint2");
  plant_->AddCouplerConstraint(joint1, joint2, gear_ratio);
  Build();
  const VectorXd q0 = plant_->GetPositions(*plant_context_);

  // Initial conditions should satisfy the coupler constraint.
  EXPECT_NEAR(q0(0), gear_ratio * q0(1), kEpsilon);

  simulator_->AdvanceTo(1.0);

  // Verify that the constraint is satisfied at the end of the simulation.  The
  // constraint is essentially enforced by a very stiff spring, so we shouldn't
  // expect exact satisfaction.
  const VectorXd q = plant_->GetPositions(*plant_context_);
  EXPECT_NEAR(q(0), gear_ratio * q(1), 1e-5);
}

/* Checks that the integrator with coupler constraints in force can resolve
initial conditions that are extreme violations of the constraints. See
#24403. */
TEST_P(DoublePendulum, CoupledJointsExtremeInitialConditions) {
  const double gear_ratio = 0.5;
  const auto& joint1 = plant_->GetJointByName("joint1");
  const auto& joint2 = plant_->GetJointByName("joint2");
  plant_->AddCouplerConstraint(joint1, joint2, gear_ratio);
  Build();
  const VectorXd q0 = plant_->GetPositions(*plant_context_);

  // An extreme constraint violation can push the integrator into a regime
  // where it may attempt very small internal time steps. Early implementations
  // of the coupler constraint could fail to make progress and eventually throw
  // by attempting a prohibitively small step (#24403).
  VectorXd q_bad = q0;
  q_bad(1) += 0.2;
  plant_->SetPositions(plant_context_, q_bad);
  EXPECT_NO_THROW(simulator_->AdvanceTo(1.0));
}

/* Checks that coupler constraints and joint locking don't blow up the
integrator. */
TEST_P(DoublePendulum, CoupledJointsAndJointLocking) {
  const double gear_ratio = 0.5;
  const auto& joint1 = plant_->GetJointByName("joint1");
  const auto& joint2 = plant_->GetJointByName("joint2");
  plant_->AddCouplerConstraint(joint1, joint2, gear_ratio);
  Build();
  const VectorXd q0 = plant_->GetPositions(*plant_context_);

  // Check joint locking support with coupler constraint, to the extent that it
  // does not crash nor throw.
  joint1.Lock(plant_context_);
  EXPECT_NO_THROW(simulator_->AdvanceTo(1.0));
}

/* Checks that weld constraints and joint locking don't blow up the
integrator. */
TEST_P(DoublePendulum, WeldedBodies) {
  const auto& link1 = plant_->GetBodyByName("link1");
  const auto& link2 = plant_->GetBodyByName("link2");
  plant_->AddWeldConstraint(link1, RigidTransformd(), link2, RigidTransformd());
  Build();
  const VectorXd q0 = plant_->GetPositions(*plant_context_);
  simulator_->AdvanceTo(1.0);

  // Check joint locking support with weld constraint, to the extent that it
  // does not crash nor throw.
  const auto& joint1 = plant_->GetJointByName("joint1");
  joint1.Lock(plant_context_);
  EXPECT_NO_THROW(simulator_->AdvanceTo(1.1));
}

/* Checks that a ball rolling along the ground produces the expected result. */
TEST_P(BallOnTable, RollingContact) {
  Build();

  // Place the ball on the ground, with initial linear and angular velocity
  // for rolling without slipping.
  const double radius = 0.1;
  const double roll_speed = 0.8;
  VectorXd q0 = plant_->GetPositions(*plant_context_);
  q0[4] = 0.0;     // x position
  q0[6] = radius;  // z position
  plant_->SetPositions(plant_context_, q0);
  VectorXd v0 = VectorXd::Zero(6);
  v0[1] = roll_speed / radius;
  v0[3] = roll_speed;
  plant_->SetVelocities(plant_context_, v0);

  // Compare a simulation with the analytical solution.
  const double simulation_time = 1.0;
  const double x_expected = roll_speed * simulation_time;
  double z_expected{};
  if (have_scene_graph_) {
    z_expected = radius;
  } else {
    // Contact can't be evaluated, so the ball is in free-fall.
    z_expected =
        radius - 0.5 * UniformGravityFieldElement<double>::kDefaultStrength *
                     std::pow(simulation_time, 2.0);
    // Matching the analytical result here requires tighter accuracy.
    integrator_->set_target_accuracy(5e-7);
  }

  simulator_->AdvanceTo(simulation_time);

  const VectorXd q = plant_->GetPositions(*plant_context_);
  const double x_actual = q[4];
  const double z_actual = q[6];

  EXPECT_NEAR(x_actual, x_expected, 1e-3);
  EXPECT_NEAR(z_actual, z_expected, 1e-3);

  // Check joint locking support with contact (specifically as implemented by
  // the ICF patch constraint).
  const auto& ball = plant_->GetBodyByName("link1");
  ball.Lock(plant_context_);
  EXPECT_NO_THROW(simulator_->AdvanceTo(simulation_time + 0.1));
}

/* Checks that quaternions always remain normalized. */
TEST_P(BallOnTable, QuaternionNormalization) {
  Build();

  // Set initial velocities that will yeet the ball into the distance while
  // spinning wildly.
  VectorXd v0(6);
  v0 << 100.0, -218.0, -82.0, 10.0, 20.0, 500.0;
  plant_->SetVelocities(plant_context_, v0);

  // Run the simulation briefly with fixed (large) steps.
  integrator_->set_maximum_step_size(0.1);
  integrator_->set_fixed_step_mode(true);
  simulator_->AdvanceTo(1.0);

  // Quaternions should remain normalized.
  const VectorXd q = plant_->GetPositions(*plant_context_);
  const Vector4d quat = q.segment<4>(0);
  const double quat_norm = quat.norm();
  EXPECT_NEAR(quat_norm, 1.0, 10 * kEpsilon);
}

/* Verifies that we can simulate non-actuator external forces. */
TEST_P(BallOnTable, ExternalForces) {
  const double simulation_time = 0.3;
  Build();
  const double initial_height = plant_->GetPositions(*plant_context_)[6];

  // Apply a constant upward force.
  VectorXd f_ext = VectorXd::Zero(plant_->num_velocities());
  f_ext[5] = 50.0;  // +z direction
  plant_->get_applied_generalized_force_input_port().FixValue(plant_context_,
                                                              f_ext);
  integrator_->set_target_accuracy(1e-5);
  simulator_->AdvanceTo(simulation_time);

  // Compare with the analytical solution.
  const double m = plant_->CalcTotalMass(*plant_context_);
  const double g = 9.81;
  const double net_acceleration = -g + 50.0 / m;
  const double expected_height = initial_height + 0.5 * net_acceleration *
                                                      simulation_time *
                                                      simulation_time;
  const double final_height = plant_->GetPositions(*plant_context_)[6];
  EXPECT_NEAR(final_height, expected_height, 1e-3);

  // Check joint locking support with external forces (specifically as
  // implemented by the ICF gain constraint).
  const auto& ball = plant_->GetBodyByName("link1");
  ball.Lock(plant_context_);
  EXPECT_NO_THROW(simulator_->AdvanceTo(simulation_time + 0.1));
}

/* Returns the sum of the masses of all bodies in the plant, except world. */
double CalcTotalMass(const MultibodyPlant<double>& plant,
                     const Context<double>& context) {
  double mass = 0.0;
  for (BodyIndex i(1); i < plant.num_bodies(); ++i) {
    mass += plant.get_body(i).get_mass(context);
  }
  return mass;
}

const std::vector<SpatialForce<double>>& EvalReactionForces(
    const MultibodyPlant<double>& plant, const Context<double>& context) {
  return plant.get_reaction_forces_output_port()
      .Eval<std::vector<SpatialForce<double>>>(context);
}

const ContactResults<double>& EvalContactResults(
    const MultibodyPlant<double>& plant, const Context<double>& context) {
  return plant.get_contact_results_output_port().Eval<ContactResults<double>>(
      context);
}

/* Checks the sampled dynamics output ports of a continuous plant integrated by
CENIC, for a system without contact or constraints. */
TEST_P(DoublePendulum, SampledOutputPorts) {
  plant_->SetUseSampledOutputPorts(true);
  Build();
  const int nv = plant_->num_velocities();
  const auto& vdot_port = plant_->get_generalized_acceleration_output_port();

  // Before the first step, the sampled dynamics ports are all zero or empty.
  for (const SpatialForce<double>& F :
       EvalReactionForces(*plant_, *plant_context_)) {
    EXPECT_TRUE(F.get_coeffs().isZero());
  }
  EXPECT_TRUE(vdot_port.Eval(*plant_context_).isZero());
  EXPECT_EQ(
      EvalContactResults(*plant_, *plant_context_).num_point_pair_contacts(),
      0);

  // Take a single fixed step of size h.
  const double h = 1e-3;
  const VectorXd x0 = plant_->GetPositionsAndVelocities(*plant_context_);
  integrator_->set_fixed_step_mode(true);
  integrator_->set_maximum_step_size(h);
  simulator_->AdvanceTo(h);
  ASSERT_EQ(integrator_->get_num_steps_taken(), 1);
  const VectorXd v0 = x0.tail(nv);
  const VectorXd v1 = plant_->GetVelocities(*plant_context_);

  // The sampled acceleration is exactly the step's finite difference.
  EXPECT_TRUE(CompareMatrices(vdot_port.Eval(*plant_context_), (v1 - v0) / h,
                              100 * kEpsilon));

  // The sampled reaction forces match the live reaction forces of an identical
  // (unsampled) continuous plant at the state where the step started, up to
  // O(h) differences (e.g., joint damping is implicit in the ICF step).
  MultibodyPlant<double> reference_plant(0.0);
  Parser(&reference_plant).AddModelsFromString(kDoublePendulumMjcf, "xml");
  reference_plant.Finalize();
  auto reference_context = reference_plant.CreateDefaultContext();
  reference_plant.SetPositionsAndVelocities(reference_context.get(), x0);
  const std::vector<SpatialForce<double>>& expected =
      EvalReactionForces(reference_plant, *reference_context);
  const std::vector<SpatialForce<double>>& actual =
      EvalReactionForces(*plant_, *plant_context_);
  ASSERT_EQ(actual.size(), expected.size());
  for (int i = 0; i < std::ssize(actual); ++i) {
    EXPECT_TRUE(CompareMatrices(actual[i].get_coeffs(),
                                expected[i].get_coeffs(), 1e-3,
                                MatrixCompareType::absolute));
    EXPECT_GT(actual[i].translational().norm(), 1e-3);
  }

  // Continue in error-controlled mode. The sample is refreshed on every step
  // and remains physically reasonable.
  integrator_->set_fixed_step_mode(false);
  integrator_->set_maximum_step_size(0.1);
  simulator_->AdvanceTo(0.5);
  const double weight = CalcTotalMass(*plant_, *plant_context_) *
                        UniformGravityFieldElement<double>::kDefaultStrength;
  const std::vector<SpatialForce<double>>& reactions =
      EvalReactionForces(*plant_, *plant_context_);
  EXPECT_TRUE(reactions[0].get_coeffs().allFinite());
  EXPECT_GT(reactions[0].translational().norm(), 0.1 * weight);
  EXPECT_LT(reactions[0].translational().norm(), 10.0 * weight);
  EXPECT_FALSE(vdot_port.Eval(*plant_context_).isZero());
}

/* The motivating case for sampled reporting: a near-rigid weld constraint with
a small initial violation. Evaluating the ICF constraint force law at the
current configuration would report a huge (unphysical) force; the impulse of the
solve that actually advanced the state is O(weight). */
TEST_P(DoublePendulum, SampledReactionForcesWithWeldConstraint) {
  // Find the pose of link2 in link1 at the zero configuration.
  RigidTransformd X_L1L2;
  {
    MultibodyPlant<double> scratch(0.0);
    Parser(&scratch).AddModelsFromString(kDoublePendulumMjcf, "xml");
    scratch.Finalize();
    auto context = scratch.CreateDefaultContext();
    X_L1L2 = scratch.CalcRelativeTransform(
        *context, scratch.GetBodyByName("link1").body_frame(),
        scratch.GetBodyByName("link2").body_frame());
  }

  // Weld link2 to link1 at that pose, offset by a tiny violation.
  const double violation = 1e-8;
  const auto& link1 = plant_->GetBodyByName("link1");
  const auto& link2 = plant_->GetBodyByName("link2");
  plant_->AddWeldConstraint(
      link1, X_L1L2 * RigidTransformd(Eigen::Vector3d(0.0, 0.0, violation)),
      link2, RigidTransformd());
  plant_->SetUseSampledOutputPorts(true);
  Build();

  // Start at the zero configuration for joint2 (consistent with the weld up
  // to the violation) with joint2 at rest, and joint1 swinging.
  plant_->SetPositions(plant_context_, Vector2d(M_PI / 4, 0.0));
  plant_->SetVelocities(plant_context_, Vector2d(0.1, 0.0));
  const double weight = CalcTotalMass(*plant_, *plant_context_) *
                        UniformGravityFieldElement<double>::kDefaultStrength;

  const double h = 1e-3;
  integrator_->set_fixed_step_mode(true);
  integrator_->set_maximum_step_size(h);
  simulator_->AdvanceTo(h);
  ASSERT_EQ(integrator_->get_num_steps_taken(), 1);

  // The weld holds joint2 (near) still.
  const VectorXd v = plant_->GetVelocities(*plant_context_);
  EXPECT_NEAR(v[1], 0.0, 1e-3);

  // Reaction forces are O(weight); both joints carry load.
  const std::vector<SpatialForce<double>>& reactions =
      EvalReactionForces(*plant_, *plant_context_);
  ASSERT_EQ(std::ssize(reactions), 2);
  for (const SpatialForce<double>& F : reactions) {
    EXPECT_TRUE(F.get_coeffs().allFinite());
    EXPECT_LT(F.translational().norm(), 10.0 * weight);
    EXPECT_GT(F.translational().norm(), 0.01 * weight);
  }
  const VectorXd vdot =
      plant_->get_generalized_acceleration_output_port().Eval(*plant_context_);
  EXPECT_TRUE(vdot.allFinite());
  EXPECT_LT(vdot.cwiseAbs().maxCoeff(), 1e3);
}

/* Checks sampled reaction forces when joint locking reduces the ICF problem:
the locked joint must still report the load it carries. */
TEST_P(DoublePendulum, SampledReactionForcesWithJointLocking) {
  plant_->SetUseSampledOutputPorts(true);
  Build();
  plant_->SetPositions(plant_context_, Vector2d(M_PI / 4, 0.3));
  plant_->SetVelocities(plant_context_, Vector2d(0.5, 0.0));
  const auto& joint2 = plant_->GetJointByName("joint2");
  joint2.Lock(plant_context_);

  const double h = 1e-3;
  integrator_->set_fixed_step_mode(true);
  integrator_->set_maximum_step_size(h);
  simulator_->AdvanceTo(h);
  ASSERT_EQ(integrator_->get_num_steps_taken(), 1);

  const VectorXd vdot =
      plant_->get_generalized_acceleration_output_port().Eval(*plant_context_);
  EXPECT_EQ(vdot[1], 0.0);  // Locked.
  EXPECT_NE(vdot[0], 0.0);

  // The locked joint2 carries link2: its reaction force is nonzero, and the
  // reaction torque about the hinge axis (y) is nonzero, unlike for a free
  // hinge.
  const std::vector<SpatialForce<double>>& reactions =
      EvalReactionForces(*plant_, *plant_context_);
  const SpatialForce<double>& F_joint2 = reactions[joint2.ordinal()];
  EXPECT_TRUE(F_joint2.get_coeffs().allFinite());
  EXPECT_GT(F_joint2.translational().norm(), 1e-3);
  EXPECT_GT(std::abs(F_joint2.rotational()[1]), 1e-4);

  // For comparison, the unlocked hinge transmits only its (small) damping
  // torque about its axis, up to the O(h) inconsistency between the implicit
  // ICF step and inverse dynamics at the start of the step.
  const auto& joint1 = plant_->GetJointByName<RevoluteJoint>("joint1");
  const double v1_0 = 0.5;  // Initial rate of joint1, set above.
  EXPECT_NEAR(reactions[joint1.ordinal()].rotational()[1],
              -joint1.default_damping() * v1_0, 1e-4);
}

INSTANTIATE_TEST_SUITE_P(
    AllParamsPendulum, DoublePendulum,
    testing::Combine(testing::Range(0, 3), testing::Range(0, 3),
                     testing::Bool()),
    [](const testing::TestParamInfo<DoublePendulum::ParamType>& x) {
      return fmt::format("plant_depth_{}_external_depth_{}_scene_graph_{}",
                         std::get<0>(x.param), std::get<1>(x.param),
                         std::get<2>(x.param));
    });

/* Checks the sampled contact-related output ports for a ball resting on the
table. When `hydroelastic` is true, the ball and table are made compliant
hydroelastic so that contact is reported as a contact surface; otherwise as a
point pair. */
class BallOnTableSampled : public BallOnTable {
 protected:
  void CheckSampledContact(bool hydroelastic) {
    if (have_scene_graph_ && hydroelastic) {
      geometry::SceneGraphConfig config;
      config.default_proximity_properties.compliance_type = "compliant";
      config.default_proximity_properties.resolution_hint = 0.05;
      scene_graph_->set_config(config);
    }
    plant_->SetUseSampledOutputPorts(true);
    Build();

    // Place the ball on the table at rest.
    const double radius = 0.1;
    VectorXd q0 = plant_->GetPositions(*plant_context_);
    q0[6] = radius;
    plant_->SetPositions(plant_context_, q0);
    const auto& ball = plant_->GetBodyByName("link1");
    const double g = UniformGravityFieldElement<double>::kDefaultStrength;
    const double weight = ball.get_mass(*plant_context_) * g;

    // Let it settle (error-controlled steps).
    simulator_->AdvanceTo(1.0);

    const ContactResults<double>& contact_results =
        EvalContactResults(*plant_, *plant_context_);
    EXPECT_EQ(contact_results.plant(), plant_);
    const VectorXd vdot =
        plant_->get_generalized_acceleration_output_port().Eval(
            *plant_context_);
    const VectorXd tau_contact =
        plant_
            ->get_generalized_contact_forces_output_port(ball.model_instance())
            .Eval(*plant_context_);
    const std::vector<SpatialForce<double>>& reactions =
        EvalReactionForces(*plant_, *plant_context_);
    ASSERT_EQ(std::ssize(reactions), 1);

    if (!have_scene_graph_) {
      // Without geometry, the ball is in free fall.
      EXPECT_EQ(contact_results.num_point_pair_contacts(), 0);
      EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 0);
      EXPECT_TRUE(CompareMatrices(
          vdot, (VectorXd(6) << 0, 0, 0, 0, 0, -g).finished(), 1e-9));
      EXPECT_TRUE(tau_contact.isZero());
      return;
    }

    // At rest, the contact force balances the weight, the ball does not
    // accelerate, and the free joint transmits no force.
    const double tolerance = 0.05 * weight;
    EXPECT_LT(vdot.cwiseAbs().maxCoeff(), 0.05 * g);
    EXPECT_LT(reactions[0].get_coeffs().norm(), tolerance);
    EXPECT_TRUE(CompareMatrices(
        tau_contact, (VectorXd(6) << 0, 0, 0, 0, 0, weight).finished(),
        tolerance));
    const auto& accelerations =
        plant_->get_body_spatial_accelerations_output_port()
            .Eval<std::vector<SpatialAcceleration<double>>>(*plant_context_);
    EXPECT_LT(accelerations[ball.index()].get_coeffs().norm(), 0.05 * g);

    // The contact results report the force on the ball.
    const geometry::SceneGraphInspector<double>& inspector =
        scene_graph_->model_inspector();
    if (hydroelastic) {
      ASSERT_EQ(contact_results.num_hydroelastic_contacts(), 1);
      EXPECT_EQ(contact_results.num_point_pair_contacts(), 0);
      const HydroelasticContactInfo<double>& info =
          contact_results.hydroelastic_contact_info(0);
      const RigidBody<double>* body_M = plant_->GetBodyFromFrameId(
          inspector.GetFrameId(info.contact_surface().id_M()));
      ASSERT_NE(body_M, nullptr);
      // F_Ac_W is the force on body M (== A). The force on the ball is +weight.
      const double sign = (body_M->index() == ball.index()) ? 1.0 : -1.0;
      EXPECT_TRUE(CompareMatrices(sign * info.F_Ac_W().translational(),
                                  Eigen::Vector3d(0, 0, weight), tolerance));
      EXPECT_LT(info.F_Ac_W().rotational().norm(), tolerance * radius);
    } else {
      ASSERT_EQ(contact_results.num_point_pair_contacts(), 1);
      EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 0);
      const PointPairContactInfo<double>& info =
          contact_results.point_pair_contact_info(0);
      const RigidBody<double>* body_A = plant_->GetBodyFromFrameId(
          inspector.GetFrameId(info.point_pair().id_A));
      const RigidBody<double>* body_B = plant_->GetBodyFromFrameId(
          inspector.GetFrameId(info.point_pair().id_B));
      ASSERT_NE(body_A, nullptr);
      ASSERT_NE(body_B, nullptr);
      EXPECT_EQ(info.bodyA_index(), body_A->index());
      EXPECT_EQ(info.bodyB_index(), body_B->index());
      // contact_force() is the force on body B. The force on the ball is
      // +weight.
      const double sign = (info.bodyB_index() == ball.index()) ? 1.0 : -1.0;
      EXPECT_TRUE(CompareMatrices(sign * info.contact_force(),
                                  Eigen::Vector3d(0, 0, weight), tolerance));
      // The contact point is at the bottom of the ball, near the table.
      EXPECT_NEAR(info.contact_point()[2], 0.0, 1e-3);
      EXPECT_NEAR(info.separation_speed(), 0.0, 1e-3);
      EXPECT_NEAR(info.slip_speed(), 0.0, 1e-3);
    }

    // Lock the ball. As for a discrete plant, MultibodyPlant filters out
    // contacts between bodies that both have no degrees of freedom (here the
    // locked ball and the world), so the sample reports no contact, no
    // contact forces, and no acceleration -- rather than stale values from
    // before the lock.
    ball.Lock(plant_context_);
    simulator_->AdvanceTo(1.1);
    const ContactResults<double>& locked_results =
        EvalContactResults(*plant_, *plant_context_);
    EXPECT_EQ(locked_results.num_hydroelastic_contacts(), 0);
    EXPECT_EQ(locked_results.num_point_pair_contacts(), 0);
    EXPECT_TRUE(
        plant_
            ->get_generalized_contact_forces_output_port(ball.model_instance())
            .Eval(*plant_context_)
            .isZero());
    EXPECT_TRUE(plant_->get_generalized_acceleration_output_port()
                    .Eval(*plant_context_)
                    .isZero());
    // With no contact in the model, the locked joint now carries the weight.
    const SpatialForce<double>& F_locked =
        EvalReactionForces(*plant_, *plant_context_)[0];
    EXPECT_TRUE(CompareMatrices(F_locked.translational(),
                                Eigen::Vector3d(0, 0, weight), tolerance));
  }
};

TEST_P(BallOnTableSampled, PointContact) {
  CheckSampledContact(/* hydroelastic = */ false);
}

TEST_P(BallOnTableSampled, HydroelasticContact) {
  CheckSampledContact(/* hydroelastic = */ true);
}

INSTANTIATE_TEST_SUITE_P(
    AllParamsBallSampled, BallOnTableSampled,
    testing::Combine(testing::Range(0, 2), testing::Range(0, 1),
                     testing::Bool()),
    [](const testing::TestParamInfo<BallOnTableSampled::ParamType>& x) {
      return fmt::format("plant_depth_{}_external_depth_{}_scene_graph_{}",
                         std::get<0>(x.param), std::get<1>(x.param),
                         std::get<2>(x.param));
    });

INSTANTIATE_TEST_SUITE_P(
    AllParamsBall, BallOnTable,
    testing::Combine(testing::Range(0, 3), testing::Range(0, 3),
                     testing::Bool()),
    [](const testing::TestParamInfo<BallOnTable::ParamType>& x) {
      return fmt::format("plant_depth_{}_external_depth_{}_scene_graph_{}",
                         std::get<0>(x.param), std::get<1>(x.param),
                         std::get<2>(x.param));
    });

}  // namespace
}  // namespace cenic
}  // namespace multibody
}  // namespace drake
