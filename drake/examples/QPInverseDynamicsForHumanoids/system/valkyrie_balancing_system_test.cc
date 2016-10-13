#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/valkyrie_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_value_source.h"

namespace drake {

using systems::DiagramBuilder;
using systems::Diagram;
using systems::Simulator;
using systems::ExplicitEulerIntegrator;

namespace examples {
namespace qp_inverse_dynamics {

// In this test, the Valkyrie robot is initialized to a nominal configuration
// with zero velocities, and the qp controller is setup to track this
// state. The robot is then perturbed in velocity for the Torso Pitch joint.
// The test forward simulates the closed loop system for 2 seconds.
// The simulation does not perform forward dynamics computation, instead, it
// integrates the computed acceleration from the controller. This dummy
// simulation should be replaced later with real simulation.
// The controller should drive the position and velocity close to zero in 2
// seconds.
GTEST_TEST(Sys2QPInverseDynamicsController, Standing) {
  std::string urdf =
      GetDrakePath() +
      std::string(
          "/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
  RigidBodyTree robot(urdf);

  // Build diagram.
  DiagramBuilder<double> builder;
  QPControllerSystem* qp_con =
      builder.AddSystem(std::make_unique<QPControllerSystem>(robot));
  ValkyrieSystem* val_sim =
      builder.AddSystem(std::make_unique<ValkyrieSystem>(robot));
  PlanEvalSystem* plan_eval =
      builder.AddSystem(std::make_unique<PlanEvalSystem>(robot));

  builder.Connect(qp_con->get_output_port_qp_output(),
                  val_sim->get_input_port_qp_output());
  builder.Connect(val_sim->get_output_port_humanoid_status(),
                  qp_con->get_input_port_humanoid_status());
  builder.Connect(val_sim->get_output_port_humanoid_status(),
                  plan_eval->get_input_port_humanoid_status());
  builder.Connect(plan_eval->get_output_port_qp_input(),
                  qp_con->get_input_port_qp_input());

  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  // Setup simulation.
  Simulator<double> simulator(*diagram);
  systems::Context<double>* val_sim_context =
      diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                          val_sim);
  // Set initial state.
  std::unique_ptr<examples::qp_inverse_dynamics::HumanoidStatus> rs0 =
      val_sim->SetInitialCondition(val_sim_context);
  // Set plan eval's desired to the initial state.
  plan_eval->SetupDesired(*rs0);
  // Perturb the initial condition.
  val_sim->PerturbVelocity("torsoPitch", 0.1, val_sim_context);

  // Simulation.
  // dt = 4e-3 is picked arbitrarily to ensure the test finishes within a
  // reasonable amount of time.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(
      *diagram, 4e-3, simulator.get_mutable_context());
  simulator.Initialize();
  simulator.StepTo(2.0);

  // Check final state.
  // Since the feet have equality constraints set to 0 in the qp controller,
  // they should have no velocity after simulation.
  // Thus, the tolerances on feet velocities are smaller than those for the
  // generalized position and velocity.
  std::unique_ptr<examples::qp_inverse_dynamics::HumanoidStatus> rs1 =
      val_sim->GetHumanoidStatusFromContext(*val_sim_context);

  EXPECT_TRUE(rs1->foot(Side::LEFT).velocity().norm() < 1e-6);
  EXPECT_TRUE(rs1->foot(Side::RIGHT).velocity().norm() < 1e-6);

  EXPECT_TRUE(CompareMatrices(rs1->position(), rs1->GetNominalPosition(), 1e-4,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      rs1->velocity(), Eigen::VectorXd::Zero(rs1->robot().get_num_velocities()),
      1e-4, MatrixCompareType::absolute));
}

}  // end namespace qp_inverse_dynamics
}  // end namespace examples
}  // end namespace drake
