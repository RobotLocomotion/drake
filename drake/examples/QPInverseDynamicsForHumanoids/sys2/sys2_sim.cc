#include "drake/systems/analysis/simulator.h"
#include "drake/common/drake_path.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/constant_value_source.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/sys2/sys2_dummy_val.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/sys2/sys2_qp.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/sys2/sys2_plan_eval.h"

#include "gtest/gtest.h"
#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace qp_inverse_dynamics {

GTEST_TEST(testSys2QPInverseDynamicsController, testStanding) {
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
  RigidBodyTree robot(urdf);

  // Build diagram.
  DiagramBuilder<double> builder;
  System2QP* qp_con = builder.AddSystem(std::make_unique<System2QP>(robot));
  System2DummyValkyrieSim* val_sim =
      builder.AddSystem(std::make_unique<System2DummyValkyrieSim>(robot));
  System2PlanEval* plan_eval =
      builder.AddSystem(std::make_unique<System2PlanEval>(robot));

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
  Context<double>* val_sim_context = diagram->GetMutableSubsystemContext(
      simulator.get_mutable_context(), val_sim);
  // Set initial state.
  std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus> rs0 =
      val_sim->SetInitialCondition(val_sim_context);
  // Set plan eval's desired to the initial state.
  plan_eval->SetupDesired(*rs0);
  // Perturb the initial condition.
  val_sim->PerturbVelocity("torsoPitch", 0.1, val_sim_context);

  // Simulation.
  simulator.request_initial_step_size_attempt(4e-3);
  simulator.set_integrator_type(IntegratorType::ExplicitEuler);
  simulator.Initialize();
  simulator.StepTo(2.0);

  // Check final state.
  std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus> rs1 =
      val_sim->GetHumanoidStatusFromContext(*val_sim_context);

  EXPECT_TRUE(rs1->foot(Side::LEFT).velocity().norm() < 1e-6);
  EXPECT_TRUE(rs1->foot(Side::RIGHT).velocity().norm() < 1e-6);

  EXPECT_TRUE(drake::CompareMatrices(rs1->position(), rs1->GetNominalPosition(),
                                     1e-4, drake::MatrixCompareType::absolute));
  EXPECT_TRUE(drake::CompareMatrices(
      rs1->velocity(), Eigen::VectorXd::Zero(rs1->robot().get_num_velocities()),
      1e-4, drake::MatrixCompareType::absolute));
}

}  // end namespace qp_inverse_dynamics
}  // end namespace example
}  // end namespace drake
