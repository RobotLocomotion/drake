#include "drake/systems/analysis/simulator.h"
#include "drake/common/drake_path.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/constant_value_source.h"
#include "drake/systems/framework/diagram_builder.h"

#include "sys2_dummy_val.h"
#include "sys2_qp.h"
#include "sys2_plan_eval.h"

#include "gtest/gtest.h"
#include "drake/common/eigen_matrix_compare.h"

using namespace drake;
using namespace systems;

static std::string urdf = drake::GetDrakePath() + std::string("/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
RigidBodyTree robot(urdf);

void const_acc_test() {
  DiagramBuilder<double> builder;

  example::qp_inverse_dynamics::QPOutput out(robot);
  out.mutable_vd() = Eigen::VectorXd::Constant(robot.get_num_velocities(), 0.01);

  ConstantValueSource<double> *const_qp_out = builder.AddSystem(std::make_unique<ConstantValueSource<double>>(std::unique_ptr<AbstractValue>(new Value<example::qp_inverse_dynamics::QPOutput>(out))));
  System2DummyValkyrieSim *val_sim = builder.AddSystem(std::make_unique<System2DummyValkyrieSim>(robot));

  builder.Connect(const_qp_out->get_output_port(0), val_sim->get_input_port(0));
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);

  // set initial state
  // probably should pass in sub context?
  val_sim->SetInitialCondition(simulator.get_mutable_context());

  simulator.request_initial_step_size_attempt(1e-2);
  simulator.Initialize();

  simulator.StepTo(0.1);
}

void close_loop_test() {
  DiagramBuilder<double> builder;
  System2QP *qp_con = builder.AddSystem(std::make_unique<System2QP>(robot));
  System2DummyValkyrieSim *val_sim = builder.AddSystem(std::make_unique<System2DummyValkyrieSim>(robot));
  System2PlanEval *plan_eval = builder.AddSystem(std::make_unique<System2PlanEval>(robot));

  builder.Connect(qp_con->get_output_port_qp_output(), val_sim->get_input_port_qp_output());
  builder.Connect(val_sim->get_output_port_humanoid_status(), qp_con->get_input_port_humanoid_status());
  builder.Connect(val_sim->get_output_port_humanoid_status(), plan_eval->get_input_port_humanoid_status());
  builder.Connect(plan_eval->get_output_port_qp_input(), qp_con->get_input_port_qp_input());

  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  // probably should pass in sub context?
  std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus> rs0 = val_sim->SetInitialCondition(simulator.get_mutable_context());
  plan_eval->SetupDesired(*rs0);
  val_sim->PerturbPosition("torsoPitch", 0.1, simulator.get_mutable_context());

  simulator.request_initial_step_size_attempt(4e-3);
  simulator.Initialize();
  simulator.StepTo(2.0);

  std::unique_ptr<example::qp_inverse_dynamics::HumanoidStatus> rs1 = val_sim->GetHumanoidStatusFromContext(simulator.get_context());

  EXPECT_TRUE(rs1->foot(Side::LEFT).velocity().norm() < 1e-6);
  EXPECT_TRUE(rs1->foot(Side::RIGHT).velocity().norm() < 1e-6);

  EXPECT_TRUE(drake::CompareMatrices(rs1->position(), rs1->GetNominalPosition(), 1e-4,
                                     drake::MatrixCompareType::absolute));
  EXPECT_TRUE(drake::CompareMatrices(
      rs1->velocity(), Eigen::VectorXd::Zero(rs1->robot().get_num_velocities()), 1e-4,
      drake::MatrixCompareType::absolute));
}

int main() {
  //const_acc_test();

  close_loop_test();

  return 0;
}
