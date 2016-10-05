#include "drake/systems/analysis/simulator.h"
#include "drake/common/drake_path.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/constant_value_source.h"
#include "drake/systems/framework/diagram_builder.h"

#include "sys2_dummy_val.h"
#include "sys2_qp.h"
#include "sys2_plan_eval.h"

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
  val_sim->set_initial_state(simulator.get_mutable_context());

  simulator.request_initial_step_size_attempt(1e-2);
  simulator.Initialize();

  simulator.StepTo(0.1);
}

example::qp_inverse_dynamics::QPInput default_QP_input(const RigidBodyTree& r) {
  example::qp_inverse_dynamics::QPInput input(r);
  input.mutable_desired_comdd().setZero();
  input.mutable_w_com() = 1e3;

  input.mutable_desired_vd().setZero();
  input.mutable_w_vd() = 1;

  example::qp_inverse_dynamics:: DesiredBodyAcceleration pelvdd_d(*r.FindBody("pelvis"));
  pelvdd_d.mutable_weight() = 1e1;
  pelvdd_d.mutable_acceleration().setZero();
  input.mutable_desired_body_accelerations().push_back(pelvdd_d);

  example::qp_inverse_dynamics::DesiredBodyAcceleration torsodd_d(*r.FindBody("torso"));
  torsodd_d.mutable_weight() = 1e1;
  torsodd_d.mutable_acceleration().setZero();
  input.mutable_desired_body_accelerations().push_back(torsodd_d);

  input.mutable_w_basis_reg() = 1e-6;

  // Make contact points.
  example::qp_inverse_dynamics::ContactInformation left_foot_contact(
      *r.FindBody("leftFoot"), 4);
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(0.2, 0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(0.2, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(-0.05, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(-0.05, 0.05, -0.09));

  example::qp_inverse_dynamics::ContactInformation right_foot_contact(
      *r.FindBody("rightFoot"), 4);
  right_foot_contact.mutable_contact_points() =
      left_foot_contact.contact_points();

  input.mutable_contact_info().push_back(left_foot_contact);
  input.mutable_contact_info().push_back(right_foot_contact);

  return input;
}

void close_loop_test() {
  example::qp_inverse_dynamics::QPInput input = default_QP_input(robot);

  DiagramBuilder<double> builder;
  System2DummyValkyrieSim *val_sim = builder.AddSystem(std::make_unique<System2DummyValkyrieSim>(robot));
  System2QP *qp_con = builder.AddSystem(std::make_unique<System2QP>(robot));
  ConstantValueSource<double> *const_qp_input = builder.AddSystem(std::make_unique<ConstantValueSource<double>>(std::unique_ptr<AbstractValue>(new Value<example::qp_inverse_dynamics::QPInput>(input))));

  builder.Connect(qp_con->get_output_port(0), val_sim->get_input_port(0));
  builder.Connect(val_sim->get_output_port(0), qp_con->get_input_port(0));
  builder.Connect(const_qp_input->get_output_port(0), qp_con->get_input_port(1));

  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  // probably should pass in sub context?
  val_sim->set_initial_state(simulator.get_mutable_context());

  simulator.request_initial_step_size_attempt(1e-2);
  simulator.Initialize();
  simulator.StepTo(1.0);
}

int main() {
  //const_acc_test();

  close_loop_test();

  return 0;
}
