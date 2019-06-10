/*
 * Pieces to be converted into unit tests.
 */
#include <vector>

#include "drake/manipulation/robot_plan_runner/plan_sender.h"
#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/joint_space_plan.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using std::cout;
using std::endl;
using trajectories::PiecewisePolynomial;
using robot_plans::PlanData;
using robot_plans::PlanType;

class SimpleController : public systems::LeafSystem<double> {
 public:
  SimpleController() {
    this->set_name("controller");

    input_port_idx_plan_data_ =
        this->DeclareAbstractInputPort("plan_data", Value<PlanData>{})
            .get_index();

    this->DeclareVectorOutputPort("q_tau_cmd", systems::BasicVector<double>(1),
                                  &SimpleController::CalcOutput);
  }

 private:
  int input_port_idx_plan_data_;

  void CalcOutput(const systems::Context<double>& context,
                  systems::BasicVector<double>* output) const {
    const AbstractValue* plan_data_ptr =
        this->EvalAbstractInput(context, input_port_idx_plan_data_);
    int a = plan_data_ptr->get_value<PlanData>().plan_signature;
    output->SetAtIndex(0, a);
    std::cout << context.get_time() << " " << a << std::endl;
  }
};

int do_main() {
  robot_plans::JointSpacePlan plan;

  Eigen::VectorXd t_knots(3);
  t_knots << 0, 1, 2;

  Eigen::MatrixXd q_knots(7, 3);
  q_knots.col(0) << 0, 0, 0, 0, 0, 0, 0;
  q_knots.col(1) << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
  q_knots.col(2) << 1, 1, 1, 1, 1, 1, 1;

  auto qtraj = PiecewisePolynomial<double>::Cubic(
      t_knots, q_knots, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));

  PlanData plan1;
  plan1.plan_type = PlanType::kJointSpacePlan;
  plan1.joint_traj = qtraj;

  Eigen::VectorXd q(7);
  Eigen::VectorXd v(7);
  Eigen::VectorXd tau_external(7);
  Eigen::VectorXd q_cmd(7);
  Eigen::VectorXd tau_cmd(7);

  std::vector<double> t_list = {0, 0.5, 1, 1.5, 2, 2.5};

  const double control_period = 0.005;
  for (auto t : t_list) {
    plan.Step(q, v, tau_external, control_period, t, plan1, &q_cmd, &tau_cmd);
    cout << t << endl << q_cmd << endl << tau_cmd << endl;
  }

  // build diagram system.
  std::vector<PlanData> plan_list{plan1};
  systems::DiagramBuilder<double> builder;
  auto plan_sender_sys = builder.AddSystem<PlanSender>(plan_list);
  auto controller_sys = builder.AddSystem<SimpleController>();
  auto zoh_sys = builder.AddSystem<systems::ZeroOrderHold>(0.1, 1);

  builder.Connect(plan_sender_sys->GetOutputPort("plan_data"),
                  controller_sys->GetInputPort("plan_data"));
  builder.Connect(controller_sys->GetOutputPort("q_tau_cmd"),
                  zoh_sys->get_input_port());

  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  systems::Context<double>& plan_sender_context =
      diagram->GetMutableSubsystemContext(*plan_sender_sys,
                                          diagram_context.get());
  plan_sender_context.FixInputPort(
      0, systems::BasicVector<double>(Eigen::VectorXd::Zero(7)));
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);

  cout << "simulation starts from here." << endl;
  simulator.AdvanceTo(6.0);

  return 0;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake

int main() { return drake::manipulation::robot_plan_runner::do_main(); };