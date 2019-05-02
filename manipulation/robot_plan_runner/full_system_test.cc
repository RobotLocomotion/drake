#include <fstream>

#include "drake/manipulation/robot_plan_runner/robot_plans.h"
#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"
#include "drake/manipulation/robot_plan_runner/plan_sender.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace {

using manipulation::robot_plan_runner::PlanData;
using manipulation::robot_plan_runner::PlanSender;
using manipulation::robot_plan_runner::RobotPlanRunner;
using std::cout;
using std::endl;

int do_main() {
  systems::DiagramBuilder<double> builder;

  PlanData foo;
  std::vector<PlanData> plan_list{foo};
  auto plan_sender = builder.AddSystem<PlanSender>(plan_list);
  auto plan_runner = builder.AddSystem<RobotPlanRunner>();

  builder.Connect(plan_sender->GetOutputPort("plan_data"),
                  plan_runner->GetInputPort("plan_data"));

  auto diagram = builder.Build();
  std::string a = diagram->GetGraphvizString();

  std::ofstream out("system_graphviz_string.txt");
  out << a;
  out.close();

//  systems::Simulator<double> simulator(*diagram);
//
//  simulator.set_publish_every_time_step(false);
//  simulator.set_target_realtime_rate(1.0);
//  simulator.AdvanceTo(3.0);

  return 0;
};

}  // namespace
}  // namespace drake

int main() { return drake::do_main(); };