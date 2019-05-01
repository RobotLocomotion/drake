
#include "drake/manipulation/robot_plan_runner/robot_plans.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace {

using manipulation::robot_plan_runner::PlanData;
using std::cout;
using std::endl;

class Controller : public systems::LeafSystem<double> {
 public:
  Controller() {
    this->set_name("controller");

    input_port_idx_plan_data_ =
        this->DeclareAbstractInputPort("plan_data", Value<PlanData>{})
            .get_index();

    this->DeclareVectorOutputPort("q_tau_cmd", systems::BasicVector<double>(1),
                                  &Controller::CalcOutput);
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

class PlanSender : public systems::LeafSystem<double> {
 public:
  PlanSender() {
    this->set_name("plan_sender");
    abstract_state_index_ = this->DeclareAbstractState(
        AbstractValue::Make<int>(1));

    this->DeclareAbstractOutputPort(
        "plan_data", &PlanSender::CalcJointSpacePlan,
        {abstract_state_ticket(abstract_state_index_)});
  }

  void DoCalcNextUpdateTime(const systems::Context<double>& context,
                            systems::CompositeEventCollection<double>* events,
                            double* time) const override {
    LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
    DRAKE_THROW_UNLESS(events->HasEvents() == false);
    DRAKE_THROW_UNLESS(std::isinf(*time));

    double t = context.get_time();
    cout << t << " DoCalcNextUpdateTime called" << endl;
    if (t >= 1 && !has_updated_) {
      *time = t;

      cout << "Adds event" << endl;
      has_updated_ = true;
      systems::EventCollection<systems::UnrestrictedUpdateEvent<double>>&
          uu_events = events->get_mutable_unrestricted_update_events();
      uu_events.add_event(
          std::make_unique<systems::UnrestrictedUpdateEvent<double>>(
              systems::TriggerType::kTimed));
    }
  }

 private:
  void CalcJointSpacePlan(const systems::Context<double>& context,
                          PlanData* output_plan_data) const {
    std::cout << "output plan copied!" << std::endl;
    PlanData new_plan;
    new_plan.plan_signature = has_updated_;
    *output_plan_data = new_plan;
  };
  mutable bool has_updated_{false};
  systems::AbstractStateIndex abstract_state_index_{-1};
};

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto plan_sender_sys = builder.AddSystem<PlanSender>();
  auto controller_sys = builder.AddSystem<Controller>();
  auto zoh_sys = builder.AddSystem<systems::ZeroOrderHold>(0.1, 1);

  builder.Connect(plan_sender_sys->GetOutputPort("plan_data"),
                  controller_sys->GetInputPort("plan_data"));
  builder.Connect(controller_sys->GetOutputPort("q_tau_cmd"),
                  zoh_sys->get_input_port());

  auto diagram = builder.Build();

  systems::UnrestrictedUpdateEvent<double> b(systems::TriggerType::kTimed);

  auto a = std::make_unique<systems::UnrestrictedUpdateEvent<double>>(
      systems::TriggerType::kTimed);
  systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(3.0);

  return 0;
};

}  // namespace
}  // namespace drake

int main() { return drake::do_main(); };