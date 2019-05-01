
#include "drake/manipulation/robot_plan_runner/plan_sender.h"
#include "drake/manipulation/robot_plan_runner/robot_plans.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using std::cout;
using std::endl;
using trajectories::PiecewisePolynomial;
using systems::Context;
using systems::State;

template<class T>
void PrintStlVector(std::vector<T> v) {
  for(const auto& vi : v) {
    cout << vi << " ";
  }
  cout << endl;
};


PlanSender::PlanSender(const std::vector<PlanData>& plan_data_list) {
  this->set_name("PlanSender");

  // Declare a state that does not get changed. It exists so that an abstract
  // dependency ticket can be declared.
  abstract_state_index_ =
      this->DeclareAbstractState(AbstractValue::Make<int>(1));

  this->DeclareAbstractOutputPort(
      "plan_data", &PlanSender::CalcPlan,
      {abstract_state_ticket(abstract_state_index_)});

  this->DeclareInitializationUnrestrictedUpdateEvent(&PlanSender::Initialize);

  plan_data_list_ = plan_data_list;
};


systems::EventStatus PlanSender::Initialize(const Context<double> & context,
                                State<double> *state ) const {
  cout << "initiliazation!" << endl;
  // initialize plan list
  int i = 0;
  plan_start_times_.push_back(0);
  for (auto& plan_data : plan_data_list_) {
    double t_next = plan_data.joint_traj.value().end_time();
    t_next += plan_start_times_[i];
    plan_start_times_.push_back(t_next);
    plan_data.plan_signature = i;
    i++;
  }
  num_plans_ = plan_data_list_.size();
  cout << "start times ";
  PrintStlVector(plan_start_times_);

  return systems::EventStatus::Succeeded();
};

void PlanSender::DoCalcNextUpdateTime(const systems::Context<double>& context,
                          systems::CompositeEventCollection<double>* events,
                          double* time) const {
// call parent version and make sure that there're no other events to
// handle.
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  double t = context.get_time();
  cout << t << " DoCalcNextUpdateTime called" << endl;
  if (current_plan_idx_ < num_plans_ - 1) {
/*
 * DoCalcNextUpdateTime is called at t = -eps. That's when the first plan
 * should be scheduled.
 */
    if (t < 0 || t >= plan_start_times_[current_plan_idx_ + 1]) {
      cout << "Adds event at time " << t << endl;
      *time = t;
      systems::EventCollection<systems::UnrestrictedUpdateEvent<double>> &
          uu_events = events->get_mutable_unrestricted_update_events();
      uu_events.add_event(
          std::make_unique<systems::UnrestrictedUpdateEvent<double>>(
              systems::TriggerType::kTimed));

// update "unregistered" states
      current_plan_idx_++;
    }
  }
}

void PlanSender::CalcPlan(const drake::systems::Context<double>& context,
              PlanData* output_plan_data) const {
  cout << "Plan " << current_plan_idx_
       << " has started at t=" << context.get_time() << endl;
  *output_plan_data = plan_data_list_[current_plan_idx_];
};



}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake