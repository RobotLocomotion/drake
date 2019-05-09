
#include "drake/manipulation/robot_plan_runner/plan_sender.h"
#include "drake/manipulation/robot_plan_runner/robot_plans.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using std::cout;
using std::endl;
using systems::Context;
using systems::State;
using trajectories::PiecewisePolynomial;

template <class T>
void PrintStlVector(std::vector<T> v) {
  for (const auto& vi : v) {
    cout << vi << " ";
  }
  cout << endl;
};

trajectories::PiecewisePolynomial<double>
ConnectTwoPositionsWithCubicPolynomial(
    const Eigen::Ref<const Eigen::VectorXd>& q_start,
    const Eigen::Ref<const Eigen::VectorXd>& q_end, double duration) {
  DRAKE_THROW_UNLESS(q_start.size() == q_end.size());
  Eigen::VectorXd t_knots(3);
  t_knots << 0, duration / 2, duration;
  const int num_positions = q_start.size();
  Eigen::MatrixXd q_knots(num_positions, 3);
  q_knots.col(0) << q_start;
  q_knots.col(1) << (q_start + q_end) / 2;
  q_knots.col(2) << q_end;

  return trajectories::PiecewisePolynomial<double>::Cubic(
      t_knots, q_knots, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
}

PlanSender::PlanSender(const std::vector<PlanData>& plan_data_list)
    : num_positions_(7),
      transition_time_sec_(4.),
      zoh_time_sec_(2.0),
      extra_time_(1.) {
  this->set_name("PlanSender");

  // Declare a state that does not get changed. It exists so that an abstract
  // dependency ticket can be declared.
  abstract_state_index_ =
      this->DeclareAbstractState(AbstractValue::Make<int>(1));

  // Input and output ports.
  this->DeclareAbstractOutputPort(
      "plan_data", &PlanSender::CalcPlan,
      {abstract_state_ticket(abstract_state_index_)});
  input_port_idx_q_ =
      this->DeclareInputPort("q", systems::kVectorValued, num_positions_)
          .get_index();

  // Declare Initialization.
  this->DeclareInitializationUnrestrictedUpdateEvent(&PlanSender::Initialize);

  plan_data_list_ = plan_data_list;
};

systems::EventStatus PlanSender::Initialize(const Context<double>& context,
                                            State<double>* state) const {
  cout << "initiliazation!" << endl;
  Eigen::VectorXd q_current =
      this->get_input_port(input_port_idx_q_).Eval(context);
  // Create a plan that connects the current position of the robot to the
  // position at the beginning of plan_data_list_. This requires the first
  // position in plan_data_list_ to have a non-empty joint_traj.
  const PlanData& plan0 = plan_data_list_[0];
  if (plan0.joint_traj.has_value()) {
    Eigen::VectorXd q0 = plan0.joint_traj.value().value(0);

    PiecewisePolynomial<double> joint_traj =
        ConnectTwoPositionsWithCubicPolynomial(q_current, q0,
                                               transition_time_sec_);
    PlanData first_plan;
    first_plan.joint_traj = joint_traj;
    first_plan.plan_type = PlanType::kJointSpacePlan;
    plan_data_list_.insert(plan_data_list_.begin(), first_plan);
  }

  // Create a 1 second zero order hold at the beginning of all plans.
  {
    PlanData zoh_plan;
    zoh_plan.plan_type = PlanType::kJointSpacePlan;
    Eigen::VectorXd t_knots(2);
    t_knots << 0, zoh_time_sec_;
    Eigen::MatrixXd q_knots(num_positions_, 2);
    q_knots.col(0) = q_current;
    q_knots.col(1) = q_current;
    zoh_plan.joint_traj =
        PiecewisePolynomial<double>::ZeroOrderHold(t_knots, q_knots);
    plan_data_list_.insert(plan_data_list_.begin(), zoh_plan);
  }

  // initialize plan list
  int i = 0;
  plan_start_times_.push_back(0);
  for (auto& plan_data : plan_data_list_) {
    double t_next = plan_data.joint_traj.value().end_time();
    if (i != 0) {
      // not the zoh plan
      t_next += extra_time_;
    }
    t_next += plan_start_times_[i];
    plan_start_times_.push_back(t_next);
    plan_data.plan_signature = i;
    i++;
  }
  num_plans_ = plan_data_list_.size();
  cout << "start times ";
  PrintStlVector(plan_start_times_);

  // set AbstractState (plan_index) to -1
  state->get_mutable_abstract_state().get_mutable_value(0).set_value<int>(-1);

  return systems::EventStatus::Succeeded();
};

void PlanSender::DoCalcNextUpdateTime(
    const systems::Context<double>& context,
    systems::CompositeEventCollection<double>* events, double* time) const {
  // call parent version and make sure that there're no other events to
  // handle.
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  double t = context.get_time();
  //  cout << t << " DoCalcNextUpdateTime called" << endl;
  if (current_plan_idx_ < num_plans_ - 1) {
    /*
     * DoCalcNextUpdateTime is called at t = -eps. That's when the first plan
     * should be scheduled.
     */
    if (t < 0 || t >= plan_start_times_[current_plan_idx_ + 1]) {
      cout << "Adds event at time " << t << endl;
      *time = t < 0 ? 0 : t;
      systems::EventCollection<systems::UnrestrictedUpdateEvent<double>>&
          uu_events = events->get_mutable_unrestricted_update_events();
      uu_events.add_event(
          std::make_unique<systems::UnrestrictedUpdateEvent<double>>(
              systems::TriggerType::kTimed,
              [this](const Context<double>& context,
                     const systems::UnrestrictedUpdateEvent<double>&,
                     State<double>* x) {
                this->UpdatePlanIndex(context, &*x);
              }));

      // update "unregistered" states
      current_plan_idx_++;
    }
  }
}

double PlanSender::get_all_plans_duration() const {
  // zoh + transition
  double t_total = zoh_time_sec_ + transition_time_sec_ + extra_time_;
  for (const auto& plan_data : plan_data_list_) {
    // TODO: extend this for ee_traj as well
    if (plan_data.joint_traj.has_value()) {
      t_total += plan_data.joint_traj.value().end_time() + extra_time_;
    }
  }
  return t_total;
}

void PlanSender::CalcPlan(const drake::systems::Context<double>& context,
                          PlanData* output_plan_data) const {
  cout << "Plan " << current_plan_idx_
       << " has started at t=" << context.get_time() << endl;
  *output_plan_data = plan_data_list_[current_plan_idx_];
};

void PlanSender::UpdatePlanIndex(const systems::Context<double>& context,
                                 systems::State<double>* state) const {
  int& state_value = state->get_mutable_abstract_state()
                         .get_mutable_value(0)
                         .get_mutable_value<int>();
  state_value++;
  cout << "current_plan_idx " << current_plan_idx_ << endl;
  cout << "abstract state " << state_value << endl;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake