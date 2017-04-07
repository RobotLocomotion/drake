#include "drake/examples/kuka_iiwa_arm/robot_plan_interpolator.h"

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

using robotlocomotion::robot_plan_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// This corresponds to the actual plan.
constexpr int kAbsStateIdxPlan = 0;
// This corresponds to a flag that indicates whether the plan has been
// initialized properly by RobotPlanInterpolator::Initialize().
constexpr int kAbsStateIdxInitFlag = 1;

}  // namespace

constexpr double RobotPlanInterpolator::kDefaultPlanUpdateInterval;

// TODO(sam.creasey) If we had version of Trajectory which supported
// outputting the derivatives in value(), we could avoid keeping track
// of multiple polynomials below.
struct RobotPlanInterpolator::PlanData {
  PlanData() {}
  ~PlanData() {}

  double start_time{0};
  std::vector<char> encoded_msg;
  PiecewisePolynomial<double> pp;
  PiecewisePolynomial<double> pp_deriv;
  PiecewisePolynomial<double> pp_double_deriv;
};

RobotPlanInterpolator::RobotPlanInterpolator(
    const std::string& model_path, double update_interval)
    : plan_input_port_(this->DeclareAbstractInputPort().get_index()) {
  this->set_name("RobotPlanInterpolator");
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      model_path, multibody::joints::kFixed, &tree_);
  // TODO(sam.creasey) This implementation doesn't know how to
  // calculate velocities/accelerations for differing numbers of
  // positions and velocities.
  DRAKE_DEMAND(tree_.get_num_positions() == tree_.get_num_velocities());
  const int num_pv = tree_.get_num_positions() + tree_.get_num_velocities();

  state_input_port_ =
      this->DeclareVectorInputPort(systems::BasicVector<double>(num_pv))
          .get_index();
  state_output_port_ =
      this->DeclareVectorOutputPort(
              systems::BasicVector<double>(num_pv),
              &RobotPlanInterpolator::OutputState)
          .get_index();
  acceleration_output_port_ =
      this->DeclareVectorOutputPort(
              systems::BasicVector<double>(tree_.get_num_velocities()),
              &RobotPlanInterpolator::OutputAccel)
          .get_index();

  this->DeclarePeriodicUnrestrictedUpdate(update_interval, 0);
}

RobotPlanInterpolator::~RobotPlanInterpolator() {}

std::unique_ptr<systems::AbstractValues>
RobotPlanInterpolator::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
  const PlanData default_plan;
  // Actual plan.
  abstract_vals[kAbsStateIdxPlan] =
      systems::AbstractValue::Make<PlanData>(default_plan);
  // Flag indicating whether RobotPlanInterpolator::Initialize() has
  // been called.
  abstract_vals[kAbsStateIdxInitFlag] =
      systems::AbstractValue::Make<bool>(false);
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void RobotPlanInterpolator::SetDefaultState(
    const systems::Context<double>&,
    systems::State<double>* state) const {
  PlanData& plan =
      state->get_mutable_abstract_state<PlanData>(kAbsStateIdxPlan);
  plan = PlanData();
  state->get_mutable_abstract_state<bool>(kAbsStateIdxInitFlag) = false;
}

void RobotPlanInterpolator::OutputState(const systems::Context<double>& context,
                                  systems::BasicVector<double>* output) const {
  const PlanData& plan = context.get_abstract_state<PlanData>(kAbsStateIdxPlan);
  const bool inited = context.get_abstract_state<bool>(kAbsStateIdxInitFlag);
  DRAKE_DEMAND(inited);

  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();

  const double current_plan_time = context.get_time() - plan.start_time;
  output_vec.head(tree_.get_num_positions()) =
      plan.pp.value(current_plan_time);
  output_vec.tail(tree_.get_num_velocities()) =
      plan.pp_deriv.value(current_plan_time);
}

void RobotPlanInterpolator::OutputAccel(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const PlanData& plan = context.get_abstract_state<PlanData>(kAbsStateIdxPlan);
  const bool inited = context.get_abstract_state<bool>(kAbsStateIdxInitFlag);
  DRAKE_DEMAND(inited);

  Eigen::VectorBlock<VectorX<double>> output_acceleration_vec =
      output->get_mutable_value();

  const double current_plan_time = context.get_time() - plan.start_time;
  output_acceleration_vec = plan.pp_double_deriv.value(current_plan_time);
}

void RobotPlanInterpolator::MakeFixedPlan(
    double plan_start_time, const VectorX<double>& q0,
    systems::State<double>* state) const {
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(q0.size() == tree_.get_num_positions());
  PlanData& plan =
      state->get_mutable_abstract_state<PlanData>(kAbsStateIdxPlan);

  std::vector<Eigen::MatrixXd> knots(2, q0);
  std::vector<double> times{0., 1.};
  plan.start_time = plan_start_time;
  plan.pp = PiecewisePolynomial<double>::ZeroOrderHold(times, knots);
  plan.pp_deriv = plan.pp.derivative();
  plan.pp_double_deriv = plan.pp_deriv.derivative();
  drake::log()->info("Generated fixed plan at {}", q0.transpose());
}

void RobotPlanInterpolator::Initialize(double plan_start_time,
                                       const VectorX<double>& q0,
                                       systems::State<double>* state) const {
  DRAKE_DEMAND(state != nullptr);
  MakeFixedPlan(plan_start_time, q0, state);
  state->get_mutable_abstract_state<bool>(kAbsStateIdxInitFlag) = true;
}

void RobotPlanInterpolator::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  PlanData& plan =
      state->get_mutable_abstract_state<PlanData>(kAbsStateIdxPlan);
  const robot_plan_t& plan_input =
      this->EvalAbstractInput(context, plan_input_port_)
          ->GetValue<robot_plan_t>();

  // I (sam.creasey) wish I could think of a more effective way to
  // determine that a new message has arrived, but unfortunately
  // this is the best I've got.
  std::vector<char> encoded_msg(plan_input.getEncodedSize());
  plan_input.encode(encoded_msg.data(), 0, encoded_msg.size());
  if (encoded_msg != plan.encoded_msg) {
    plan.encoded_msg.swap(encoded_msg);
    if (plan_input.num_states) {
      plan.start_time = context.get_time();
      std::vector<Eigen::MatrixXd> knots(
          plan_input.num_states,
          Eigen::MatrixXd::Zero(tree_.get_num_positions(), 1));
      std::map<std::string, int> name_to_idx =
          tree_.computePositionNameToIndexMap();
      for (int i = 0; i < plan_input.num_states; ++i) {
        const auto& plan_state = plan_input.plan[i];
        for (int j = 0; j < plan_state.num_joints; ++j) {
          if (name_to_idx.count(plan_state.joint_name[j]) == 0) {
            continue;
          }
          knots[i](name_to_idx[plan_state.joint_name[j]], 0) =
              plan_state.joint_position[j];
        }
      }

      std::vector<double> input_time;
      for (int k = 0; k < static_cast<int>(plan_input.plan.size()); ++k) {
        input_time.push_back(plan_input.plan[k].utime / 1e6);
      }

      if (knots.size() >= 3) {
        const Eigen::MatrixXd knot_dot =
            Eigen::MatrixXd::Zero(tree_.get_num_velocities(), 1);
        plan.pp = PiecewisePolynomial<double>::Cubic(
            input_time, knots, knot_dot, knot_dot);
      } else {
        plan.pp = PiecewisePolynomial<double>::FirstOrderHold(
            input_time, knots);
      }
      plan.pp_deriv = plan.pp.derivative();
      plan.pp_double_deriv = plan.pp_deriv.derivative();
    } else {
      // The plan is empty.  Encode a plan for the current measured
      // position.
      const systems::BasicVector<double>* state_input =
          this->EvalVectorInput(context, state_input_port_);
      DRAKE_DEMAND(state_input);
      MakeFixedPlan(context.get_time(),
                    state_input->get_value().head(tree_.get_num_positions()),
                    state);
    }
  }
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
