#include "drake/examples/kuka_iiwa_arm/iiwa_plan_source.h"

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

using robotlocomotion::robot_plan_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const int kNumJoints = 7;
const double kPlanUpdateInterval = 0.1;

}  // namespace

// TODO(sam.creasey) If we had version of Trajectory which supported
// outputting the derivatives in value(), we could avoid keeping track
// of multiple polynomials below.
struct IiwaPlanSource::PlanData {
  PlanData() {}
  ~PlanData() {}

  double start_time{0};
  std::vector<char> encoded_msg;
  PiecewisePolynomial<double> pp;
  PiecewisePolynomial<double> pp_deriv;
};

IiwaPlanSource::IiwaPlanSource(const std::string& model_path)
    : plan_input_port_(this->DeclareAbstractInputPort().get_index()),
      status_input_port_(this->DeclareAbstractInputPort().get_index()) {
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      model_path, multibody::joints::kFixed, &tree_);

  this->DeclareOutputPort(systems::kVectorValued, kNumJoints * 2);
  this->DeclarePeriodicUnrestrictedUpdate(kPlanUpdateInterval, 0);
}

IiwaPlanSource::~IiwaPlanSource() {}

std::unique_ptr<systems::AbstractValues> IiwaPlanSource::AllocateAbstractState()
    const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  const PlanData default_plan;
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      std::make_unique<systems::Value<PlanData>>(default_plan)));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void IiwaPlanSource::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  const PlanData& plan = context.get_abstract_state<PlanData>(0);
  const lcmt_iiwa_status& status =
      this->EvalAbstractInput(context, status_input_port_)
      ->GetValue<lcmt_iiwa_status>();

  if (!status.utime) {
    throw std::runtime_error(
        "iiwa status not known when attempting to "
        "calculate plan output");
  }

  Eigen::VectorBlock<VectorX<double>> output_vec =
      this->GetMutableOutputVector(output, 0);
  if (!plan.start_time) {
    // We don't have a plan yet, so emit the last commanded position
    // with no velocity.
    for (int i = 0; i < kNumJoints; i++) {
      output_vec(i) = status.joint_position_measured[i];
    }
    output_vec.tail(kNumJoints) = Eigen::VectorXd::Zero(kNumJoints);
  } else {
    output_vec.head(kNumJoints) =
        plan.pp.value(context.get_time() - plan.start_time);
    output_vec.tail(kNumJoints) =
        plan.pp_deriv.value(context.get_time() - plan.start_time);
  }
}

void IiwaPlanSource::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  const lcmt_iiwa_status& status =
      this->EvalAbstractInput(context, status_input_port_)
      ->GetValue<lcmt_iiwa_status>();

  // Generate a default plan to hold at the measured position if we
  // don't have another plan yet.
  PlanData& plan = state->get_mutable_abstract_state<PlanData>(0);
  if (status.utime && !plan.start_time) {
    Eigen::MatrixXd measured(kNumJoints, 1);
    for (int i = 0; i < kNumJoints; i++) {
      measured(i) = status.joint_position_measured[i];
    }

    std::vector<Eigen::MatrixXd> knots(2, measured);
    std::vector<double> times{0., 1.};
    plan.start_time = context.get_time();
    plan.pp = PiecewisePolynomial<double>::ZeroOrderHold(times, knots);
    plan.pp_deriv = plan.pp.derivative();
    drake::log()->info("Generated fixed plan at {}", measured.transpose());
  }

  const robot_plan_t& plan_input =
      this->EvalAbstractInput(context, plan_input_port_)
      ->GetValue<robot_plan_t>();

  // I (sam.creasey) wish I could think of a more effective way to
  // determine that a new message has arrived, but unfortunately
  // this is the best I've got.
  std::vector<char> encoded_msg(plan_input.getEncodedSize());
  plan_input.encode(encoded_msg.data(), 0, encoded_msg.size());
  if (plan_input.num_states && encoded_msg != plan.encoded_msg) {
    if (!status.utime) {
      throw std::runtime_error("Plan received before status");
    }

    plan.encoded_msg.swap(encoded_msg);
    plan.start_time = context.get_time();
    std::vector<Eigen::MatrixXd> knots(plan_input.num_states,
                                       Eigen::MatrixXd::Zero(kNumJoints, 1));
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
      const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints, 1);
      plan.pp = PiecewisePolynomial<double>::Cubic(input_time, knots,
                                                   knot_dot, knot_dot);
    } else {
      plan.pp =
          PiecewisePolynomial<double>::FirstOrderHold(input_time, knots);
    }
    plan.pp_deriv = plan.pp.derivative();
  }
}


}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
