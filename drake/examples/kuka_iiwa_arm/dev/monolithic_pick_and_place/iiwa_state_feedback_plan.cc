#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/iiwa_state_feedback_plan.h"

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
#include "drake/systems/framework/basic_vector.h"

using robotlocomotion::robot_plan_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {
const int kNumJoints = 7;
}  // namespace

// TODO(sam.creasey) If we had version of Trajectory which supported
// outputting the derivatives in value(), we could avoid keeping track
// of multiple polynomials below.
struct IiwaStateFeedbackPlanSource::InternalData {
  InternalData() { iiwa_state = Eigen::VectorXd::Zero(2 * kNumJoints); }
  ~InternalData() {}

  Eigen::VectorXd iiwa_state;
  double start_time{0};
  std::vector<char> encoded_msg;
  PiecewisePolynomial<double> pp;
  PiecewisePolynomial<double> pp_deriv;
  PiecewisePolynomial<double> pp_double_deriv;
};

IiwaStateFeedbackPlanSource::IiwaStateFeedbackPlanSource(
    const std::string& model_path, const double update_interval)
    : input_port_plan_(this->DeclareAbstractInputPort().get_index()),
      input_port_state_(
          this->DeclareInputPort(systems::kVectorValued, 2 * kNumJoints)
              .get_index()),
      output_port_state_trajectory_(
          this->DeclareOutputPort(systems::kVectorValued, kNumJoints * 2)
              .get_index()),
      output_port_acceleration_trajectory_(
          this->DeclareOutputPort(systems::kVectorValued, kNumJoints)
              .get_index()) {
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      model_path, multibody::joints::kFixed, &tree_);
  this->DeclarePeriodicUnrestrictedUpdate(update_interval, 0);
}

IiwaStateFeedbackPlanSource::~IiwaStateFeedbackPlanSource() {}

std::unique_ptr<systems::AbstractValues>
IiwaStateFeedbackPlanSource::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  const InternalData default_plan;
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      std::make_unique<systems::Value<InternalData>>(default_plan)));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void IiwaStateFeedbackPlanSource::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  const InternalData& plan = context.get_abstract_state<InternalData>(0);

  Eigen::VectorBlock<VectorX<double>> output_state_vector =
      this->GetMutableOutputVector(output, output_port_state_trajectory_);

  Eigen::VectorBlock<VectorX<double>> output_acceleration_vector =
      this->GetMutableOutputVector(output,
                                   output_port_acceleration_trajectory_);

  if (plan.pp.getNumberOfSegments() <= 1) {
    // We don't have a plan yet, so emit the last commanded position
    // with no velocity.
    for (int i = 0; i < kNumJoints; i++) {
      output_state_vector(i) = plan.iiwa_state[i];
    }
    output_state_vector.tail(kNumJoints) = Eigen::VectorXd::Zero(kNumJoints);
    output_acceleration_vector = Eigen::VectorXd::Zero(kNumJoints);
  } else {
    output_state_vector.head(kNumJoints) =
        plan.pp.value(context.get_time() - plan.start_time);
    output_state_vector.tail(kNumJoints) =
        plan.pp_deriv.value(context.get_time() - plan.start_time);
    output_acceleration_vector =
        plan.pp_double_deriv.value(context.get_time() - plan.start_time);
  }
}

void IiwaStateFeedbackPlanSource::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  const systems::BasicVector<double>* input =
      this->EvalVectorInput(context, input_port_state_);
  DRAKE_DEMAND(input);

  const auto& iiwa_state = input->get_value();

  // Generate a default current_plan_in_state to hold at the measured position
  // if we don't have another current_plan_in_state yet.
  InternalData& current_plan_in_state =
      state->get_mutable_abstract_state<InternalData>(0);

  static robot_plan_t last_plan{};

  const robot_plan_t& plan_in_input =
      this->EvalAbstractInput(context, input_port_plan_)
          ->GetValue<robot_plan_t>();

  Eigen::MatrixXd measured(kNumJoints, 1);
  for (int i = 0; i < kNumJoints; i++) {
    measured(i) = iiwa_state[i];
  }

  std::vector<char> encoded_msg(plan_in_input.getEncodedSize());
  plan_in_input.encode(encoded_msg.data(), 0, encoded_msg.size());

  // Check if plan in input is valid
  if (plan_in_input.num_states) {
    // Check if plan is different from the currently executing / executed plan.
    if (encoded_msg != current_plan_in_state.encoded_msg) {
      current_plan_in_state.encoded_msg.swap(encoded_msg);
      current_plan_in_state.start_time = context.get_time();

      std::vector<Eigen::MatrixXd> knots(plan_in_input.num_states,
                                         Eigen::MatrixXd::Zero(kNumJoints, 1));
      std::map<std::string, int> name_to_idx =
          tree_.computePositionNameToIndexMap();
      for (int i = 0; i < plan_in_input.num_states; ++i) {
        const auto& plan_state = plan_in_input.plan[i];
        for (int j = 0; j < plan_state.num_joints; ++j) {
          if (name_to_idx.count(plan_state.joint_name[j]) == 0) {
            continue;
          }
          knots[i](name_to_idx[plan_state.joint_name[j]], 0) =
              plan_state.joint_position[j];
        }
      }
      std::vector<double> input_time;
      for (int k = 0; k < static_cast<int>(plan_in_input.plan.size()); ++k) {
        input_time.push_back(plan_in_input.plan[k].utime / 1e6);
      }

      if (knots.size() >= 3) {
        const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints, 1);
        current_plan_in_state.pp = PiecewisePolynomial<double>::Cubic(
            input_time, knots, knot_dot, knot_dot);
      } else {
        current_plan_in_state.pp =
            PiecewisePolynomial<double>::FirstOrderHold(input_time, knots);
      }
      current_plan_in_state.pp_deriv = current_plan_in_state.pp.derivative();
      current_plan_in_state.pp_double_deriv =
          current_plan_in_state.pp_deriv.derivative();
    }
  } else {
    std::vector<Eigen::MatrixXd> knots(2, measured);
    std::vector<double> times{0., 1.};
    current_plan_in_state.start_time = context.get_time();
    current_plan_in_state.pp =
        PiecewisePolynomial<double>::ZeroOrderHold(times, knots);
    current_plan_in_state.pp_deriv = current_plan_in_state.pp.derivative();
    current_plan_in_state.pp_double_deriv =
        current_plan_in_state.pp_deriv.derivative();
  }
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
