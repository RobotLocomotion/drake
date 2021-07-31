#include "drake/manipulation/planner/robot_plan_interpolator.h"

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace manipulation {
namespace planner {

using multibody::BodyIndex;
using multibody::JointIndex;
using trajectories::PiecewisePolynomial;

// TODO(sammy-tri) If we had version of Trajectory which supported
// outputting the derivatives in value(), we could avoid keeping track
// of multiple polynomials below.
struct RobotPlanInterpolator::PlanData {
  PlanData() {}

  double start_time{0};
  std::vector<char> encoded_msg;
  PiecewisePolynomial<double> pp;
  PiecewisePolynomial<double> pp_deriv;
  PiecewisePolynomial<double> pp_double_deriv;
};

RobotPlanInterpolator::RobotPlanInterpolator(
    const std::string& model_path, const InterpolatorType interp_type,
    double update_interval)
    : plan_input_port_(this->DeclareAbstractInputPort(
          "plan", Value<lcmt_robot_plan>()).get_index()),
      interp_type_(interp_type) {
  multibody::Parser(&plant_).AddModelFromFile(model_path);

  // Search for any bodies with no parent.  We'll weld those to the world.
  std::set<BodyIndex> parent_bodies;
  std::set<BodyIndex> child_bodies;
  for (JointIndex i(0); i < plant_.num_joints(); ++i) {
    const multibody::Joint<double>& joint = plant_.get_joint(i);
    if (joint.parent_body().index() == plant_.world_body().index()) {
      // Nothing to weld, we're connected to the world.
      parent_bodies.clear();
      break;
    }
    parent_bodies.insert(joint.parent_body().index());
    child_bodies.insert(joint.child_body().index());
  }

  if (!parent_bodies.empty()) {
    for (const BodyIndex& child : child_bodies) {
      if (parent_bodies.count(child)) {
        parent_bodies.erase(child);
      }
    }

    // Weld all remaining parents to the world.  This probably isn't going to
    // work for all model types.
    for (const BodyIndex& index : parent_bodies) {
      plant_.WeldFrames(plant_.world_frame(),
                        plant_.get_body(index).body_frame());
    }
  }
  plant_.Finalize();

  // TODO(sammy-tri) This implementation doesn't know how to
  // calculate velocities/accelerations for differing numbers of
  // positions and velocities.
  DRAKE_DEMAND(plant_.num_positions() == plant_.num_velocities());
  const int num_pv = plant_.num_positions() + plant_.num_velocities();

  state_output_port_ =
      this->DeclareVectorOutputPort("state", num_pv,
                                    &RobotPlanInterpolator::OutputState)
          .get_index();
  acceleration_output_port_ =
      this->DeclareVectorOutputPort("acceleration", plant_.num_velocities(),
                                    &RobotPlanInterpolator::OutputAccel)
          .get_index();

  // This corresponds to the actual plan.
  plan_index_ = this->DeclareAbstractState(Value<PlanData>());

  // Flag indicating whether RobotPlanInterpolator::Initialize has been called.
  init_flag_index_ = this->DeclareAbstractState(Value<bool>(false));

  this->DeclarePeriodicUnrestrictedUpdate(update_interval, 0);
}

RobotPlanInterpolator::~RobotPlanInterpolator() {}

void RobotPlanInterpolator::SetDefaultState(
    const systems::Context<double>&,
    systems::State<double>* state) const {
  PlanData& plan =
      state->get_mutable_abstract_state<PlanData>(plan_index_);
  plan = PlanData();
  state->get_mutable_abstract_state<bool>(init_flag_index_) = false;
}

void RobotPlanInterpolator::OutputState(const systems::Context<double>& context,
                                  systems::BasicVector<double>* output) const {
  const PlanData& plan = context.get_abstract_state<PlanData>(plan_index_);
  const bool inited = context.get_abstract_state<bool>(init_flag_index_);
  DRAKE_DEMAND(inited);

  Eigen::VectorBlock<VectorX<double>> output_vec =
      output->get_mutable_value();

  const double current_plan_time = context.get_time() - plan.start_time;
  output_vec.head(plant_.num_positions()) =
      plan.pp.value(current_plan_time);
  output_vec.tail(plant_.num_velocities()) =
      plan.pp_deriv.value(current_plan_time);
}

void RobotPlanInterpolator::OutputAccel(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const PlanData& plan = context.get_abstract_state<PlanData>(plan_index_);
  const bool inited = context.get_abstract_state<bool>(init_flag_index_);
  DRAKE_DEMAND(inited);

  Eigen::VectorBlock<VectorX<double>> output_acceleration_vec =
      output->get_mutable_value();

  const double current_plan_time = context.get_time() - plan.start_time;
  output_acceleration_vec = plan.pp_double_deriv.value(current_plan_time);

  // Stop outputting accelerations at the end of the plan.
  if (current_plan_time > plan.pp_double_deriv.end_time()) {
    output_acceleration_vec.fill(0);
  }
}

void RobotPlanInterpolator::MakeFixedPlan(
    double plan_start_time, const VectorX<double>& q0,
    systems::State<double>* state) const {
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(q0.size() == plant_.num_positions());
  PlanData& plan =
      state->get_mutable_abstract_state<PlanData>(plan_index_);

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
  state->get_mutable_abstract_state<bool>(init_flag_index_) = true;
}

void RobotPlanInterpolator::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  PlanData& plan =
      state->get_mutable_abstract_state<PlanData>(plan_index_);
  const lcmt_robot_plan& plan_input =
      get_plan_input_port().Eval<lcmt_robot_plan>(context);

  // I (sammy-tri) wish I could think of a more effective way to
  // determine that a new message has arrived, but unfortunately
  // this is the best I've got.
  std::vector<char> encoded_msg(plan_input.getEncodedSize());
  plan_input.encode(encoded_msg.data(), 0, encoded_msg.size());
  if (encoded_msg != plan.encoded_msg) {
    plan.encoded_msg.swap(encoded_msg);
    if (plan_input.num_states == 0) {
      // The plan is empty.  Encode a plan for the current planned position.
      const double current_plan_time = context.get_time() - plan.start_time;
      MakeFixedPlan(context.get_time(),
                    plan.pp.value(current_plan_time),
                    state);
    } else if (plan_input.num_states == 1) {
      drake::log()->info("Ignoring plan with only one knot point.");
    } else {
      plan.start_time = context.get_time();
      std::vector<Eigen::MatrixXd> knots(
          plan_input.num_states,
          Eigen::MatrixXd::Zero(plant_.num_positions(), 1));
      for (int i = 0; i < plan_input.num_states; ++i) {
        const auto& plan_state = plan_input.plan[i];
        for (int j = 0; j < plan_state.num_joints; ++j) {
          if (!plant_.HasJointNamed(plan_state.joint_name[j])) {
            continue;
          }
          const auto joint_index = plant_.GetJointByName(
              plan_state.joint_name[j]).position_start();
          knots[i](joint_index, 0) = plan_state.joint_position[j];
        }
      }

      std::vector<double> input_time;
      for (int k = 0; k < static_cast<int>(plan_input.plan.size()); ++k) {
        input_time.push_back(plan_input.plan[k].utime / 1e6);
      }

      const Eigen::MatrixXd knot_dot =
          Eigen::MatrixXd::Zero(plant_.num_velocities(), 1);
      switch (interp_type_) {
        case InterpolatorType::ZeroOrderHold :
          plan.pp = PiecewisePolynomial<double>::ZeroOrderHold(
              input_time, knots);
          break;
        case InterpolatorType::FirstOrderHold :
          plan.pp = PiecewisePolynomial<double>::FirstOrderHold(
              input_time, knots);
          break;
        case InterpolatorType::Pchip :
          plan.pp = PiecewisePolynomial<double>::CubicShapePreserving(
              input_time, knots, true);
          break;
        case InterpolatorType::Cubic :
          plan.pp =
              PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
                  input_time, knots, knot_dot, knot_dot);
          break;
      }
      plan.pp_deriv = plan.pp.derivative();
      plan.pp_double_deriv = plan.pp_deriv.derivative();
    }
  }
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
