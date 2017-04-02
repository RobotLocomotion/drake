#include "drake/systems/trajectory_optimization/direct_trajectory_optimization.h"

#include <limits>
#include <stdexcept>

#include "drake/common/symbolic_expression.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace {
VectorXd VectorDiff(const VectorXd& vec) {
  DRAKE_ASSERT(vec.size() > 1);
  const int len_minus1 = vec.size() - 1;
  return vec.tail(len_minus1) - vec.head(len_minus1);
}
}  // namespace

// DirectTrajectoryOptimization
// For readability of long lines, these single-letter variables names are
// sometimes used:
// N number of timesteps/samples
// h timesteps (there are N-1 of these)
// x state
// u control input
DirectTrajectoryOptimization::DirectTrajectoryOptimization(
    const int num_inputs, const int num_states, const int num_time_samples,
    const double trajectory_time_lower_bound,
    const double trajectory_time_upper_bound)
    : num_inputs_(num_inputs),
      num_states_(num_states),
      N_(num_time_samples),
      h_vars_(NewContinuousVariables(N_ - 1, "h")),
      x_vars_(NewContinuousVariables(num_states_ * N_, "x")),
      u_vars_(NewContinuousVariables(num_inputs_ * N_, "u")),
      placeholder_t_var_(NewContinuousVariables<1>("h")),
      placeholder_x_vars_(NewContinuousVariables(num_states_, "x")),
      placeholder_u_vars_(NewContinuousVariables(num_inputs_, "u")) {
  DRAKE_ASSERT(num_time_samples > 1);
  DRAKE_ASSERT(num_states_ > 0);
  DRAKE_ASSERT(num_inputs_ > 0);
  DRAKE_ASSERT(trajectory_time_lower_bound <= trajectory_time_upper_bound);
  // Construct total time linear constraint.
  // TODO(Lucy-tri) add case for all timesteps independent (if needed).
  AddLinearConstraint(h_vars_.cast<symbolic::Expression>().sum(),
                      trajectory_time_lower_bound, trajectory_time_upper_bound);
  for (int i = 0; i < N_ - 2; ++i) {
    AddLinearEqualityConstraint(h_vars_(i + 1) - h_vars_(i), 0.0);
  }
  // // Ensure that all h values are non-negative.
  AddLinearConstraint(h_vars_.array() >= 0.0);
}

void DirectTrajectoryOptimization::AddInputBounds(
    const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound) {
  DRAKE_ASSERT(lower_bound.size() == num_inputs_);
  DRAKE_ASSERT(upper_bound.size() == num_inputs_);

  Eigen::VectorXd lb_all(num_inputs_ * N_);
  Eigen::VectorXd ub_all(num_inputs_ * N_);
  for (int i = 0; i < N_; i++) {
    lb_all.segment(num_inputs_ * i, num_inputs_) = lower_bound;
    ub_all.segment(num_inputs_ * i, num_inputs_) = upper_bound;
  }
  AddBoundingBoxConstraint(lb_all, ub_all, u_vars_);
}

void DirectTrajectoryOptimization::AddTimeIntervalBounds(
    const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound) {
  AddBoundingBoxConstraint(lower_bound, upper_bound, h_vars_);
}

void DirectTrajectoryOptimization::AddTimeIntervalBounds(
    const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound,
    const std::vector<int>& interval_indices) {
  solvers::VectorXDecisionVariable h(interval_indices.size());
  for (int i = 0; i < static_cast<int>(interval_indices.size()); ++i) {
    h(i) = h_vars_(interval_indices[i]);
  }
  AddBoundingBoxConstraint(lower_bound, upper_bound, h);
}

namespace {
/// Since the final cost evaluation needs a total time, we need a
/// wrapper which will calculate the total time from the individual
/// time steps and mangle the output appropriately.
class FinalCostWrapper : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FinalCostWrapper)

  FinalCostWrapper(int num_time_samples, int num_states,
                   std::shared_ptr<Constraint> constraint)
      : Constraint(constraint->num_constraints(),
                   (num_time_samples - 1) + num_states,
                   constraint->lower_bound(), constraint->upper_bound()),
        num_time_samples_(num_time_samples),
        num_states_(num_states),
        constraint_(constraint) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    // TODO(sam.creasey) If we actually need this, we could cut and
    // paste most of the implementation below (or maybe delegate to a
    // templated version).  I don't expect that scenario to occur.
    throw std::runtime_error("Non-Taylor constraint eval not implemented.");
  }

  void DoEval(const Eigen::Ref<const TaylorVecXd>& x,
              TaylorVecXd& y) const override {
    DRAKE_ASSERT(x.rows() == (num_time_samples_ - 1) + num_states_);

    TaylorVecXd wrapped_x(num_states_ + 1);
    wrapped_x(0) = x.head(num_time_samples_ - 1).sum();
    wrapped_x.tail(num_states_) = x.tail(num_states_);
    DRAKE_ASSERT(wrapped_x(0).derivatives().rows() ==
                 x(0).derivatives().rows());

    constraint_->Eval(wrapped_x, y);
    DRAKE_ASSERT(y(0).derivatives().rows() == x(0).derivatives().rows());
  };

 private:
  const int num_time_samples_;
  const int num_states_;
  std::shared_ptr<Constraint> constraint_;
};

}  // namespace

// We just use a generic constraint here since we need to mangle the
// input and output anyway.
void DirectTrajectoryOptimization::AddFinalCost(
    std::shared_ptr<solvers::Constraint> constraint) {
  auto wrapper =
      std::make_shared<FinalCostWrapper>(N_, num_states_, constraint);
  AddCost(wrapper, {h_vars_, x_vars_.tail(num_states_)});
}

void DirectTrajectoryOptimization::GetInitialVars(
    double timespan_init_in, const PiecewisePolynomial<double>& traj_init_u,
    const PiecewisePolynomial<double>& traj_init_x) {
  VectorXd timespan_init{VectorXd::LinSpaced(N_, 0, timespan_init_in)};
  SetInitialGuess(h_vars_, VectorDiff(timespan_init));

  VectorXd guess_u(u_vars_.size());
  if (traj_init_u.empty()) {
    guess_u.fill(0.003);  // Start with some small number <= 0.01.
  } else {
    for (int t = 0; t < N_; ++t) {
      guess_u.segment(num_inputs_ * t, num_inputs_) =
          traj_init_u.value(timespan_init[t]);
    }
  }
  SetInitialGuess(u_vars_, guess_u);

  DRAKE_ASSERT(!traj_init_x.empty());  // TODO(Lucy-tri) see below.
  VectorXd guess_x(x_vars_.size());
  if (traj_init_x.empty()) {
    guess_x.fill(0.003);  // Start with some small number <= 0.01.
    // TODO(Lucy-tri) Do what DirectTrajectoryOptimization.m does.
  } else {
    for (int t = 0; t < N_; ++t) {
      guess_x.segment(num_states_ * t, num_states_) =
          traj_init_x.value(timespan_init[t]);
    }
  }
  SetInitialGuess(x_vars_, guess_x);
}

solvers::SolutionResult DirectTrajectoryOptimization::SolveTraj(
    double timespan_init, const PiecewisePolynomial<double>& traj_init_u,
    const PiecewisePolynomial<double>& traj_init_x) {
  GetInitialVars(timespan_init, traj_init_u, traj_init_x);

  // If we're using IPOPT, it can't quite solve trajectories to the
  // default precision level.
  SetSolverOption(drake::solvers::SolverType::kIpopt, "tol", 1e-7);

  solvers::SolutionResult result = Solve();
  return result;
}

std::vector<double> DirectTrajectoryOptimization::GetTimeVector() const {
  std::vector<double> times;
  times.resize(N_, 0);

  const auto h_values = GetSolution(h_vars_);
  for (int i = 1; i < N_; i++) {
    times[i] = times[i - 1] + h_values(i - 1);
  }
  return times;
}

std::vector<Eigen::MatrixXd> DirectTrajectoryOptimization::GetInputVector()
    const {
  std::vector<Eigen::MatrixXd> inputs;
  inputs.reserve(N_);

  const auto u_values = GetSolution(u_vars_);

  for (int i = 0; i < N_; i++) {
    inputs.push_back(u_values.segment(i * num_inputs_, num_inputs_));
  }
  return inputs;
}

std::vector<Eigen::MatrixXd> DirectTrajectoryOptimization::GetStateVector()
    const {
  std::vector<Eigen::MatrixXd> states;
  states.reserve(N_);

  const auto x_values = GetSolution(x_vars_);

  for (int i = 0; i < N_; i++) {
    states.push_back(x_values.segment(i * num_states_, num_states_));
  }
  return states;
}

symbolic::Expression
DirectTrajectoryOptimization::SubstitutePlaceholderVariables(
    const symbolic::Expression& e, int interval_index) const {
  symbolic::Substitution sub;

  // time(i) is the sum of h intervals 0...(i-1)
  const symbolic::Expression time =
      h_vars_.head(interval_index).cast<symbolic::Expression>().sum();

  sub.emplace(placeholder_t_var_(0), time);
  for (int i = 0; i < num_states_; i++)
    sub.emplace(placeholder_x_vars_(i),
                x_vars_(interval_index * num_states_ + i));
  for (int i = 0; i < num_inputs_; i++)
    sub.emplace(placeholder_u_vars_(i),
                u_vars_(interval_index * num_inputs_ + i));
  return e.Substitute(sub);
}

void DirectTrajectoryOptimization::GetResultSamples(
    Eigen::MatrixXd* inputs, Eigen::MatrixXd* states,
    std::vector<double>* times_out) const {
  std::vector<double> times = GetTimeVector();
  times_out->swap(times);

  inputs->resize(num_inputs_, N_);
  inputs->fill(0);
  states->resize(num_states_, N_);
  states->fill(0);

  const auto& u_values = GetSolution(u_vars_);
  const auto& x_values = GetSolution(x_vars_);

  for (int i = 0; i < N_; i++) {
    inputs->col(i) = u_values.segment(i * num_inputs_, num_inputs_);
    states->col(i) = x_values.segment(i * num_states_, num_states_);
  }
}

PiecewisePolynomialTrajectory
DirectTrajectoryOptimization::ReconstructInputTrajectory() const {
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::FirstOrderHold(GetTimeVector(),
                                                  GetInputVector()));
}

PiecewisePolynomialTrajectory
DirectTrajectoryOptimization::ReconstructStateTrajectory() const {
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::FirstOrderHold(GetTimeVector(),
                                                  GetStateVector()));
}

}  // namespace systems
}  // namespace drake
