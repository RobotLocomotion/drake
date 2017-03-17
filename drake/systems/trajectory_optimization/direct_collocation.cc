#include "drake/systems/trajectory_optimization/direct_collocation.h"

#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/systems/trajectory_optimization/direct_collocation_constraint.h"

namespace drake {
namespace systems {

DircolTrajectoryOptimization::DircolTrajectoryOptimization(
    const systems::System<double>* system,
    const systems::Context<double>& context, int num_time_samples,
    double trajectory_time_lower_bound, double trajectory_time_upper_bound)
    : systems::DirectTrajectoryOptimization(
          system->get_num_total_inputs(),
          context.get_continuous_state()->size(), num_time_samples,
          trajectory_time_lower_bound, trajectory_time_upper_bound),
      system_(system),
      context_(system_->CreateDefaultContext()),
      continuous_state_(system_->AllocateTimeDerivatives()) {
  DRAKE_DEMAND(context.has_only_continuous_state());

  context_->SetTimeStateAndParametersFrom(context);

  // Allocate the input port and keep an alias around.
  input_port_value_ = new FreestandingInputPortValue(
      std::make_unique<BasicVector<double>>(system_->get_input_port(0).size()));
  std::unique_ptr<InputPortValue> input_port_value(input_port_value_);
  context_->SetInputPortValue(0, std::move(input_port_value));

  // Add the dynamic constraints.
  auto constraint =
      std::make_shared<systems::SystemDirectCollocationConstraint>(*system,
                                                                   context);

  DRAKE_ASSERT(static_cast<int>(constraint->num_constraints()) == num_states());

  // For N-1 timesteps, add a constraint which depends on the knot
  // value along with the state and input vectors at that knot and the
  // next.
  for (int i = 0; i < N() - 1; i++) {
    AddConstraint(constraint,
                  {h_vars().segment<1>(i),
                   x_vars().segment(i * num_states(), num_states() * 2),
                   u_vars().segment(i * num_inputs(), num_inputs() * 2)});
  }
}

namespace {
/// Since the running cost evaluation needs the timestep mangled, we
/// need to wrap it and convert the input.
class RunningCostEndWrapper : public solvers::Constraint {
 public:
  explicit RunningCostEndWrapper(
      std::shared_ptr<solvers::Constraint> constraint)
      : solvers::Constraint(constraint->num_constraints(),
                            constraint->num_vars(), constraint->lower_bound(),
                            constraint->upper_bound()),
        constraint_(constraint) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    throw std::runtime_error("Non-Taylor constraint eval not implemented.");
  }

  void DoEval(const Eigen::Ref<const TaylorVecXd>& x,
              TaylorVecXd& y) const override {
    TaylorVecXd wrapped_x = x;
    wrapped_x(0) *= 0.5;
    constraint_->Eval(wrapped_x, y);
  };

 private:
  std::shared_ptr<Constraint> constraint_;
};

class RunningCostMidWrapper : public solvers::Constraint {
 public:
  explicit RunningCostMidWrapper(
      std::shared_ptr<solvers::Constraint> constraint)
      : Constraint(constraint->num_constraints(),
                   constraint->num_vars() + 1,  // We wrap x(0) and x(1) into
                                                // (x(0) + x(1)) * 0.5, so one
                                                // less variable when calling
                                                // Eval.
                   constraint->lower_bound(), constraint->upper_bound()),
        constraint_(constraint) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    throw std::runtime_error("Non-Taylor constraint eval not implemented.");
  }

  void DoEval(const Eigen::Ref<const TaylorVecXd>& x,
              TaylorVecXd& y) const override {
    TaylorVecXd wrapped_x(x.rows() - 1);
    wrapped_x.tail(x.rows() - 2) = x.tail(x.rows() - 2);
    wrapped_x(0) = (x(0) + x(1)) * 0.5;
    constraint_->Eval(wrapped_x, y);
  };

 private:
  std::shared_ptr<Constraint> constraint_;
};

}  // anon namespace

// We just use a generic constraint here since we need to mangle the
// input and output anyway.
void DircolTrajectoryOptimization::AddRunningCost(
    std::shared_ptr<solvers::Constraint> constraint) {
  AddCost(std::make_shared<RunningCostEndWrapper>(constraint),
          {h_vars().head(1), x_vars().head(num_states()),
           u_vars().head(num_inputs())});

  for (int i = 1; i < N() - 1; i++) {
    AddCost(std::make_shared<RunningCostMidWrapper>(constraint),
            {h_vars().segment(i - 1, 2),
             x_vars().segment(i * num_states(), num_states()),
             u_vars().segment(i * num_inputs(), num_inputs())});
  }

  AddCost(std::make_shared<RunningCostEndWrapper>(constraint),
          {h_vars().tail(1), x_vars().tail(num_states()),
           u_vars().tail(num_inputs())});
}

PiecewisePolynomialTrajectory
DircolTrajectoryOptimization::ReconstructStateTrajectory() const {
  const std::vector<Eigen::MatrixXd> input_vec = GetInputVector();
  const std::vector<Eigen::MatrixXd> state_vec = GetStateVector();
  std::vector<Eigen::MatrixXd> derivatives;
  derivatives.reserve(input_vec.size());

  for (size_t i = 0; i < input_vec.size(); ++i) {
    input_port_value_->GetMutableVectorData<double>()->SetFromVector(
        input_vec[i]);
    context_->get_mutable_continuous_state()->SetFromVector(state_vec[i]);
    system_->CalcTimeDerivatives(*context_, continuous_state_.get());
    derivatives.push_back(continuous_state_->CopyToVector());
  }
  return PiecewisePolynomialTrajectory(PiecewisePolynomial<double>::Cubic(
      GetTimeVector(), state_vec, derivatives));
}

}  // namespace systems
}  // namespace drake
