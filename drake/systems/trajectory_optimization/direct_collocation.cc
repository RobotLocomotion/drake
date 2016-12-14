#include "drake/systems/trajectory_optimization/direct_collocation.h"

#include <stdexcept>

#include "drake/systems/trajectory_optimization/direct_collocation_constraint.h"

namespace drake {
namespace systems {

DircolTrajectoryOptimization::DircolTrajectoryOptimization(
    const systems::System<double>& system,
    const systems::Context<double>& context, int num_time_samples,
    double trajectory_time_lower_bound, double trajectory_time_upper_bound)
    : systems::DirectTrajectoryOptimization(
          system.get_num_total_inputs(), context.get_continuous_state()->size(),
          num_time_samples, trajectory_time_lower_bound,
          trajectory_time_upper_bound) {
  DRAKE_DEMAND(context.has_only_continuous_state());

  // Add the dynamic constraints.
  auto constraint =
      std::make_shared<systems::SystemDirectCollocationConstraint>(system,
                                                                    context);

  DRAKE_ASSERT(static_cast<int>(constraint->num_constraints()) == num_states());

  // For N-1 timesteps, add a constraint which depends on the knot
  // value along with the state and input vectors at that knot and the
  // next.
  for (int i = 0; i < N() - 1; i++) {
    opt_problem()->AddConstraint(
        constraint, {h_vars().segment<1>(i),
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
                            constraint->lower_bound(),
                            constraint->upper_bound()),
        constraint_(constraint) {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    throw std::runtime_error("Non-Taylor constraint eval not implemented.");
  }

  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
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
      : Constraint(constraint->num_constraints(), constraint->lower_bound(),
                   constraint->upper_bound()),
        constraint_(constraint) {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    throw std::runtime_error("Non-Taylor constraint eval not implemented.");
  }

  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
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
  opt_problem()->AddCost(std::make_shared<RunningCostEndWrapper>(constraint),
                         {h_vars().head(1), x_vars().head(num_states()),
                          u_vars().head(num_inputs())});

  for (int i = 1; i < N() - 1; i++) {
    opt_problem()->AddCost(std::make_shared<RunningCostMidWrapper>(constraint),
                           {h_vars().segment(i - 1, 2),
                            x_vars().segment(i * num_states(), num_states()),
                            u_vars().segment(i * num_inputs(), num_inputs())});
  }

  opt_problem()->AddCost(std::make_shared<RunningCostEndWrapper>(constraint),
                         {h_vars().tail(1), x_vars().tail(num_states()),
                          u_vars().tail(num_inputs())});
}

}  // namespace systems
}  // namespace drake
