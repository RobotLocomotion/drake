#include "drake/systems/controllers/dynamic_programming.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/math/wrap_to.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace controllers {

DynamicProgrammingOptions::PeriodicBoundaryCondition::PeriodicBoundaryCondition(
    int state_index_in, double low_in, double high_in)
    : state_index(state_index_in), low(low_in), high(high_in) {
  DRAKE_DEMAND(low_in < high_in);
}

std::pair<std::unique_ptr<BarycentricMeshSystem<double>>, Eigen::RowVectorXd>
FittedValueIteration(
    Simulator<double>* simulator,
    const std::function<double(const Context<double>& context)>& cost_function,
    const math::BarycentricMesh<double>::MeshGrid& state_grid,
    const math::BarycentricMesh<double>::MeshGrid& input_grid, double timestep,
    const DynamicProgrammingOptions& options) {
  DRAKE_DEMAND(options.discount_factor > 0. && options.discount_factor <= 1.);

  const int state_size = state_grid.size();
  const int input_size = input_grid.size();
  DRAKE_DEMAND(state_size > 0);
  DRAKE_DEMAND(input_size > 0);

  const auto& system = simulator->get_system();
  auto& context = simulator->get_mutable_context();

  math::BarycentricMesh<double> state_mesh(state_grid);
  math::BarycentricMesh<double> input_mesh(input_grid);

  const int num_states = state_mesh.get_num_mesh_points();
  const int num_inputs = input_mesh.get_num_mesh_points();
  const int num_state_indices = state_mesh.get_num_interpolants();

  // TODO(russt): handle discrete state.
  DRAKE_DEMAND(context.has_only_continuous_state() ||
               options.assume_non_continuous_states_are_fixed);
  DRAKE_DEMAND(context.num_continuous_states() == state_size);

  const InputPort<double>* input_port =
      system.get_input_port_selection(options.input_port_index);
  DRAKE_DEMAND(input_port != nullptr);
  DRAKE_DEMAND(input_port->size() == input_size);

  DRAKE_DEMAND(timestep > 0.);

  // TODO(russt): check that the system is time-invariant.

  // Make sure all periodic boundary conditions are in range.
  for (const auto& b : options.periodic_boundary_conditions) {
    DRAKE_DEMAND(b.state_index >= 0 && b.state_index < state_size);
    DRAKE_DEMAND(b.low < b.high);
    DRAKE_DEMAND(b.low >= *(state_grid[b.state_index].begin()));
    DRAKE_DEMAND(b.high <= *(state_grid[b.state_index].rbegin()));
  }

  // The transition probabilities are represented as a sparse matrix,
  // where Tind[input](:,state) is a list of non-zero indexes into the
  // state_mesh, and T[input](:,state) is the associated list of coefficients.
  // cost[input](j) is the cost of taking action input from state mesh index j.
  std::vector<Eigen::MatrixXi> Tind(num_inputs);
  std::vector<Eigen::MatrixXd> T(num_inputs);
  std::vector<Eigen::RowVectorXd> cost(num_inputs);

  drake::log()->info("Computing transition and cost matrices.");
  auto& sim_state = context.get_mutable_continuous_state_vector();

  Eigen::VectorXd input_vec(input_mesh.get_input_size());
  Eigen::VectorXd state_vec(state_mesh.get_input_size());

  Eigen::VectorXi Tind_tmp(num_state_indices);
  Eigen::VectorXd T_tmp(num_state_indices);

  for (int input = 0; input < num_inputs; input++) {
    Tind[input].resize(num_state_indices, num_states);
    T[input].resize(num_state_indices, num_states);
    cost[input].resize(num_states);

    input_mesh.get_mesh_point(input, &input_vec);
    input_port->FixValue(&context, input_vec);

    for (int state = 0; state < num_states; state++) {
      context.SetTime(0.0);
      sim_state.SetFromVector(state_mesh.get_mesh_point(state));
      simulator->Initialize();

      cost[input](state) = timestep * cost_function(context);

      simulator->AdvanceTo(timestep);
      state_vec = sim_state.CopyToVector();

      for (const auto& b : options.periodic_boundary_conditions) {
        state_vec[b.state_index] =
            math::wrap_to(state_vec[b.state_index], b.low, b.high);
      }

      state_mesh.EvalBarycentricWeights(state_vec, &Tind_tmp, &T_tmp);
      Tind[input].col(state) = Tind_tmp;
      T[input].col(state) = T_tmp;
    }
  }
  drake::log()->info("Done computing transition and cost matrices.");

  // Perform value iteration loop.
  Eigen::RowVectorXd J = Eigen::RowVectorXd::Zero(num_states);
  Eigen::RowVectorXd Jnext(num_states);
  Eigen::MatrixXd Pi(input_mesh.get_input_size(), num_states);

  drake::log()->info("Running value iteration.");
  double max_diff = std::numeric_limits<double>::infinity();
  int iteration = 0;
  while (max_diff > options.convergence_tol) {
    for (int state = 0; state < num_states; state++) {
      Jnext(state) = std::numeric_limits<double>::infinity();

      int best_input = 0;
      for (int input = 0; input < num_inputs; input++) {
        // Q(x,u) = g(x,u) + γ J(f(x,u)).
        double Q = cost[input](state);
        for (int index = 0; index < num_state_indices; index++) {
          Q += options.discount_factor * T[input](index, state) *
               J(Tind[input](index, state));
        }
        // Cost-to-go: J = minᵤ Q(x,u).
        // Policy:  π(x) = argminᵤ Q(x,u).
        if (Q < Jnext(state)) {
          Jnext(state) = Q;
          best_input = input;
        }
      }
      Pi.col(state) = input_mesh.get_mesh_point(best_input);
    }
    max_diff = (J - Jnext).lpNorm<Eigen::Infinity>();
    J = Jnext;
    iteration++;
    if (options.visualization_callback) {
      options.visualization_callback(iteration, state_mesh, J, Pi);
    }
  }
  drake::log()->info("Value iteration converged to requested tolerance.");

  // Create the policy.
  auto policy = std::make_unique<BarycentricMeshSystem<double>>(state_mesh, Pi);

  return std::make_pair(std::move(policy), J);
}

Eigen::VectorXd LinearProgrammingApproximateDynamicProgramming(
    Simulator<double>* simulator,
    const std::function<double(const Context<double>& context)>& cost_function,
    const std::function<
        symbolic::Expression(const Eigen::Ref<const Eigen::VectorXd>& state,
                             const VectorX<symbolic::Variable>& parameters)>&
        linearly_parameterized_cost_to_go_function,
    int num_parameters, const Eigen::Ref<const Eigen::MatrixXd>& state_samples,
    const Eigen::Ref<const Eigen::MatrixXd>& input_samples, double timestep,
    const DynamicProgrammingOptions& options) {
  // discount_factor needs to be < 1 to avoid unbounded solutions (J = J* + ∞).
  DRAKE_DEMAND(options.discount_factor > 0. && options.discount_factor <= 1.);
  DRAKE_DEMAND(num_parameters > 0);

  const int state_size = state_samples.rows();
  const int input_size = input_samples.rows();
  const int num_states = state_samples.cols();
  const int num_inputs = input_samples.cols();

  DRAKE_DEMAND(state_size > 0);
  DRAKE_DEMAND(input_size > 0);

  const auto& system = simulator->get_system();
  auto& context = simulator->get_mutable_context();

  // TODO(russt): handle discrete state.
  DRAKE_DEMAND(context.has_only_continuous_state());
  DRAKE_DEMAND(context.num_continuous_states() == state_size);

  DRAKE_DEMAND(context.num_input_ports() == 1);
  DRAKE_DEMAND(system.num_total_inputs() == input_size);

  DRAKE_DEMAND(timestep > 0.);

  // TODO(russt): check that the system is time-invariant.

  // TODO(russt): implement wrapping (API doesn't provide enough info yet).
  // Make sure all periodic boundary conditions are in range.
  for (const auto& b : options.periodic_boundary_conditions) {
    DRAKE_DEMAND(b.state_index >= 0 && b.state_index < state_size);
    DRAKE_DEMAND(b.low < b.high);
  }

  drake::log()->info(
      "Computing one-step dynamics and setting up the linear program.");
  solvers::MathematicalProgram prog;

  const solvers::VectorXDecisionVariable params =
      prog.NewContinuousVariables(num_parameters, "a");

  // Alias the function name for readability below.
  const auto& J = linearly_parameterized_cost_to_go_function;

  // Maximize ∑ J.
  for (int state = 0; state < num_states; state++) {
    prog.AddLinearCost(-J(state_samples.col(state), params));
  }

  // ∀x, ∀u, J(x) ≤ cost(x,u) + γJ(f(x,u)).
  auto& sim_state = context.get_mutable_continuous_state_vector();
  Eigen::VectorXd state_vec(state_size);
  Eigen::VectorXd next_state_vec(state_size);
  for (int input = 0; input < num_inputs; input++) {
    system.get_input_port(0).FixValue(&context, input_samples.col(input));
    for (int state = 0; state < num_states; state++) {
      context.SetTime(0.0);
      state_vec = state_samples.col(state);
      sim_state.SetFromVector(state_vec);
      simulator->Initialize();

      const double cost = timestep * cost_function(context);

      simulator->AdvanceTo(timestep);
      next_state_vec = sim_state.CopyToVector();

      for (const auto& b : options.periodic_boundary_conditions) {
        next_state_vec[b.state_index] =
            math::wrap_to(next_state_vec[b.state_index], b.low, b.high);
      }

      const symbolic::Formula f =
          (J(state_vec, params) <=
           cost + options.discount_factor * J(next_state_vec, params));

      // Filter out constraints that are trivially true (because
      // AddConstraint throws).
      if (!symbolic::is_true(f)) {
        prog.AddLinearConstraint(f);
      }
    }
  }

  drake::log()->info("Solving linear program.");
  const solvers::MathematicalProgramResult result = Solve(prog);
  if (!result.is_success()) {
    drake::log()->error("No solution found.  SolutionResult = " +
                        to_string(result.get_solution_result()));
  }
  drake::log()->info("Done solving linear program.");

  return result.GetSolution(params);
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake
