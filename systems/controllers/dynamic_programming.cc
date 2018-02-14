#include "drake/systems/controllers/dynamic_programming.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/math/wrap_to.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace controllers {

std::pair<std::unique_ptr<BarycentricMeshSystem<double>>, Eigen::MatrixXd>
FittedValueIteration(
    Simulator<double>* simulator,
    const std::function<double(const Context<double>& context)>& cost_function,
    const math::BarycentricMesh<double>::MeshGrid& state_grid,
    const math::BarycentricMesh<double>::MeshGrid& input_grid, double timestep,
    const DynamicProgrammingOptions& options) {
  // TODO(russt): handle discrete state.
  const auto& system = simulator->get_system();
  auto& context = simulator->get_mutable_context();

  DRAKE_DEMAND(context.has_only_continuous_state());
  DRAKE_DEMAND(context.get_continuous_state().size() ==
               static_cast<int>(state_grid.size()));

  DRAKE_DEMAND(context.get_num_input_ports() == 1);
  DRAKE_DEMAND(system.get_num_total_inputs() ==
               static_cast<int>(input_grid.size()));

  DRAKE_DEMAND(timestep > 0.);
  DRAKE_DEMAND(options.discount_factor > 0. && options.discount_factor <= 1.);
  if (!options.state_indices_with_periodic_boundary_conditions.empty()) {
    // Make sure all periodic boundary conditions are in range.
    DRAKE_DEMAND(
        *options.state_indices_with_periodic_boundary_conditions.begin() >= 0);
    DRAKE_DEMAND(
        *options.state_indices_with_periodic_boundary_conditions.rbegin() <
        context.get_continuous_state().size());
  }

  // TODO(russt): check that the system is time-invariant.

  math::BarycentricMesh<double> state_mesh(state_grid);
  math::BarycentricMesh<double> input_mesh(input_grid);

  const int num_states = state_mesh.get_num_mesh_points();
  const int num_inputs = input_mesh.get_num_mesh_points();
  const int num_state_indices = state_mesh.get_num_interpolants();

  // The transition probabilities are represented as a sparse matrix,
  // where Tind[input](:,state) is a list of non-zero indexes into the
  // state_mesh, and T[input](:,state) is the associated list of coefficients.
  std::vector<Eigen::MatrixXi> Tind(num_inputs);
  std::vector<Eigen::MatrixXd> T(num_inputs);
  // cost[input](j) is the cost of taking action input from state mesh index j.
  std::vector<Eigen::RowVectorXd> cost(num_inputs);

  {  // Build transition matrices.
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
      context.FixInputPort(0, input_vec);

      for (int state = 0; state < num_states; state++) {
        context.set_time(0.0);
        sim_state.SetFromVector(state_mesh.get_mesh_point(state));

        cost[input](state) = timestep * cost_function(context);

        simulator->StepTo(timestep);
        state_vec = sim_state.CopyToVector();

        for (int dim :
             options.state_indices_with_periodic_boundary_conditions) {
          const double low = *state_grid[dim].begin();
          const double high = *state_grid[dim].rbegin();
          state_vec[dim] = math::wrap_to(state_vec[dim], low, high);
        }

        state_mesh.EvalBarycentricWeights(state_vec, &Tind_tmp, &T_tmp);
        Tind[input].col(state) = Tind_tmp;
        T[input].col(state) = T_tmp;
      }
    }
    drake::log()->info("Done computing transition and cost matrices.");
  }

  // Perform value iteration loop.
  Eigen::RowVectorXd J = Eigen::RowVectorXd::Zero(num_states);
  Eigen::RowVectorXd Jnext(num_states);
  Eigen::MatrixXd Pi(input_mesh.get_input_size(), num_states);

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

  // Create the policy.
  auto policy = std::make_unique<BarycentricMeshSystem<double>>(state_mesh, Pi);

  return std::make_pair(std::move(policy), J);
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake
