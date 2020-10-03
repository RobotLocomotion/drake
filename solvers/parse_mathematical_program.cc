#include "drake/solvers/parse_mathematical_program.h"

#include <map>

namespace drake {
namespace solvers {
namespace internal {
void AggregateQuadraticCosts(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs,
    Eigen::SparseMatrix<double>* Q_lower,
    Eigen::SparseVector<double>* linear_coeff,
    VectorX<symbolic::Variable>* vars, double* constant_cost) {
  // Get all the variables shown up in `quadratic_costs`, and store their index
  // in `vars`.
  int num_vars = 0;
  vars->resize(0);
  std::map<symbolic::Variable::Id, int> var_to_index_map;
  std::vector<Eigen::Triplet<double>> Q_lower_triplets;
  std::vector<Eigen::Triplet<double>> linear_coeff_triplets;
  *constant_cost = 0;
  for (const auto& quadratic_cost : quadratic_costs) {
    const int num_cost_var = quadratic_cost.variables().rows();
    vars->conservativeResize(vars->rows() + num_cost_var);
    // cost_var_indices[i] stores the index of quadratic_cost.variables()(i) in
    // `vars`.
    std::vector<int> cost_var_indices(num_cost_var);
    for (int i = 0; i < num_cost_var; ++i) {
      int var_index = -1;
      const symbolic::Variable& var = quadratic_cost.variables()(i);
      auto it = var_to_index_map.find(var.get_id());
      if (it != var_to_index_map.end()) {
        var_index = it->second;
      } else {
        var_index = num_vars;
        var_to_index_map.emplace_hint(it, var.get_id(), var_index);
        (*vars)(var_index) = var;
        num_vars++;
      }
      cost_var_indices[i] = var_index;
    }
    for (int i = 0; i < num_cost_var; ++i) {
      if (quadratic_cost.evaluator()->b()(i) != 0) {
        linear_coeff_triplets.emplace_back(cost_var_indices[i], 0,
                                           quadratic_cost.evaluator()->b()(i));
      }
      for (int j = 0; j <= i; ++j) {
        if (quadratic_cost.evaluator()->Q()(i, j) != 0) {
          Q_lower_triplets.emplace_back(cost_var_indices[i],
                                        cost_var_indices[j],
                                        quadratic_cost.evaluator()->Q()(i, j));
        }
      }
    }
    *constant_cost += quadratic_cost.evaluator()->c();
  }

  *Q_lower = Eigen::SparseMatrix<double>(num_vars, num_vars);
  Q_lower->setFromTriplets(Q_lower_triplets.begin(), Q_lower_triplets.end());
  *linear_coeff = Eigen::SparseVector<double>(num_vars);
  for (const auto& triplet : linear_coeff_triplets) {
    linear_coeff->coeffRef(triplet.row()) += triplet.value();
  }
  // vars.rows() is the summation of quadratic_cost.variables().rows(). When
  // different quadratic costs share some variables, the tail entries of vars
  // are just empty. We call conservativeResize to get rid of these empty
  // entries.
  vars->conservativeResize(num_vars);
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
