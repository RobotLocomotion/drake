#include "drake/solvers/aggregate_costs_constraints.h"

#include <map>

namespace drake {
namespace solvers {
namespace internal {
void AggregateQuadraticCosts(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs,
    Eigen::SparseMatrix<double>* Q_lower,
    Eigen::SparseVector<double>* linear_coeff,
    VectorX<symbolic::Variable>* vars, double* constant_cost) {
  // We first store all the variables in vars_vec, and convert it to VectorX
  // object in the end.
  std::vector<symbolic::Variable> vars_vec{};
  // Get all the variables shown up in `quadratic_costs`, and store their index
  // in `vars`.
  std::map<symbolic::Variable::Id, int> var_to_index_map;
  std::vector<Eigen::Triplet<double>> Q_lower_triplets;
  std::vector<Eigen::Triplet<double>> linear_coeff_triplets;
  *constant_cost = 0;
  for (const auto& quadratic_cost : quadratic_costs) {
    const int num_cost_var = quadratic_cost.variables().rows();
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
        var_index = vars_vec.size();
        var_to_index_map.emplace_hint(it, var.get_id(), var_index);
        vars_vec.push_back(var);
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

  *Q_lower = Eigen::SparseMatrix<double>(vars_vec.size(), vars_vec.size());
  Q_lower->setFromTriplets(Q_lower_triplets.begin(), Q_lower_triplets.end());
  *linear_coeff = Eigen::SparseVector<double>(vars_vec.size());
  for (const auto& triplet : linear_coeff_triplets) {
    linear_coeff->coeffRef(triplet.row()) += triplet.value();
  }
  vars->resize(vars_vec.size());
  for (int i = 0; i < static_cast<int>(vars_vec.size()); ++i) {
    (*vars)(i) = vars_vec[i];
  }
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
