#include "drake/solvers/aggregate_costs_constraints.h"

#include <map>

namespace drake {
namespace solvers {
namespace {
// @param Q_lower the lower triangular matrix of Q.
// @param linear_coeff_triplets The triplets that records the linear
// coefficient.
// @param vars_vec A vector containing all the variables shown up in the
// quadratic cost.
// @param var_to_index_map Records the index of a variable in vars_vec.
// @param constant_cost The total constant term in the quadratic cost.
void AggregateQuadraticCostsHelper(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs,
    Eigen::SparseMatrix<double>* Q_lower,
    std::vector<symbolic::Variable>* quadratic_vars_vec,
    std::map<symbolic::Variable::Id, int>* quadratic_var_to_index_map,
    std::vector<Eigen::Triplet<double>>* linear_coeff_triplets,
    std::vector<symbolic::Variable>* linear_vars_vec,
    std::map<symbolic::Variable::Id, int>* linear_var_to_index_map,
    double* constant_cost) {
  std::vector<Eigen::Triplet<double>> Q_lower_triplets;
  *constant_cost = 0;
  for (const auto& quadratic_cost : quadratic_costs) {
    const int num_cost_var = quadratic_cost.variables().rows();
    // cost_quadratic_var_indices[i] stores the index of
    // quadratic_cost.variables()(i) in `quadratic_vars`.
    std::vector<int> cost_quadratic_var_indices(num_cost_var);
    for (int i = 0; i < num_cost_var; ++i) {
      int var_index = -1;
      const symbolic::Variable& var = quadratic_cost.variables()(i);
      auto it = quadratic_var_to_index_map->find(var.get_id());
      if (it != quadratic_var_to_index_map->end()) {
        var_index = it->second;
      } else {
        var_index = quadratic_vars_vec->size();
        quadratic_var_to_index_map->emplace_hint(it, var.get_id(), var_index);
        quadratic_vars_vec->push_back(var);
      }
      cost_quadratic_var_indices[i] = var_index;
    }
    for (int i = 0; i < num_cost_var; ++i) {
      if (quadratic_cost.evaluator()->b()(i) != 0) {
        const symbolic::Variable& var_i = quadratic_cost.variables()(i);
        auto it = linear_var_to_index_map->find(var_i.get_id());
        int linear_var_index = -1;
        if (it != linear_var_to_index_map->end()) {
          linear_var_index = it->second;
        } else {
          linear_var_index = linear_vars_vec->size();
          linear_var_to_index_map->emplace_hint(it, var_i.get_id(),
                                                linear_var_index);
          linear_vars_vec->push_back(var_i);
        }

        linear_coeff_triplets->emplace_back(linear_var_index, 0,
                                            quadratic_cost.evaluator()->b()(i));
      }
      for (int j = 0; j < num_cost_var; ++j) {
        if (quadratic_cost.evaluator()->Q()(i, j) != 0) {
          if (cost_quadratic_var_indices[i] >= cost_quadratic_var_indices[j]) {
            Q_lower_triplets.emplace_back(
                cost_quadratic_var_indices[i], cost_quadratic_var_indices[j],
                quadratic_cost.evaluator()->Q()(i, j));
          }
        }
      }
    }
    *constant_cost += quadratic_cost.evaluator()->c();
  }

  *Q_lower = Eigen::SparseMatrix<double>(quadratic_vars_vec->size(),
                                         quadratic_vars_vec->size());
  Q_lower->setFromTriplets(Q_lower_triplets.begin(), Q_lower_triplets.end());
}

void AggregateLinearCostsHelper(
    const std::vector<Binding<LinearCost>>& linear_costs,
    std::vector<Eigen::Triplet<double>>* linear_coeff_triplets,
    std::vector<symbolic::Variable>* vars_vec,
    std::map<symbolic::Variable::Id, int>* var_to_index_map,
    double* constant_cost) {
  for (const auto& cost : linear_costs) {
    const Eigen::SparseVector<double> cost_coeff =
        cost.evaluator()->a().sparseView();
    for (Eigen::SparseVector<double>::InnerIterator it(cost_coeff); it; ++it) {
      const symbolic::Variable var = cost.variables()(it.index());
      auto map_it = var_to_index_map->find(var.get_id());
      int var_index = -1;
      if (map_it == var_to_index_map->end()) {
        var_index = vars_vec->size();
        var_to_index_map->emplace_hint(map_it, var.get_id(), var_index);
        vars_vec->push_back(var);
      } else {
        var_index = map_it->second;
      }
      linear_coeff_triplets->emplace_back(var_index, 0, it.value());
    }
    *constant_cost += cost.evaluator()->b();
  }
}
}  // namespace

void AggregateLinearCosts(const std::vector<Binding<LinearCost>>& linear_costs,
                          Eigen::SparseVector<double>* linear_coeff,
                          VectorX<symbolic::Variable>* vars,
                          double* constant_cost) {
  std::vector<Eigen::Triplet<double>> linear_coeff_triplets;
  // We first store all the variables in vars_vec, and convert it to VectorX
  // object in the end.
  std::vector<symbolic::Variable> vars_vec;
  *constant_cost = 0;
  // Get all the variables shown up in `linear_costs`, and store their index
  // in `vars`.
  std::map<symbolic::Variable::Id, int> var_to_index_map;
  AggregateLinearCostsHelper(linear_costs, &linear_coeff_triplets, &vars_vec,
                             &var_to_index_map, constant_cost);
  *linear_coeff = Eigen::SparseVector<double>(vars_vec.size());
  for (const auto& triplet : linear_coeff_triplets) {
    linear_coeff->coeffRef(triplet.row()) += triplet.value();
  }
  vars->resize(vars_vec.size());
  for (int i = 0; i < static_cast<int>(vars_vec.size()); ++i) {
    (*vars)(i) = vars_vec[i];
  }
}

void AggregateQuadraticAndLinearCosts(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs,
    const std::vector<Binding<LinearCost>>& linear_costs,
    Eigen::SparseMatrix<double>* Q_lower,
    VectorX<symbolic::Variable>* quadratic_vars,
    Eigen::SparseVector<double>* linear_coeff,
    VectorX<symbolic::Variable>* linear_vars, double* constant_cost) {
  std::vector<symbolic::Variable> quadratic_vars_vec;
  std::map<symbolic::Variable::Id, int> quadratic_var_to_index_map;
  std::vector<Eigen::Triplet<double>> linear_coeff_triplets;
  std::vector<symbolic::Variable> linear_vars_vec;
  std::map<symbolic::Variable::Id, int> linear_var_to_index_map;
  AggregateQuadraticCostsHelper(quadratic_costs, Q_lower, &quadratic_vars_vec,
                                &quadratic_var_to_index_map,
                                &linear_coeff_triplets, &linear_vars_vec,
                                &linear_var_to_index_map, constant_cost);
  AggregateLinearCostsHelper(linear_costs, &linear_coeff_triplets,
                             &linear_vars_vec, &linear_var_to_index_map,
                             constant_cost);
  *linear_coeff = Eigen::SparseVector<double>(linear_vars_vec.size());
  for (const auto& triplet : linear_coeff_triplets) {
    linear_coeff->coeffRef(triplet.row()) += triplet.value();
  }
  linear_vars->resize(linear_vars_vec.size());
  for (int i = 0; i < static_cast<int>(linear_vars_vec.size()); ++i) {
    (*linear_vars)(i) = linear_vars_vec[i];
  }
  quadratic_vars->resize(quadratic_vars_vec.size());
  for (int i = 0; i < static_cast<int>(quadratic_vars_vec.size()); ++i) {
    (*quadratic_vars)(i) = quadratic_vars_vec[i];
  }
}
}  // namespace solvers
}  // namespace drake
