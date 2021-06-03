#include "drake/solvers/aggregate_costs_constraints.h"

#include <algorithm>
#include <limits>
#include <map>

namespace drake {
namespace solvers {
namespace {
const double kInf = std::numeric_limits<double>::infinity();
// A helper class to add variable to an ordered vector while aggregating
// costs/constraints.
class VariableVector {
 public:
  VariableVector() {}

  // Get the index of a variable if it is in the vector already, otherwise add
  // it to the vector. Returns the index of this variable in the vector.
  int GetOrAdd(const symbolic::Variable& var) {
    const auto it = var_to_index_map_.find(var.get_id());
    int var_index = -1;
    if (it == var_to_index_map_.end()) {
      var_index = vars_.size();
      vars_.push_back(var);
      var_to_index_map_.emplace_hint(it, var.get_id(), var_index);
    } else {
      var_index = it->second;
    }
    return var_index;
  }

  // Returns the variable as an Eigen vector.
  VectorX<symbolic::Variable> CopyToEigen() const {
    VectorX<symbolic::Variable> result(vars_.size());
    for (int i = 0; i < static_cast<int>(vars_.size()); ++i) {
      result(i) = vars_[i];
    }
    return result;
  }

  int size() const { return vars_.size(); }

 private:
  std::vector<symbolic::Variable> vars_;
  std::map<symbolic::Variable::Id, int> var_to_index_map_;
};

// @param Q_lower[out] the lower triangular matrix of Q.
// @param quadratic_var_vec[out] A vector containing all the variables shown
// up in the quadratic cost.
// @param linear_coeff_triplets[in/out] The coefficients of the linear cost.
// @param linear_var_vec[in/out] The variables in the linear costs.
// @param constant_cost[in/out] The total constant term in the quadratic cost.
void AggregateQuadraticCostsHelper(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs,
    Eigen::SparseMatrix<double>* Q_lower, VariableVector* quadratic_var_vec,
    std::vector<Eigen::Triplet<double>>* linear_coeff_triplets,
    VariableVector* linear_var_vec, double* constant_cost) {
  std::vector<Eigen::Triplet<double>> Q_lower_triplets;
  for (const auto& quadratic_cost : quadratic_costs) {
    const int num_cost_var = quadratic_cost.variables().rows();
    // cost_quadratic_var_indices[i] stores the index of
    // quadratic_cost.variables()(i) in `quadratic_vars`.
    std::vector<int> cost_quadratic_var_indices(num_cost_var);
    for (int i = 0; i < num_cost_var; ++i) {
      const symbolic::Variable& var = quadratic_cost.variables()(i);
      cost_quadratic_var_indices[i] = quadratic_var_vec->GetOrAdd(var);
    }
    for (int i = 0; i < num_cost_var; ++i) {
      if (quadratic_cost.evaluator()->b()(i) != 0) {
        const symbolic::Variable& var_i = quadratic_cost.variables()(i);
        const int linear_var_index = linear_var_vec->GetOrAdd(var_i);

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

  *Q_lower = Eigen::SparseMatrix<double>(quadratic_var_vec->size(),
                                         quadratic_var_vec->size());
  Q_lower->setFromTriplets(Q_lower_triplets.begin(), Q_lower_triplets.end());
}

// @param linear_coeff_triplets[in/out] The coefficient of linear costs.
// @param var_vec[in/out] The linear variables.
// @param constant_cost[in/out] The constant term in the cost.
void AggregateLinearCostsHelper(
    const std::vector<Binding<LinearCost>>& linear_costs,
    std::vector<Eigen::Triplet<double>>* linear_coeff_triplets,
    VariableVector* var_vec, double* constant_cost) {
  for (const auto& cost : linear_costs) {
    const Eigen::SparseVector<double> cost_coeff =
        cost.evaluator()->a().sparseView();
    for (Eigen::SparseVector<double>::InnerIterator it(cost_coeff); it; ++it) {
      const symbolic::Variable var = cost.variables()(it.index());
      const int var_index = var_vec->GetOrAdd(var);
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
  // We first store all the variables in var_vec, and convert it to VectorX
  // object in the end.
  VariableVector var_vec{};
  *constant_cost = 0;
  AggregateLinearCostsHelper(linear_costs, &linear_coeff_triplets, &var_vec,
                             constant_cost);
  *linear_coeff = Eigen::SparseVector<double>(var_vec.size());
  for (const auto& triplet : linear_coeff_triplets) {
    linear_coeff->coeffRef(triplet.row()) += triplet.value();
  }
  *vars = var_vec.CopyToEigen();
}

void AggregateQuadraticAndLinearCosts(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs,
    const std::vector<Binding<LinearCost>>& linear_costs,
    Eigen::SparseMatrix<double>* Q_lower,
    VectorX<symbolic::Variable>* quadratic_vars,
    Eigen::SparseVector<double>* linear_coeff,
    VectorX<symbolic::Variable>* linear_vars, double* constant_cost) {
  VariableVector quadratic_var_vec{};
  std::vector<Eigen::Triplet<double>> linear_coeff_triplets;
  VariableVector linear_var_vec{};
  *constant_cost = 0;
  AggregateQuadraticCostsHelper(quadratic_costs, Q_lower, &quadratic_var_vec,
                                &linear_coeff_triplets, &linear_var_vec,
                                constant_cost);
  AggregateLinearCostsHelper(linear_costs, &linear_coeff_triplets,
                             &linear_var_vec, constant_cost);
  *linear_coeff = Eigen::SparseVector<double>(linear_var_vec.size());
  for (const auto& triplet : linear_coeff_triplets) {
    linear_coeff->coeffRef(triplet.row()) += triplet.value();
  }
  *linear_vars = linear_var_vec.CopyToEigen();
  *quadratic_vars = quadratic_var_vec.CopyToEigen();
}

std::unordered_map<symbolic::Variable, Bound> AggregateBoundingBoxConstraints(
    const std::vector<Binding<BoundingBoxConstraint>>&
        bounding_box_constraints) {
  std::unordered_map<symbolic::Variable, Bound> bounds;
  for (const auto& constraint : bounding_box_constraints) {
    for (int i = 0; i < constraint.variables().rows(); ++i) {
      const symbolic::Variable& var = constraint.variables()(i);
      const double var_lower = constraint.evaluator()->lower_bound()(i);
      const double var_upper = constraint.evaluator()->upper_bound()(i);
      auto it = bounds.find(var);
      if (it == bounds.end()) {
        bounds.emplace_hint(it, var,
                            Bound{.lower = var_lower, .upper = var_upper});
      } else {
        if (var_lower > it->second.lower) {
          it->second.lower = var_lower;
        }
        if (var_upper < it->second.upper) {
          it->second.upper = var_upper;
        }
      }
    }
  }
  return bounds;
}

void AggregateBoundingBoxConstraints(const MathematicalProgram& prog,
                                     Eigen::VectorXd* lower,
                                     Eigen::VectorXd* upper) {
  *lower = Eigen::VectorXd::Constant(prog.num_vars(), -kInf);
  *upper = Eigen::VectorXd::Constant(prog.num_vars(), kInf);
  for (const auto& constraint : prog.bounding_box_constraints()) {
    for (int i = 0; i < constraint.variables().rows(); ++i) {
      const int var_index =
          prog.FindDecisionVariableIndex(constraint.variables()(i));
      if (constraint.evaluator()->lower_bound()(i) > (*lower)(var_index)) {
        (*lower)(var_index) = constraint.evaluator()->lower_bound()(i);
      }
      if (constraint.evaluator()->upper_bound()(i) < (*upper)(var_index)) {
        (*upper)(var_index) = constraint.evaluator()->upper_bound()(i);
      }
    }
  }
}

const Binding<QuadraticCost>* FindNonconvexQuadraticCost(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs) {
  for (const auto& cost : quadratic_costs) {
    if (!cost.evaluator()->is_convex()) {
      return &cost;
    }
  }
  return nullptr;
}

namespace internal {
bool CheckConvexSolverAttributes(const MathematicalProgram& prog,
                                 const ProgramAttributes& solver_capabilities,
                                 std::string_view solver_name,
                                 std::string* explanation) {
  const ProgramAttributes& required_capabilities = prog.required_capabilities();
  const bool capabilities_match = AreRequiredAttributesSupported(
      required_capabilities, solver_capabilities, explanation);
  if (!capabilities_match) {
    if (explanation) {
      *explanation = fmt::format("{} is unable to solve because {}.",
                                 solver_name, *explanation);
    }
    return false;
  }
  const Binding<QuadraticCost>* nonconvex_quadratic_cost =
      FindNonconvexQuadraticCost(prog.quadratic_costs());
  if (nonconvex_quadratic_cost != nullptr) {
    if (explanation) {
      *explanation = fmt::format(
          "{} is unable to solve because (at least) the quadratic cost {} is "
          "non-convex. Either change this cost to a convex one, or switch "
          "to a different solver like SNOPT/IPOPT/NLOPT.",
          solver_name, nonconvex_quadratic_cost->to_string());
    }
    return false;
  }
  if (explanation) {
    explanation->clear();
  }
  return true;
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
