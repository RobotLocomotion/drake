#include "drake/solvers/aggregate_costs_constraints.h"

#include <algorithm>
#include <limits>
#include <map>

#include <fmt/format.h>

#include "drake/math/eigen_sparse_triplet.h"

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

namespace {
template <typename C>
void AggregateBoundingBoxConstraintsImpl(const MathematicalProgram& prog,
                                         C* lower, C* upper) {
  for (const auto& constraint : prog.bounding_box_constraints()) {
    for (int i = 0; i < constraint.variables().rows(); ++i) {
      const int var_index =
          prog.FindDecisionVariableIndex(constraint.variables()(i));
      if (constraint.evaluator()->lower_bound()(i) > (*lower)[var_index]) {
        (*lower)[var_index] = constraint.evaluator()->lower_bound()(i);
      }
      if (constraint.evaluator()->upper_bound()(i) < (*upper)[var_index]) {
        (*upper)[var_index] = constraint.evaluator()->upper_bound()(i);
      }
    }
  }
}

void EmplaceNonzeroTriplet(int row, int col, double val,
                           std::vector<Eigen::Triplet<double>>* triplets) {
  if (val != 0) {
    triplets->emplace_back(row, col, val);
  }
}

}  // namespace

void AggregateBoundingBoxConstraints(const MathematicalProgram& prog,
                                     Eigen::VectorXd* lower,
                                     Eigen::VectorXd* upper) {
  *lower = Eigen::VectorXd::Constant(prog.num_vars(), -kInf);
  *upper = Eigen::VectorXd::Constant(prog.num_vars(), kInf);
  AggregateBoundingBoxConstraintsImpl(prog, lower, upper);
}

void AggregateBoundingBoxConstraints(const MathematicalProgram& prog,
                                     std::vector<double>* lower,
                                     std::vector<double>* upper) {
  *lower = std::vector<double>(prog.num_vars(), -kInf);
  *upper = std::vector<double>(prog.num_vars(), kInf);
  AggregateBoundingBoxConstraintsImpl(prog, lower, upper);
}

void AggregateDuplicateVariables(const Eigen::SparseMatrix<double>& A,
                                 const VectorX<symbolic::Variable>& vars,
                                 Eigen::SparseMatrix<double>* A_new,
                                 VectorX<symbolic::Variable>* vars_new) {
  // First select the unique variables from decision_vars.
  vars_new->resize(vars.rows());
  // vars_to_vars_new records the mapping from vars to vars_new. Namely
  // vars[i] = (*vars_new)[vars_to_vars_new[vars[i].get_id()]]
  std::unordered_map<symbolic::Variable::Id, int> vars_to_vars_new;
  int unique_var_count = 0;
  for (int i = 0; i < vars.rows(); ++i) {
    const bool seen_already = vars_to_vars_new.contains(vars(i).get_id());
    if (!seen_already) {
      (*vars_new)(unique_var_count) = vars(i);
      vars_to_vars_new.emplace(vars(i).get_id(), unique_var_count);
      unique_var_count++;
    }
  }
  // A * vars = A_new * vars_new.
  // A_new_triplets are the non-zero triplets in A_new.
  std::vector<Eigen::Triplet<double>> A_new_triplets;
  A_new_triplets.reserve(A.nonZeros());
  for (int i = 0; i < A.cols(); ++i) {
    const int A_new_col = vars_to_vars_new.at(vars(i).get_id());
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
      A_new_triplets.emplace_back(it.row(), A_new_col, it.value());
    }
  }
  *A_new = Eigen::SparseMatrix<double>(A.rows(), unique_var_count);
  A_new->setFromTriplets(A_new_triplets.begin(), A_new_triplets.end());
  vars_new->conservativeResize(unique_var_count);
}

namespace internal {
const Binding<QuadraticCost>* FindNonconvexQuadraticCost(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs) {
  for (const auto& cost : quadratic_costs) {
    if (!cost.evaluator()->is_convex()) {
      return &cost;
    }
  }
  return nullptr;
}

const Binding<QuadraticConstraint>* FindNonconvexQuadraticConstraint(
    const std::vector<Binding<QuadraticConstraint>>& quadratic_constraints) {
  for (const auto& constraint : quadratic_constraints) {
    if (!constraint.evaluator()->is_convex()) {
      return &constraint;
    }
  }
  return nullptr;
}

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
      internal::FindNonconvexQuadraticCost(prog.quadratic_costs());
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
  const Binding<QuadraticConstraint>* nonconvex_quadratic_constraint =
      internal::FindNonconvexQuadraticConstraint(prog.quadratic_constraints());
  if (nonconvex_quadratic_constraint != nullptr) {
    if (explanation) {
      *explanation = fmt::format(
          "{} is unable to solve because (at least) the quadratic constraint "
          "{} is non-convex. Either change this constraint to a convex one, or "
          "switch to a different solver like SNOPT/IPOPT/NLOPT.",
          solver_name, nonconvex_quadratic_constraint->to_string());
    }
    return false;
  }
  return true;
}

void ParseLinearCosts(const MathematicalProgram& prog, std::vector<double>* c,
                      double* constant) {
  DRAKE_DEMAND(static_cast<int>(c->size()) >= prog.num_vars());
  for (const auto& linear_cost : prog.linear_costs()) {
    // Each linear cost is in the form of aᵀx + b
    const auto& a = linear_cost.evaluator()->a();
    const VectorXDecisionVariable& x = linear_cost.variables();
    for (int i = 0; i < a.rows(); ++i) {
      (*c)[prog.FindDecisionVariableIndex(x(i))] += a(i);
    }
    (*constant) += linear_cost.evaluator()->b();
  }
}

void ParseLinearEqualityConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, std::vector<int>* linear_eq_y_start_indices,
    int* num_linear_equality_constraints_rows) {
  DRAKE_ASSERT(linear_eq_y_start_indices->empty());
  DRAKE_ASSERT(static_cast<int>(b->size()) == *A_row_count);
  *num_linear_equality_constraints_rows = 0;
  linear_eq_y_start_indices->reserve(prog.linear_equality_constraints().size());
  // The linear equality constraint A x = b is converted to
  // A x + s = b. s in zero cone.
  for (const auto& linear_equality_constraint :
       prog.linear_equality_constraints()) {
    const Eigen::SparseMatrix<double>& Ai =
        linear_equality_constraint.evaluator()->get_sparse_A();
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(Ai);
    A_triplets->reserve(A_triplets->size() + Ai_triplets.size());
    const solvers::VectorXDecisionVariable& x =
        linear_equality_constraint.variables();
    // x_indices[i] is the index of x(i)
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(Ai_triplet.row() + *A_row_count,
                               x_indices[Ai_triplet.col()], Ai_triplet.value());
    }
    const int num_Ai_rows =
        linear_equality_constraint.evaluator()->num_constraints();
    b->reserve(b->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; ++i) {
      b->push_back(linear_equality_constraint.evaluator()->lower_bound()(i));
    }
    linear_eq_y_start_indices->push_back(*A_row_count);
    *A_row_count += num_Ai_rows;
    *num_linear_equality_constraints_rows += num_Ai_rows;
  }
}

void ParseLinearConstraints(const solvers::MathematicalProgram& prog,
                            std::vector<Eigen::Triplet<double>>* A_triplets,
                            std::vector<double>* b, int* A_row_count,
                            std::vector<std::vector<std::pair<int, int>>>*
                                linear_constraint_dual_indices,
                            int* num_linear_constraint_rows) {
  // The linear constraint lb ≤ aᵀx ≤ ub is converted to
  // -aᵀx + s1 = lb,
  //  aᵀx + s2 = ub
  // s1, s2 in the positive cone.
  // The special cases are when ub = ∞ or lb = -∞.
  // When ub = ∞, then we only add the constraint
  // -aᵀx + s = lb, s in the positive cone.
  // When lb = -∞, then we only add the constraint
  // aᵀx + s = ub, s in the positive cone.
  *num_linear_constraint_rows = 0;
  linear_constraint_dual_indices->reserve(prog.linear_constraints().size());
  for (const auto& linear_constraint : prog.linear_constraints()) {
    linear_constraint_dual_indices->emplace_back(
        linear_constraint.evaluator()->num_constraints());
    const Eigen::VectorXd& ub = linear_constraint.evaluator()->upper_bound();
    const Eigen::VectorXd& lb = linear_constraint.evaluator()->lower_bound();
    const VectorXDecisionVariable& x = linear_constraint.variables();
    const Eigen::SparseMatrix<double>& Ai =
        linear_constraint.evaluator()->get_sparse_A();
    // We store the starting row index in A_triplets for each row of
    // linear_constraint. Namely the constraint lb(i) <= A.row(i)*x <= ub(i) is
    // stored in A_triplets with starting_row_indices[i] (or
    // starting_row_indices[i]+1 if both lb(i) and ub(i) are finite).
    std::vector<int> starting_row_indices(
        linear_constraint.evaluator()->num_constraints());
    for (int i = 0; i < linear_constraint.evaluator()->num_constraints(); ++i) {
      const bool needs_ub{!std::isinf(ub(i))};
      const bool needs_lb{!std::isinf(lb(i))};
      auto& dual_index = linear_constraint_dual_indices->back()[i];
      // Use -1 to indicate the constraint bound is infinity.
      dual_index.first = -1;
      dual_index.second = -1;
      if (!needs_ub && !needs_lb) {
        // We use -1 to indicate that we won't add linear constraint when both
        // bounds are infinity.
        starting_row_indices[i] = -1;
      } else {
        starting_row_indices[i] = *A_row_count + *num_linear_constraint_rows;
        // We first add the constraint for lower bound, and then add the
        // constraint for upper bound. This is consistent with the loop below
        // when we modify A_triplets.
        if (needs_lb) {
          b->push_back(-lb(i));
          dual_index.first = *A_row_count + *num_linear_constraint_rows;
          (*num_linear_constraint_rows)++;
        }
        if (needs_ub) {
          b->push_back(ub(i));
          dual_index.second = *A_row_count + *num_linear_constraint_rows;
          (*num_linear_constraint_rows)++;
        }
      }
    }
    for (int j = 0; j < Ai.cols(); ++j) {
      const int xj_index = prog.FindDecisionVariableIndex(x(j));
      for (Eigen::SparseMatrix<double>::InnerIterator it(Ai, j); it; ++it) {
        const int Ai_row_count = it.row();
        const bool needs_ub{!std::isinf(ub(Ai_row_count))};
        const bool needs_lb{!std::isinf(lb(Ai_row_count))};
        if (!needs_lb && !needs_ub) {
          continue;
        }
        int row_index = starting_row_indices[Ai_row_count];
        if (needs_lb) {
          // If lb != -∞, then the constraint -aᵀx + s = lb will be added to
          // the matrix A, in the row row_index.
          A_triplets->emplace_back(row_index, xj_index, -it.value());
          ++row_index;
        }
        if (needs_ub) {
          // If ub != ∞, then the constraint aᵀx + s = ub will be added to the
          // matrix A, in the row row_index.
          A_triplets->emplace_back(row_index, xj_index, it.value());
        }
      }
    }
  }
  *A_row_count += *num_linear_constraint_rows;
}

void ParseQuadraticCosts(const MathematicalProgram& prog,
                         std::vector<Eigen::Triplet<double>>* P_upper_triplets,
                         std::vector<double>* c, double* constant) {
  for (const auto& cost : prog.quadratic_costs()) {
    const auto var_indices = prog.FindDecisionVariableIndices(cost.variables());
    for (int j = 0; j < cost.evaluator()->Q().cols(); ++j) {
      for (int i = 0; i <= j; ++i) {
        if (cost.evaluator()->Q()(i, j) != 0) {
          // Since we allow duplicated variables in a quadratic cost, we need to
          // handle this more carefully. If i != j but var_indices[i] ==
          // var_indices[j], then it means that the cost is a diagonal term
          // (Q(i, j) + Q(j, i)) * x[var_indices[i]]² = 2 * Q(i, j)*
          // x[var_indices[i]]², not a cross term (Q(i, j) + Q(j, i)) *
          // x[var_indices[i]] * x[var_indices[j]]. Hence we need a factor of 2
          // for this special case.
          const double factor =
              (i != j && var_indices[i] == var_indices[j]) ? 2 : 1;
          // Since we only add the upper diagonal entries, we need to branch
          // based on whether var_indices[i] > var_indices[j]
          int row_index = var_indices[i];
          int col_index = var_indices[j];
          if (var_indices[i] > var_indices[j]) {
            row_index = var_indices[j];
            col_index = var_indices[i];
          }
          P_upper_triplets->emplace_back(row_index, col_index,
                                         factor * cost.evaluator()->Q()(i, j));
        }
      }
      (*c)[var_indices[j]] += cost.evaluator()->b()(j);
    }
    *constant += cost.evaluator()->c();
  }
}

void ParseL2NormCosts(const MathematicalProgram& prog,
                      int* num_solver_variables,
                      std::vector<Eigen::Triplet<double>>* A_triplets,
                      std::vector<double>* b, int* A_row_count,
                      std::vector<int>* second_order_cone_length,
                      std::vector<int>* lorentz_cone_y_start_indices,
                      std::vector<double>* cost_coeffs,
                      std::vector<int>* t_slack_indices) {
  int t_slack_count = 0;
  for (const auto& cost : prog.l2norm_costs()) {
    const VectorXDecisionVariable& x = cost.variables();
    t_slack_indices->push_back(t_slack_count + *num_solver_variables);
    const Eigen::SparseMatrix<double>& C = cost.evaluator()->get_sparse_A();
    const auto& d = cost.evaluator()->b();
    // Add the constraint
    // [0   -1] * [x] + s = [0]
    // [-C   0]   [t]       [d]
    A_triplets->emplace_back(*A_row_count, t_slack_indices->back(), -1);
    b->push_back(0);
    for (int k = 0; k < C.outerSize(); ++k) {
      const int x_index = prog.FindDecisionVariableIndex(x(k));
      for (Eigen::SparseMatrix<double>::InnerIterator it(C, k); it; ++it) {
        A_triplets->emplace_back(*A_row_count + 1 + it.row(), x_index,
                                 -it.value());
      }
    }
    for (int i = 0; i < d.rows(); ++i) {
      b->push_back(d(i));
    }
    // Add the cost to minimize t
    cost_coeffs->push_back(1.0);

    second_order_cone_length->push_back(1 + cost.evaluator()->b().rows());
    lorentz_cone_y_start_indices->push_back(*A_row_count);

    *num_solver_variables += 1;
    *A_row_count += 1 + cost.evaluator()->b().rows();
  }
}

void ParseSecondOrderConeConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, std::vector<int>* second_order_cone_length,
    std::vector<int>* lorentz_cone_y_start_indices,
    std::vector<int>* rotated_lorentz_cone_y_start_indices) {
  // Our LorentzConeConstraint encodes that Ax + b is in Lorentz cone. We
  // convert this to SCS form, as -Ax + s = b, s in Lorentz cone.
  second_order_cone_length->reserve(
      prog.lorentz_cone_constraints().size() +
      prog.rotated_lorentz_cone_constraints().size());
  lorentz_cone_y_start_indices->reserve(prog.lorentz_cone_constraints().size());
  rotated_lorentz_cone_y_start_indices->reserve(
      prog.rotated_lorentz_cone_constraints().size());
  for (const auto& lorentz_cone_constraint : prog.lorentz_cone_constraints()) {
    // x_indices[i] is the index of x(i)
    const VectorXDecisionVariable& x = lorentz_cone_constraint.variables();
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);
    const Eigen::SparseMatrix<double>& Ai =
        lorentz_cone_constraint.evaluator()->A();
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(Ai);
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(Ai_triplet.row() + *A_row_count,
                               x_indices[Ai_triplet.col()],
                               -Ai_triplet.value());
    }
    const int num_Ai_rows = lorentz_cone_constraint.evaluator()->A().rows();
    for (int i = 0; i < num_Ai_rows; ++i) {
      b->push_back(lorentz_cone_constraint.evaluator()->b()(i));
    }
    second_order_cone_length->push_back(num_Ai_rows);
    lorentz_cone_y_start_indices->push_back(*A_row_count);
    *A_row_count += num_Ai_rows;
  }

  for (const auto& rotated_lorentz_cone :
       prog.rotated_lorentz_cone_constraints()) {
    const VectorXDecisionVariable& x = rotated_lorentz_cone.variables();
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);
    const Eigen::SparseMatrix<double> Ai =
        rotated_lorentz_cone.evaluator()->A();
    const Eigen::VectorXd& bi = rotated_lorentz_cone.evaluator()->b();
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(Ai);
    ParseRotatedLorentzConeConstraint(Ai_triplets, bi, x_indices, A_triplets, b,
                                      A_row_count, second_order_cone_length,
                                      rotated_lorentz_cone_y_start_indices);
  }
}

void ParseRotatedLorentzConeConstraint(
    const std::vector<Eigen::Triplet<double>>& A_cone_triplets,
    const Eigen::Ref<const Eigen::VectorXd>& b_cone,
    const std::vector<int>& x_indices,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, std::vector<int>* second_order_cone_length,
    std::optional<std::vector<int>*> rotated_lorentz_cone_y_start_indices) {
  // Our RotatedLorentzConeConstraint encodes that Ax + b is in the rotated
  // Lorentz cone, namely
  //  (a₀ᵀx + b₀) (a₁ᵀx + b₁) ≥ (a₂ᵀx + b₂)² + ... + (aₙ₋₁ᵀx + bₙ₋₁)²
  //  (a₀ᵀx + b₀) ≥ 0
  //  (a₁ᵀx + b₁) ≥ 0
  // , where aᵢᵀ is the i'th row of A, bᵢ is the i'th row of b. Equivalently the
  // vector
  // [ 0.5(a₀ + a₁)ᵀx + 0.5(b₀ + b₁) ]
  // [ 0.5(a₀ - a₁)ᵀx + 0.5(b₀ - b₁) ]
  // [           a₂ᵀx +           b₂ ]
  //             ...
  // [         aₙ₋₁ᵀx +         bₙ₋₁ ]
  // is in the Lorentz cone. We convert this to the SCS form, that
  //  Cx + s = d
  //  s in Lorentz cone,
  // where C = [ -0.5(a₀ + a₁)ᵀ ]   d = [ 0.5(b₀ + b₁) ]
  //           [ -0.5(a₀ - a₁)ᵀ ]       [ 0.5(b₀ - b₁) ]
  //           [           -a₂ᵀ ]       [           b₂ ]
  //                 ...                      ...
  //           [         -aₙ₋₁ᵀ ]       [          bₙ₋₁]
  for (const auto& Ai_triplet : A_cone_triplets) {
    const int x_index = x_indices[Ai_triplet.col()];
    if (Ai_triplet.row() == 0) {
      A_triplets->emplace_back(*A_row_count, x_index,
                               -0.5 * Ai_triplet.value());
      A_triplets->emplace_back(*A_row_count + 1, x_index,
                               -0.5 * Ai_triplet.value());
    } else if (Ai_triplet.row() == 1) {
      A_triplets->emplace_back(*A_row_count, x_index,
                               -0.5 * Ai_triplet.value());
      A_triplets->emplace_back(*A_row_count + 1, x_index,
                               0.5 * Ai_triplet.value());
    } else {
      A_triplets->emplace_back(*A_row_count + Ai_triplet.row(), x_index,
                               -Ai_triplet.value());
    }
  }
  b->push_back(0.5 * (b_cone(0) + b_cone(1)));
  b->push_back(0.5 * (b_cone(0) - b_cone(1)));
  for (int i = 2; i < b_cone.rows(); ++i) {
    b->push_back(b_cone(i));
  }
  if (rotated_lorentz_cone_y_start_indices.has_value()) {
    rotated_lorentz_cone_y_start_indices.value()->push_back(*A_row_count);
  }
  *A_row_count += b_cone.rows();
  second_order_cone_length->push_back(b_cone.rows());
}

void ParseExponentialConeConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count) {
  for (const auto& binding : prog.exponential_cone_constraints()) {
    // drake::solvers::ExponentialConstraint enforces that z = A * x + b is in
    // the exponential cone (z₀ ≥ z₁*exp(z₂ / z₁)). This is different from the
    // exponential cone used in SCS. In SCS, a vector s is in the exponential
    // cone, if s₂≥ s₁*exp(s₀ / s₁). To transform drake's Exponential cone to
    // SCS's exponential cone, we use
    // -[A.row(2); A.row(1); A.row(0)] * x + s = [b(2); b(1); b(0)], and s is
    // in SCS's exponential cone, where A = binding.evaluator()->A(), b =
    // binding.evaluator()->b().
    const int num_bound_variables = binding.variables().rows();
    for (int i = 0; i < num_bound_variables; ++i) {
      A_triplets->reserve(A_triplets->size() +
                          binding.evaluator()->A().nonZeros());
      const int decision_variable_index =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               binding.evaluator()->A(), i);
           it; ++it) {
        // 2 - it.row() is used for reverse the row order, as mentioned in the
        // function documentation above.
        A_triplets->emplace_back(*A_row_count + 2 - it.row(),
                                 decision_variable_index, -it.value());
      }
    }
    // The exponential cone constraint is on a 3 x 1 vector, hence for each
    // ExponentialConeConstraint, we append 3 rows to A and b.
    b->reserve(b->size() + 3);
    for (int i = 0; i < 3; ++i) {
      b->push_back(binding.evaluator()->b()(2 - i));
    }
    *A_row_count += 3;
  }
}

void ParsePositiveSemidefiniteConstraints(
    const MathematicalProgram& prog, bool upper_triangular,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, std::vector<std::optional<int>>* psd_cone_length,
    std::vector<std::optional<int>>* lmi_cone_length,
    std::vector<std::optional<int>>* psd_y_start_indices,
    std::vector<std::optional<int>>* lmi_y_start_indices) {
  DRAKE_ASSERT(ssize(*b) == *A_row_count);
  DRAKE_ASSERT(psd_cone_length != nullptr);
  // Make sure that each triplet in A_triplets has row smaller than
  // *A_row_count.
  // Use kDrakeAssertIsArmed to bypass the entire for loop in the release mode.
  if (kDrakeAssertIsArmed) {
    for (const auto& A_triplet : *A_triplets) {
      DRAKE_DEMAND(A_triplet.row() < *A_row_count);
    }
  }
  DRAKE_ASSERT(psd_cone_length->empty());
  DRAKE_ASSERT(lmi_cone_length->empty());
  DRAKE_ASSERT(psd_y_start_indices->empty());
  DRAKE_ASSERT(lmi_y_start_indices->empty());
  const double sqrt2 = std::sqrt(2);
  for (const auto& psd_constraint : prog.positive_semidefinite_constraints()) {
    if (psd_constraint.evaluator()->matrix_rows() > 2) {
      // PositiveSemidefiniteConstraint encodes the matrix X being psd.
      // We convert it to SCS/Clarabel form
      // A * x + s = 0
      // s in positive semidefinite cone.
      // where A is a diagonal matrix, with its diagonal entries being the
      // stacked column vector of the lower/upper triangular part of matrix
      // ⎡ -1 -√2 -√2 ... -√2⎤
      // ⎢-√2  -1 -√2 ... -√2⎥
      // ⎢-√2 -√2  -1 ... -√2⎥
      // ⎢    ...            ⎥
      // ⎣-√2 -√2 -√2 ...  -1⎦
      // The √2 scaling factor in the off-diagonal entries are required by SCS
      // and Clarabel, as it uses only the lower triangular part (for SCS), or
      // upper triangular part (for Clarabel) of the symmetric matrix, as
      // explained in
      // https://www.cvxgrp.org/scs/api/cones.html#semidefinite-cones and
      // https://oxfordcontrol.github.io/ClarabelDocs/stable/examples/example_sdp.
      // x is the stacked column vector of the lower triangular part of the
      // symmetric matrix X.
      const int X_rows = psd_constraint.evaluator()->matrix_rows();
      int x_index_count = 0;
      // The variables in the psd constraint are the stacked columns of the
      // matrix X (in column major order).
      const VectorXDecisionVariable& flat_X = psd_constraint.variables();
      DRAKE_DEMAND(flat_X.rows() == X_rows * X_rows);
      b->reserve(b->size() + X_rows * (X_rows + 1) / 2);
      for (int j = 0; j < X_rows; ++j) {
        const int i_start = upper_triangular ? 0 : j;
        const int i_end = upper_triangular ? j + 1 : X_rows;
        for (int i = i_start; i < i_end; ++i) {
          const double scale_factor = i == j ? 1 : sqrt2;
          A_triplets->emplace_back(
              *A_row_count + x_index_count,
              prog.FindDecisionVariableIndex(flat_X(j * X_rows + i)),
              -scale_factor);
          b->push_back(0);
          ++x_index_count;
        }
      }
      psd_cone_length->push_back(X_rows);
      psd_y_start_indices->push_back(*A_row_count);
      (*A_row_count) += X_rows * (X_rows + 1) / 2;
    } else {
      psd_cone_length->push_back(std::nullopt);
      psd_y_start_indices->push_back(std::nullopt);
    }
  }
  for (const auto& lmi_constraint :
       prog.linear_matrix_inequality_constraints()) {
    if (lmi_constraint.evaluator()->matrix_rows() > 2) {
      // LinearMatrixInequalityConstraint encodes
      // F₀ + x₁*F₁ + x₂*F₂ + ... + xₙFₙ is p.s.d
      // We convert this to SCS/Clarabel form as
      // A_cone * x + s = b_cone
      // s in SCS/Clarabel positive semidefinite cone.
      // For SCS, it uses the lower triangular of the symmetric psd matrix,
      // hence
      //              ⎡  F₁(0, 0)   F₂(0, 0) ...   Fₙ(0, 0)⎤
      //              ⎢√2F₁(1, 0) √2F₂(1, 0) ... √2Fₙ(1, 0)⎥
      //   A_cone = - ⎢√2F₁(2, 0) √2F₂(2, 0) ... √2Fₙ(2, 0)⎥,
      //              ⎢            ...                     ⎥
      //              ⎣  F₁(m, m)   F₂(m, m) ...   Fₙ(m, m)⎦
      //
      //              ⎡  F₀(0, 0)⎤
      //              ⎢√2F₀(1, 0)⎥
      //   b_cone =   ⎢√2F₀(2, 0)⎥,
      //              ⎢   ...    ⎥
      //              ⎣  F₀(m, m)⎦
      // For Clarabel, it uses the upper triangular of the symmetric psd matrix,
      // hence
      //              ⎡  F₁(0, 0)   F₂(0, 0) ...   Fₙ(0, 0)⎤
      //              ⎢√2F₁(0, 1) √2F₂(0, 1) ... √2Fₙ(0, 1)⎥
      //   A_cone = - ⎢√2F₁(1, 1) √2F₂(1, 1) ... √2Fₙ(1, 1)⎥,
      //              ⎢            ...                     ⎥
      //              ⎣  F₁(m, m)   F₂(m, m) ...   Fₙ(m, m)⎦
      //
      //              ⎡  F₀(0, 0)⎤
      //              ⎢√2F₀(0, 1)⎥
      //   b_cone =   ⎢√2F₀(1, 1)⎥,
      //              ⎢   ...    ⎥
      //              ⎣  F₀(m, m)⎦
      // For both SCS and Clarabel, we have
      //   x = [x₁; x₂; ... ; xₙ].
      // As explained above, the off-diagonal rows are scaled by √2. Please
      // refer to https://github.com/cvxgrp/scs about the scaling factor √2.
      // Note that since all F matrices are symmetric, we don't need to
      // differentiate between lower triangular or upper triangular version.
      const std::vector<Eigen::MatrixXd>& F = lmi_constraint.evaluator()->F();
      const VectorXDecisionVariable& x = lmi_constraint.variables();
      const int F_rows = lmi_constraint.evaluator()->matrix_rows();
      const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);
      int A_cone_row_count = 0;
      b->reserve(b->size() + F_rows * (F_rows + 1) / 2);
      for (int j = 0; j < F_rows; ++j) {
        const int i_start = upper_triangular ? 0 : j;
        const int i_end = upper_triangular ? j + 1 : F_rows;
        for (int i = i_start; i < i_end; ++i) {
          const double scale_factor = i == j ? 1 : sqrt2;
          for (int k = 1; k < static_cast<int>(F.size()); ++k) {
            EmplaceNonzeroTriplet(*A_row_count + A_cone_row_count,
                                  x_indices[k - 1], -scale_factor * F[k](i, j),
                                  A_triplets);
          }
          b->push_back(scale_factor * F[0](i, j));
          ++A_cone_row_count;
        }
      }
      lmi_cone_length->push_back(F_rows);
      lmi_y_start_indices->push_back(*A_row_count);
      *A_row_count += F_rows * (F_rows + 1) / 2;
    } else {
      lmi_cone_length->push_back(std::nullopt);
      lmi_y_start_indices->push_back(std::nullopt);
    }
  }
}

void ParseScalarPositiveSemidefiniteConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, int* new_positive_cone_length,
    std::vector<std::optional<int>>* scalar_psd_dual_indices,
    std::vector<std::optional<int>>* scalar_lmi_dual_indices) {
  DRAKE_ASSERT(ssize(*b) == *A_row_count);
  DRAKE_ASSERT(new_positive_cone_length != nullptr);
  *new_positive_cone_length = 0;
  // Make sure that each triplet in A_triplets has row smaller than
  // *A_row_count.
  // Use kDrakeAssertIsArmed to bypass the entire for loop in the release mode.
  if (kDrakeAssertIsArmed) {
    for (const auto& A_triplet : *A_triplets) {
      DRAKE_DEMAND(A_triplet.row() < *A_row_count);
    }
  }
  scalar_psd_dual_indices->reserve(
      prog.positive_semidefinite_constraints().size());
  for (const auto& psd_constraint : prog.positive_semidefinite_constraints()) {
    if (psd_constraint.evaluator()->matrix_rows() == 1) {
      // Impose the constraint -x + s = 0, s in positive-orthant cone.
      A_triplets->emplace_back(
          *A_row_count,
          prog.FindDecisionVariableIndex(psd_constraint.variables()[0]), -1);
      b->push_back(0);
      (*new_positive_cone_length) += 1;
      scalar_psd_dual_indices->push_back(*A_row_count);
      *A_row_count += 1;
    } else {
      scalar_psd_dual_indices->push_back(std::nullopt);
    }
  }
  scalar_lmi_dual_indices->reserve(
      prog.linear_matrix_inequality_constraints().size());
  for (const auto& lmi_constraint :
       prog.linear_matrix_inequality_constraints()) {
    if (lmi_constraint.evaluator()->matrix_rows() == 1) {
      // Impose the constraint
      // -x₁*F₁[0, 0] - x₂*F₂[0, 0] - ... - xₙFₙ[0, 0] + s  = F₀[0, 0]
      // s in positive orthant cone.
      for (int i = 0; i < ssize(lmi_constraint.evaluator()->F()) - 1; ++i) {
        EmplaceNonzeroTriplet(
            *A_row_count,
            prog.FindDecisionVariableIndex(lmi_constraint.variables()[i]),
            -lmi_constraint.evaluator()->F()[i + 1](0, 0), A_triplets);
      }
      b->push_back(lmi_constraint.evaluator()->F()[0](0, 0));
      (*new_positive_cone_length) += 1;
      scalar_lmi_dual_indices->push_back(*A_row_count);
      (*A_row_count) += 1;
    } else {
      scalar_lmi_dual_indices->push_back(std::nullopt);
    }
  }
}

void Parse2x2PositiveSemidefiniteConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, int* num_new_second_order_cones,
    std::vector<std::optional<int>>* twobytwo_psd_dual_start_indices,
    std::vector<std::optional<int>>* twobytwo_lmi_dual_start_indices) {
  DRAKE_ASSERT(ssize(*b) == *A_row_count);
  DRAKE_ASSERT(num_new_second_order_cones != nullptr);
  *num_new_second_order_cones = 0;
  // Make sure that each triplet in A_triplets has row smaller than
  // *A_row_count.
  // Use kDrakeAssertIsArmed to bypass the entire for loop in the release mode.
  if (kDrakeAssertIsArmed) {
    for (const auto& A_triplet : *A_triplets) {
      DRAKE_DEMAND(A_triplet.row() < *A_row_count);
    }
  }
  twobytwo_psd_dual_start_indices->reserve(
      prog.positive_semidefinite_constraints().size());
  twobytwo_lmi_dual_start_indices->reserve(
      prog.linear_matrix_inequality_constraints().size());
  for (const auto& psd_constraint : prog.positive_semidefinite_constraints()) {
    if (psd_constraint.evaluator()->matrix_rows() == 2) {
      // The PSD constraint imposes
      // [x(0) x(2)] is psd.
      // [x(1) x(3)]
      // where (x(0), x(1), x(2), x(3)) is psd_constraint.variables() (Note
      // that x(1) and x(2) are the same).
      // This is equivalent to
      // [x(0) + x(3)]
      // [x(0) - x(3)]  in lorentz cone.
      // [  2*x(1)   ]
      // Namely
      // [-x(0) - x(3) + s(0)]   [0]
      // [-x(0) + x(3) + s(1)] = [0]
      // [ -2 * x(1) + s(2)  ]   [0]
      // s in second order cone.
      const int x0_index =
          prog.FindDecisionVariableIndex(psd_constraint.variables()[0]);
      const int x1_index =
          prog.FindDecisionVariableIndex(psd_constraint.variables()[1]);
      const int x3_index =
          prog.FindDecisionVariableIndex(psd_constraint.variables()[3]);
      A_triplets->emplace_back(*A_row_count, x0_index, -1);
      A_triplets->emplace_back(*A_row_count, x3_index, -1);
      A_triplets->emplace_back(*A_row_count + 1, x0_index, -1);
      A_triplets->emplace_back(*A_row_count + 1, x3_index, 1);
      A_triplets->emplace_back(*A_row_count + 2, x1_index, -2);
      b->push_back(0);
      b->push_back(0);
      b->push_back(0);
      ++(*num_new_second_order_cones);
      twobytwo_psd_dual_start_indices->push_back(*A_row_count);
      *A_row_count += 3;
    } else {
      twobytwo_psd_dual_start_indices->push_back(std::nullopt);
    }
  }
  for (const auto& lmi_constraint :
       prog.linear_matrix_inequality_constraints()) {
    if (lmi_constraint.evaluator()->matrix_rows() == 2) {
      // The constraint imposes
      // F[0] + ∑ᵢF[1+i] * x(i) is psd.
      // This is equivalent to
      // [F[0](0, 0) + F[0](1, 1) + ∑ᵢ(F[1+i](0, 0) + F[1+i)(1, 1)) * x[i]]
      // [F[0](0, 0) - F[0](1, 1) + ∑ᵢ(F[1+i](0, 0) - F[1+i)(1, 1)) * x[i]]
      // [     2 * F[0](0, 1) + 2 * ∑ᵢF[1+i](0, 1)*x[i]                   ]
      // is in Lorentz cone.
      // Writing it in SCS/Clarabel format
      // -∑ᵢ(F[1+i](0,0) + F[1+i](1,1))x[i] + s[0] = F[0](0, 0) + F[0](1, 1)
      // -∑ᵢ(F[1+i](0,0) - F[1+i](1,1))x[i] + s[1] = F[0](0, 0) - F[0](1, 1)
      //             -2*∑ᵢF[1+i](0, 1)*x[i] + s[2] = 2 * F[0](0, 1)
      //  s in second order cone.
      const auto& F = lmi_constraint.evaluator()->F();
      for (int i = 0; i < lmi_constraint.variables().rows(); ++i) {
        const int var_index =
            prog.FindDecisionVariableIndex(lmi_constraint.variables()(i));
        EmplaceNonzeroTriplet(*A_row_count, var_index,
                              -F[1 + i](0, 0) - F[1 + i](1, 1), A_triplets);
        EmplaceNonzeroTriplet(*A_row_count + 1, var_index,
                              -F[1 + i](0, 0) + F[1 + i](1, 1), A_triplets);
        EmplaceNonzeroTriplet(*A_row_count + 2, var_index, -2 * F[1 + i](0, 1),
                              A_triplets);
      }
      b->push_back(F[0](0, 0) + F[0](1, 1));
      b->push_back(F[0](0, 0) - F[0](1, 1));
      b->push_back(2 * F[0](0, 1));
      ++(*num_new_second_order_cones);
      twobytwo_lmi_dual_start_indices->push_back(*A_row_count);
      *A_row_count += 3;
    } else {
      twobytwo_lmi_dual_start_indices->push_back(std::nullopt);
    }
  }
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
