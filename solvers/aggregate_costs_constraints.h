#pragma once
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/** Given many linear costs, aggregate them into
 *
 *     aᵀ*x + b,
 * @param linear_costs the linear costs to be aggregated.
 * @param linear_coeff[out] a in the documentation above.
 * @param vars[out] x in the documentation above.
 * @param constant_cost[out] b in the documentation above.
 */
void AggregateLinearCosts(const std::vector<Binding<LinearCost>>& linear_costs,
                          Eigen::SparseVector<double>* linear_coeff,
                          VectorX<symbolic::Variable>* vars,
                          double* constant_cost);

/** Given many linear and quadratic costs, aggregate them into
 *
 *     0.5*x₁ᵀQx₁ + bᵀx₂ + c
 * where x₁ and x₂ don't need to be the same.
 * @param quadratic_costs The quadratic costs to be aggregated.
 * @param linear_costs The linear costs to be aggregated.
 * @param Q_lower[out] The lower triangular part of the matrix Q.
 * @param quadratic_vars[out] x₁ in the documentation above.
 * @param linear_coeff[out] b in the documentation above.
 * @param linear_vars[out] x₂ in the documentation above.
 * @param constant_cost[out] c in the documentation above.
 */
void AggregateQuadraticAndLinearCosts(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs,
    const std::vector<Binding<LinearCost>>& linear_costs,
    Eigen::SparseMatrix<double>* Q_lower,
    VectorX<symbolic::Variable>* quadratic_vars,
    Eigen::SparseVector<double>* linear_coeff,
    VectorX<symbolic::Variable>* linear_vars, double* constant_cost);

/**
 * Stores the lower and upper bound of a variable.
 */
struct Bound {
  /** Lower bound. */
  double lower{};
  /** Upper bound. */
  double upper{};
};

/**
 * Aggregates many bounding box constraints, returns the intersection (the
 * tightest bounds) of these constraints.
 * @param bounding_box_constraints The constraints to be aggregated.
 * @retval aggregated_bounds aggregated_bounds[var.get_id()] returns the
 * (lower, upper) bounds of that variable as the tightest bounds of @p
 * bounding_box_constraints.
 */
std::unordered_map<symbolic::Variable, Bound> AggregateBoundingBoxConstraints(
    const std::vector<Binding<BoundingBoxConstraint>>&
        bounding_box_constraints);

/**
 * Aggregates all the BoundingBoxConstraints inside @p prog, returns the
 * intersection of the bounding box constraints as the lower and upper bound for
 * each variable in @p prog.
 * @param prog The optimization program containing decision variables and
 * BoundingBoxConstraints.
 * @param[out] lower (*lower)[i] is the lower bound of
 * prog.decision_variable(i).
 * @param[out] upper (*upper)[i] is the upper bound of
 * prog.decision_variable(i).
 */
void AggregateBoundingBoxConstraints(const MathematicalProgram& prog,
                                     Eigen::VectorXd* lower,
                                     Eigen::VectorXd* upper);

/**
 * Returns the first non-convex quadratic cost among @p quadratic_costs. If all
 * quadratic costs are convex, then return a nullptr.
 */
const Binding<QuadraticCost>* FindNonconvexQuadraticCost(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs);

namespace internal {
// If the program is compatible with this solver (the solver meets the required
// capabilities of the program, and the program is convex), returns true and
// clears the explanation.  Otherwise, returns false and sets the explanation.
// In either case, the explanation can be nullptr in which case it is ignored.
bool CheckConvexSolverAttributes(const MathematicalProgram& prog,
                                 const ProgramAttributes& solver_capabilities,
                                 std::string_view solver_name,
                                 std::string* explanation);
}  // namespace internal
}  // namespace solvers
}  // namespace drake
