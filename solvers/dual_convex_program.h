#pragma once

#include <memory>
#include <unordered_map>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

/** Compute a dual of the convex program. In particular, given the convex
 * program
 * min c^T x + d s.t.
 * A x + b ∈ K
 * Aeq x = beq
 * where K is the product of the supported cones.
 * We return the dual program
 * max -beqᵀy + d - bᵀλ s.t.
 * c - Aeqᵀy - Aᵀλ = 0
 * λ ∈ K*
 * where K* is the dual cone of the supported cones. Note that
 * max -beqᵀy + d - bᵀλ is encoded as min beqᵀy - d + bᵀλ and so if the primal
 * and dual are feasible we expect that
 * primal_result.get_optimal_cost() + dual_result.get_optimal_cost() = 0.
 *
 * The supported cones are
 * 1) The positive orthant which is self-dual
 * 2) The lorentz cone which is self-dual
 * 3) The rotated lorentz cone. Drake's rotated lorentz cone is not self-dual,
 * so λ ∈ (2, 2, I) K where K is the rotated lorentz cone.
 * 4) The positive semidefinite cone which is self-dual. Note  that for λ ∈ the
 * positive semidefinite cone, we take the convention that λ corresponds to the
 * lower triangular elements of the symmetric matrix Λ. To obtain the matrix Λ
 * from the corresponding matrix λ, use the function
 * math::ToSymmetricMatrixFromLowerTriangularColumns.
 *
 * [out] constraint_to_dual_variable_map Maps from the constraints of the primal
 * program to the corresponding dual variables in the dual program. For PSD
 * constraints, the mapped to variable will be a symmetric matrix. For all other
 * constraint, it will map to a vector. If dual_result is the
 * MathematicalProgramResult of the returned dual prog of this function then
 * calling dual_result.GetSolution(constraint_to_dual_variable_map.at(binding))
 * will return the value of the dual variable associated to the primal
 * constraint in binding.
 *
 * If the primal program is solved with conic solver (e.g. Mosek, Clarabel, SCS,
 * but not Gurobi) and the result is primal_result, then generally
 * dual_result.GetSolution(constraint_to_dual_variable_map.at(binding)) will be
 * equal (up to numerical precision) to primal_result.GetDualSolution(binding).
 * The only except is if binding is a linear equality constraint in which case
 * primal_result.GetDualSolution(binding) =
 * -dual_result.GetSolution(constraint_to_dual_variable_map.at(binding)).
 * This is because Drake's GetDualSolution adopts the shadow price
 * interpretation for linear equality constraints, which is the negative of the
 * dual formulation provided by this method.s
 */
std::unique_ptr<MathematicalProgram> CreateDualConvexProgram(
    const MathematicalProgram& prog,
    std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>*
        constraint_to_dual_variable_map);

}  // namespace solvers
}  // namespace drake
