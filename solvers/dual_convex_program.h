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
 * max beqᵀy + d - bᵀλ s.t.
 * c-Aeqᵀy - Aᵀλ = 0
 * λ ∈ K*
 * where K* is the dual cone of the supported cones. Note that
 * max beqᵀy + d - bᵀλ is encoded as min -beqᵀy - d + bᵀλ and so if the primal
 * and dual are feasible we expect that
 * primal_result.get_optimal_cost() + dual_result.get_optimal_cost() = 0.
 *
 * The supported cones are
 * 1) The positive orthant.
 * 2) The lorentz cone.
 * 3) The rotated lorentz cone.
 * 4) The positive semidefinite cone.
 * All these cones are self-dual and therefore K = K*. Note that for λ ∈ the
 * positive semidefinite cone, we take the convention that λ corresponds to the
 * lower triangular elements of the symmetric matrix Λ. To obtain the matrix Λ
 * from the corresponding matrix λ, use the function
 * math::ToSymmetricMatrixFromLowerTriangularColumns.
 * [out] constraint_to_dual_variable_map Maps from the constraints of the primal
 * program to the corresponding dual variables in the dual program. For PSD
 * constraints, the mapped to variable will be a symmetric matrix. For all other
 * constraint, it will map to a vector. If dual_result is the
 * MathematicalProgram result of the returned dual prog of this function then
 * calling dual_result.GetSolution(constraint_to_dual_variable_map.at(binding))
 * will return  value of the dual variable associated to the primal constraint
 * in binding.
 */
std::unique_ptr<MathematicalProgram> CreateDualConvexProgram(
    const MathematicalProgram& prog,
    std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>*
        constraint_to_dual_variable_map);

}  // namespace solvers
}  // namespace drake
