#pragma once

#include <unordered_map>
#include <memory>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

// Compute a dual of the convex program. In particular, given the convex program
// min c^T x + d
// s.t. A x + b ∈ K where K is the product of cones.
//      Aeq x = beq
// We return the dual program
// max beqᵀy + d - bᵀλ
// s.t. c-Aeqᵀy - Aᵀλ = 0
// λ ∈ K*
// TODO(Alexandre.Amice) also return a map from constraint to dual variables.
std::unique_ptr<MathematicalProgram> CreateDualConvexProgram(
    const MathematicalProgram& prog,
    std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>*
        constraint_to_dual_variable_map);

}  // namespace solvers
}  // namespace drake
