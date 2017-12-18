#pragma once

#include <memory>

#include "solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/**
 * A node in the branch-and-bound (bnb) tree.
 * The whole branch-and-bound tree solves the mixed-integer problem
 * min f(x)         (1)
 * s.t g(x) ≤ 0
 *     y ∈ {0, 1}
 * where the binary variables y are a subset of the decision variables x.
 * In this node, we will fix some binary variables to either 0 and 1, and relax
 * the rest of the binary variables to continuous variables between 0 and 1.
 * Namely we will solve the following problem with all variables being
 * continuous
 * min f(x)         (2)
 * s.t g(x) ≤ 0
 *     y₀ = b₀
 *     0 ≤ y₁ ≤ 1
 * where y₀, y₁ is a partition of the original binary variables y. y₀ is the
 * fixed binary variables, y₁ is the relaxed binary variables. b₀ is a vector
 * containing the assigned values of the fixed binary variables y₀, b₀ only
 * contains value either 0 or 1.
 *
 * Each node is created from its parent node, by fixing one binary variable to
 * either 0 or 1. We denote this special binary variable as z.
 */
class MixedIntegerBranchAndBoundNode {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MixedIntegerBranchAndBoundNode)


  /** Construct the root node from an optimization program.
   * For the mixed-integer optimization program
   * min f(x)         (1)
   * s.t g(x) ≤ 0
   *     y ∈ {0, 1}
   * we will construct a root node for this program. In the root node, it
   * enforces all the costs and constraints in the original program, except
   * the binary constraint y ∈ {0, 1}. Instead, we will relax the binary
   * constraint to 0 ≤ y ≤ 1. So the root node contains the program
   * min f(x)         (2)
   * s.t g(x) ≤ 0
   *     0 ≤ y ≤ 1
   * @param prog The mixed-integer optimization program (1) in the
   * documentation above.
   * @retval node The root node of the tree, that contains the optimization 
   * program (2) in the documentation above. This root node has no parent.
   */
  static MixedIntegerBranchAndBoundNode ConstructRootNode(const MathematicalProgram& prog);

 private:
  /**
   * Construct an empty node
   */
  MixedIntegerBranchAndBoundNode();
  MathematicalProgram prog_; // Stores the optimization program in this node.
  std::shared_ptr<MixedIntegerBranchAndBoundNode> left_child_;
  std::shared_ptr<MixedIntegerBranchAndBoundNode> right_child_;
  std::weak_ptr<MixedIntegerBranchAndBoundNode> parent_;

  // The index of the newly fixed binary variable z, in the decision variables x.
  int binary_var_index_;
  // The value of the newly fixed binary variable z, in the decision variables x.
  int binary_var_value_;

  // In each node, we will need to construct
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable::Id> original_var_to_node_var_map_;
};
}  // namespace solvers
}  // namespace drake
