#pragma once

#include <list>
#include <memory>

#include "drake/solvers/mathematical_program.h"

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

  enum class OptimalSolutionIsIntegral {
    kTrue,   ///< The program in this node has been solved, and the solution to
             ///all binary variables satisfies the integral constraints.
    kFalse,  ///< The program in this node has been solved, and the solution to
             ///some binary variables does not satisfy the integral constraints.
    kUnknown,  ///< Either the program in this node has not been solved, or we
               ///have not checked if the solution satisfy the integral
               ///constraints yet.
  };

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
   * @retval (node, map_old_vars_to_new_vars) node is the root node of the tree,
   * that contains the optimization program (2) in the documentation above. This
   * root node has no parent. We also need to recreate new decision variables in
   * the root node, from the original optimization program (1), since the binary
   * variables will be converted to continuous variables in (2). We thus return
   * the map from the old variable to the new variable.
   */
  static std::pair<
      std::unique_ptr<MixedIntegerBranchAndBoundNode>,
      std::unordered_map<symbolic::Variable::Id, symbolic::Variable>>
  ConstructRootNode(const MathematicalProgram& prog);

  /** Returns if a node is the root.
   * A root node has no parent.
   */
  bool IsRoot() const;

  /** Determine if a node is a leaf or not.
   * A leaf node has no child nodes.
   */
  bool IsLeaf() const { return !left_child_ && !right_child_; }

  /**
   * Getter for the mathematical program.
   */
  MathematicalProgram* prog() const { return prog_.get(); }

  /**
   * Getter for the left child.
   */
  MixedIntegerBranchAndBoundNode* left_child() const {
    return left_child_.get();
  }

  /**
   * Getter for the right child.
   */
  MixedIntegerBranchAndBoundNode* right_child() const {
    return right_child_.get();
  }

  /**
   * Getter for the parent node.
   */
  MixedIntegerBranchAndBoundNode* parent() const { return parent_.get(); }

  /**
   * Getter for the index of the binary variable, whose value was not fixed in
   * the parent node, but is fixed to either 0 or 1 in this node.
   */
  int binary_var_index() const { return binary_var_index_; }

  /**
   * Getter for the value of the binary variable, which was not fixed in the
   * parent node, but is fixed to either 0 or 1 in this node.
   */
  int binary_var_value() const { return binary_var_value_; }

  /**
   * Getter for the remaining binary variables in this node.
   */
  const std::list<symbolic::Variable>& remaining_binary_variables() const {
    return remaining_binary_variables_;
  }

  /** Check if the optimal solution to the program in this node satisfies all
   * integral constraints.
   * Only call this function AFTER the program is solved.
   */
  bool IsOptimalSolutionIntegral();

 private:
  /**
   * Constructs an empty node. Clone the input mathematical program to this
   * node. The child and the parent nodes are all nullptr.
   * @param prog The optimization program whose binary variable constraints are
   * all relaxed to 0 ≤ y ≤ 1.
   * @param binary_variables The list of binary variables in the mixed-integer
   * problem.
   */
  MixedIntegerBranchAndBoundNode(
      const MathematicalProgram& prog,
      const Eigen::Ref<const VectorXDecisionVariable>& binary_variables);

  std::unique_ptr<MathematicalProgram>
      prog_;  // Stores the optimization program in this node.
  std::unique_ptr<MixedIntegerBranchAndBoundNode> left_child_;
  std::unique_ptr<MixedIntegerBranchAndBoundNode> right_child_;
  std::unique_ptr<MixedIntegerBranchAndBoundNode> parent_;

  // The index of the newly fixed binary variable z, in the decision variables
  // x. The value of z was not fixed in the parent node, but is fixed in this
  // node.
  int binary_var_index_;
  // The value of the newly fixed binary variable z, in the decision variables
  // x. The value of z was not fixed in the parent node, but is fixed in this
  // node.
  int binary_var_value_;

  // The variables that were binary in the original mixed-integer optimization
  // problem, but whose value has not been fixed to either 0 or 1 yet.
  std::list<symbolic::Variable> remaining_binary_variables_;

  // Whether the optimal solution in this node satisfies all integral
  // constraints.
  OptimalSolutionIsIntegral optimal_solution_is_integral_;
};
}  // namespace solvers
}  // namespace drake
