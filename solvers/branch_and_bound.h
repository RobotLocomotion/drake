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
   * @pre prog should contain binary variables.
   * @throw std::runtime_error if the preconditions are not met.
   */
  static std::pair<
      std::unique_ptr<MixedIntegerBranchAndBoundNode>,
      std::unordered_map<symbolic::Variable::Id, symbolic::Variable>>
  ConstructRootNode(const MathematicalProgram& prog);

  /**
   * Branches on a binary variable, and creates two child nodes. In the left
   * child node, the binary variable is fixed to 0. In the right node, the
   * binary variable is fixed to 1.
   * @param binary_variable This binary variable is fixed to either 0 or 1 in
   * the child node.
   * @pre binary_variable is in remaining_binary_variables_;
   * @throw std::runtime_error if the preconditions are not met.
   */
  void Branch(const symbolic::Variable& binary_variable);

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
  MixedIntegerBranchAndBoundNode* parent() const { return parent_; }

  /**
   * Getter for the binary variable, whose value was not fixed in
   * the parent node, but is fixed to either 0 or 1 in this node.
   */
  const symbolic::Variable& fixed_binary_variable() const {
    return fixed_binary_variable_;
  }

  /**
   * Getter for the value of the binary variable, which was not fixed in the
   * parent node, but is fixed to either 0 or 1 in this node.
   */
  int fixed_binary_value() const { return fixed_binary_value_; }

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
      const std::list<symbolic::Variable>& binary_variables);

  /**
   * Fix a binary variable to a binary value. Add a constraint y = 0 or y = 1 to
   * the optimization program. Remove this binary variable from the
   * remaining_binary_variables_ list; set the binary_var_ and
   * binary_var_value_.
   */
  void FixBinaryVariable(const symbolic::Variable& binary_variable,
                         int binary_value);

  enum class OptimalSolutionIsIntegral {
    kTrue,   ///< The program in this node has been solved, and the solution to
             /// all binary variables satisfies the integral constraints.
    kFalse,  ///< The program in this node has been solved, and the solution to
    /// some binary variables does not satisfy the integral constraints.
    kUnknown,  ///< Either the program in this node has not been solved, or we
               /// have not checked if the solution satisfy the integral
    /// constraints yet.
  };

  // Stores the optimization program in this node.
  std::unique_ptr<MathematicalProgram> prog_;  
  std::unique_ptr<MixedIntegerBranchAndBoundNode> left_child_;
  std::unique_ptr<MixedIntegerBranchAndBoundNode> right_child_;
  MixedIntegerBranchAndBoundNode* parent_;

  // The newly fixed binary variable z, in the decision variables x.
  // The value of z was not fixed in the parent node, but is fixed in this
  // node.
  symbolic::Variable fixed_binary_variable_;
  // The value of the newly fixed binary variable z, in the decision variables
  // x. The value of z was not fixed in the parent node, but is fixed in this
  // node.
  int fixed_binary_value_;

  // The variables that were binary in the original mixed-integer optimization
  // problem, but whose value has not been fixed to either 0 or 1 yet.
  std::list<symbolic::Variable> remaining_binary_variables_;

  // Whether the optimal solution in this node satisfies all integral
  // constraints.
  OptimalSolutionIsIntegral optimal_solution_is_integral_;
};

/**
 * Given a mixed-integer optimization problem (or more accurately, mixed binary problem), solve this problem through branch-and-bound process. We will first replace all the binary variables with continuous variables, and relax the integral constraint on the binary variables y ∈ {0, 1} with continuous constraints 0 ≤ y ≤ 1. And then in the subsequent steps, at each node of the tree, we will fix some binary variables to either 0 or 1, and solve the rest of the variables.
 * Notice that we will re-create a new set of variables in the branch-and-bound process, since we need to replace the binary variables with continuous variables.
 */
class MixedIntegerBranchAndBound {
 public:
  /**
   * Different methods to pick a branching variable.
   */
  enum class PickVariable {
    kUserDefined,      ///< User defined. 
    kLeastAmbivalent,  ///< Pick the variable whose value is closest to 0 or 1.
    kMostAmbivalent,   ///< Pick the variable whose value is closest to 0.5
  };

  /**
   * Different methods to pick a branching node.
   */
  enum class PickNode {
    kUserDefined,    ///< User defined.
    kDepthFirst,     ///< Pick the node with the most binary variables fixed.
    kMinLowerBound,  ///< Pick the node with the smallest optimal cost.
  };

  /**
   * The function signature for the user defined method to pick a branching node or  a branching variable.
   */
  using PickNodeFun = std::function<MixedIntegerBranchAndBoundNode*(const MixedIntegerBranchAndBoundNode&)>;
  using PickVariableFun = std::function<symbolic::Variable*(const MixedIntegerBranchAndBoundNode&)>;

  /**
   * Construct a branch-and-bound tree from a mixed-integer optimization program.
   * @param prog A mixed-integer optimization program.
   */
  MixedIntegerBranchAndBound(const MathematicalProgram& prog);

  /**
   * Given an old variable in the original mixed-integer program, return the new corresponding variable in the branch-and-bound process.
   * @param old_variable A variable in the original mixed-integer program.
   * @retval new_variable The corresponding variable in the branch-and-bound procedure.
   * @pre old_variable is a variable in the mixed-integer program, passed in the constructor of this MixedIntegerBranchAndBound.
   * @throw a runtime_error if the pre-condition fails.
   */
  const symbolic::Variable& NewVariable(const symbolic::Variable& old_variable) const;

  /**
   * Given a matrix of old variables in the original mixed-integer program, return the new corresponding variables in the branch-and-bound process.
   * @param old_variables Variables in the original mixed-integer program.
   * @retval new_variables The corresponding variables in the branch-and-bound procedure.
   */
  template<typename Derived>
  typename std::enable_if<is_eigen_scalar_same<Derived, symbolic::Variable>::value, MatrixDecisionVariable<Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>>::type NewVariables(const Eigen::MatrixBase<Derived>& old_variables) const {
    Eigen::MatrixBase<Derived> new_variables;
    new_variables.resize(old_variables.rows(), old_variables.cols());
    for (int i = 0; i < old_variables.rows(); ++i) {
      for (int j = 0; j < old_variables.cols(); ++j) {
        new_variables(i, j) = NewVariable(old_variables(i, j));
      }
    }
    return new_variables;
  }

  /**
   * Getter for the root node. Note that this is aliased for the lifetime of this object.
   */
  MixedIntegerBranchAndBoundNode* root() const { return root_.get(); }

 private:
  /**
   * Pick one node to branch.
   */
  MixedIntegerBranchAndBoundNode* PickBranchingNode() const;

  /**
   * Pick the node with the minimal lower bound.
   */
  MixedIntegerBranchAndBoundNode* PickMinLowerBoundNode() const;

  /**
   * Pick the node with the most binary variables fixed.
   */
  MixedIntegerBranchAndBoundNode* PickDepthFirstNode() const;

  // The root node of the tree.
  std::unique_ptr<MixedIntegerBranchAndBoundNode> root_;

  // We re-created the decision variables in the optimization program in the branch-and-bound. All nodes uses the same new set of decision variables, which is different from the variables in the original mixed-integer program (the one passed in the constructor of MixedIntegerBranchAndBound). This map is used to find the corresponding new variable from the old variable in the mixed-integer program.
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable> map_old_vars_to_new_vars_;
  
  // The best upper bound of the mixed-integer optimization optimal cost. An upper bound is obtained by evaluating the cost at a solution satisfying all the constraints (including the integral constraints) in the mixed-integer problem.
  double best_upper_bound_;

  // The best lower bound of the mixed-integer optimization optimal cost. This best lower bound is obtained by taking the minimal of the optimal cost in each leaf node.
  double best_lower_bound_;

  // The branch and bound process will terminate, when the best upper bound is sufficiently close to the best lower bound, that either of the following conditions is satisfied:
  // 1. (best_upper_bound_ - best_lower_bound_) / abs(best_lower_bound_) < relative_gap_tol
  // 2. best_upper_bound_ - best_lower_bound_ < absolute_gap_tol_;
  double absolute_gap_tol_ = 1E-2;
  double relative_gap_tol_ = 1E-2;

  // The list of active leaves. A leaf node is active if its optimization problem has been solved, and the node is not fathomed.
  // A leaf node is fathomed if any of the following conditions is satisfied:
  // 1. The optimization problem in the node is infeasible.
  // 2. The optimal cost of the node is larger than the best upper bound.
  // 3. The optimal Solution to the node satisfies all the integral constraints.
  std::list<MixedIntegerBranchAndBoundNode*> active_leaves_;

  PickVariable pick_variable_ = PickVariable::kMostAmbivalent;

  PickNode pick_node_ = PickNode::kMinLowerBound;

  // The user defined function to pick a branching variable. Default is null.
  PickVariableFun pick_branching_variable_userfun_ = nullptr;

  // The user defined function to pick a branching node. Default is null.
  PickNodeFun pick_branching_node_userfun_ = nullptr;
};
}  // namespace solvers
}  // namespace drake
