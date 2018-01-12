#pragma once

#include <list>
#include <memory>
#include <unordered_map>
#include <utility>

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
   * we will construct a root node for this mixed-integer program. In the root
   * node, it enforces all the costs and constraints in the original program,
   * except the binary constraint y ∈ {0, 1}. Instead, it enforces the relaxed
   * constraint to 0 ≤ y ≤ 1. So the root node contains the program
   * min f(x)         (2)
   * s.t g(x) ≤ 0
   *     0 ≤ y ≤ 1
   * This optimization program is solved during the node construction.
   * @param prog The mixed-integer optimization program (1) in the
   * documentation above.
   * @param solver_id The ID of the solver for the optimization program.
   * @retval (node, map_old_vars_to_new_vars) node is the root node of the tree,
   * that contains the optimization program (2) in the documentation above. This
   * root node has no parent. We also need to recreate new decision variables in
   * the root node, from the original optimization program (1), since the binary
   * variables will be converted to continuous variables in (2). We thus return
   * the map from the old variables to the new variables.
   * @pre prog should contain binary variables.
   * @pre solver_id can be either Gurobi or Scs.
   * @throw std::runtime_error if the preconditions are not met.
   */
  static std::pair<
      std::unique_ptr<MixedIntegerBranchAndBoundNode>,
      std::unordered_map<symbolic::Variable::Id, symbolic::Variable>>
  ConstructRootNode(const MathematicalProgram& prog, const SolverId& solver_id);

  /**
   * Branches on `binary_variable`, and creates two child nodes. In the left
   * child node, the binary variable is fixed to 0. In the right node, the
   * binary variable is fixed to 1. Solves the optimization program in each
   * child node.
   * @param binary_variable This binary variable is fixed to either 0 or 1 in
   * the child node.
   * @pre binary_variable is in remaining_binary_variables_;
   * @throw std::runtime_error if the preconditions are not met.
   */
  void Branch(const symbolic::Variable& binary_variable);

  /** Returns true if a node is the root.
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
  const MathematicalProgram* prog() const { return prog_.get(); }

  /** Getter for the left child. */
  const MixedIntegerBranchAndBoundNode* left_child() const {
    return left_child_.get();
  }

  /** Setter for the left child. */
  MixedIntegerBranchAndBoundNode* left_child() { return left_child_.get(); }

  /** Getter for the right child. */
  const MixedIntegerBranchAndBoundNode* right_child() const {
    return right_child_.get();
  }

  /** Setter for the right child. */
  MixedIntegerBranchAndBoundNode* right_child() { return right_child_.get(); }

  /** Getter for the parent node. */
  const MixedIntegerBranchAndBoundNode* parent() const { return parent_; }

  /** Setter for the parent node. */
  MixedIntegerBranchAndBoundNode* parent() { return parent_; }

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

  /** Getter for the solution result when solving the optimization program. */
  SolutionResult solution_result() const { return solution_result_; }

  /**
   * Getter for optimal_solution_is_integral.
   * @pre The optimization problem is solved successfully.
   * @throws a runtime error if the precondition is not satisfied.
   */
  bool optimal_solution_is_integral() const;

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
      const std::list<symbolic::Variable>& binary_variables,
      const SolverId& solver_id);

  /**
   * Fix a binary variable to a binary value. Add a constraint y = 0 or y = 1 to
   * the optimization program. Remove this binary variable from the
   * remaining_binary_variables_ list; set the binary_var_ and
   * binary_var_value_.
   */
  void FixBinaryVariable(const symbolic::Variable& binary_variable,
                         int binary_value);

  /** Check if the optimal solution to the program in this node satisfies all
   * integral constraints.
   * Only call this function AFTER the program is solved.
   */
  void CheckOptimalSolutionIsIntegral();

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

  // The solution result of the optimization program.
  SolutionResult solution_result_;

  // Whether the optimal solution in this node satisfies all integral
  // constraints.
  OptimalSolutionIsIntegral optimal_solution_is_integral_;

  SolverId solver_id_;
};

/**
 * Given a mixed-integer optimization problem (MIP) (or more accurately, mixed
 * binary problem), solve this problem through branch-and-bound process. We will
 * first replace all the binary variables with continuous variables, and relax
 * the integral constraint on the binary variables y ∈ {0, 1} with continuous
 * constraints 0 ≤ y ≤ 1. In the subsequent steps, at each node of the tree,
 * we will fix some binary variables to either 0 or 1, and solve the rest of
 * the variables.
 * Notice that we will create a new set of variables in the branch-and-bound
 * process, since we need to replace the binary variables with continuous
 * variables.
 */
class MixedIntegerBranchAndBound {
 public:
  /**
   * Different methods to pick a branching variable.
   */
  enum class VariableSelectionMethod {
    kUserDefined,      ///< User defined.
    kLeastAmbivalent,  ///< Pick the variable whose value is closest to 0 or 1.
    kMostAmbivalent,   ///< Pick the variable whose value is closest to 0.5
  };

  /**
   * Different methods to pick a branching node.
   */
  enum class NodeSelectionMethod {
    kUserDefined,    ///< User defined.
    kDepthFirst,     ///< Pick the node with the most binary variables fixed.
    kMinLowerBound,  ///< Pick the node with the smallest optimal cost.
  };

  /**
   * The function signature for the user defined method to pick a branching node
   * or a branching variable.
   */
  using NodeSelectFun = std::function<MixedIntegerBranchAndBoundNode*(
      const MixedIntegerBranchAndBoundNode&)>;
  using VariableSelectFun = std::function<const symbolic::Variable*(
      const MixedIntegerBranchAndBoundNode&)>;

  /**
   * Construct a branch-and-bound tree from a mixed-integer optimization
   * program.
   * @param prog A mixed-integer optimization program.
   * @param solver_id The ID of the solver for the optimization.
   */
  explicit MixedIntegerBranchAndBound(const MathematicalProgram& prog,
                                      const SolverId& solver_id);

  /**
   * Solve the mixed-integer problem (MIP) through a branch and bound process.
   * @retval solution_result If solution_result=SolutionResult::kSolutionFound,
   * then the best solutions are stored inside solutions(). The user
   * can access the value of each variable(s) through GetSolution(...).
   * If solution_result=SolutionResult::kInfeasibleConstraints, then the
   * mixed-integer problem is primal infeasible.
   * If solution_result=SolutionResult::kUnbounded, then the mixed-integer
   * problem is primal unbounded.
   */
  SolutionResult Solve();

  /** Get the optimal cost. */
  double GetOptimalCost() const;

  /**
   * Get the n'th sub-optimal cost.
   * The costs are sorted in the ascending order. The sub-optimal costs do not
   * include the optimal cost.
   * @param nth_suboptimal_cost The n'th sub-optimal cost.
   * @pre `nth_suboptimal_cost` is between 0 and solutions().size() - 1.
   * @throws a runtime error if the precondition is not satisfied.
   */
  double GetSubOptimalCost(int nth_suboptimal_cost) const;

  /**
   * Get the n'th best integral solution for a variable.
   * The best solutions are sorted in the ascending order based on their costs.
   * @param mip_var A variable in the original MIP.
   * @param nth_best_solution. The index of the best integral solution.
   * @pre `mip_var` is a variable in the original MIP.
   * @pre `nth_best_solution` is between 0 and solutions().size().
   * @throw runtime error if the preconditions are not satisfied.
   */
  double GetSolution(const symbolic::Variable& mip_var,
                     int nth_best_solution = 0) const;

  /**
   * Get the n'th best integral solution for some variables.
   * The best solutions are sorted in the ascending order based on their costs.
   * @param mip_vars Variables in the original MIP.
   * @param nth_best_solution. The index of the best integral solution.
   * @pre `mip_vars` are variables in the original MIP.
   * @pre `nth_best_solution` is between 0 and solutions().size().
   * @throw runtime error if the preconditions are not satisfied.
   */
  template <typename Derived>
  typename std::enable_if<
      std::is_same<typename Derived::Scalar, symbolic::Variable>::value,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>::type
  GetSolution(const Eigen::MatrixBase<Derived>& mip_vars,
              int nth_best_solution = 0) const {
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime>
        value(mip_vars.rows(), mip_vars.cols());
    for (int i = 0; i < mip_vars.rows(); ++i) {
      for (int j = 0; j < mip_vars.cols(); ++j) {
        value(i, j) = GetSolution(mip_vars(i, j), nth_best_solution);
      }
    }
    return value;
  }

  /**
   * Given an old variable in the original mixed-integer program, return the
   * corresponding new variable in the branch-and-bound process.
   * @param old_variable A variable in the original mixed-integer program.
   * @retval new_variable The corresponding variable in the branch-and-bound
   * procedure.
   * @pre old_variable is a variable in the mixed-integer program, passed in the
   * constructor of this MixedIntegerBranchAndBound.
   * @throw a runtime_error if the pre-condition fails.
   */
  const symbolic::Variable& GetNewVariable(
      const symbolic::Variable& old_variable) const;

  /**
   * Given a matrix of old variables in the original mixed-integer program,
   * return a matrix of corresponding new variables in the branch-and-bound
   * process.
   * @param old_variables Variables in the original mixed-integer program.
   * @retval new_variables The corresponding variables in the branch-and-bound
   * procedure.
   */
  template <typename Derived>
  typename std::enable_if<
      is_eigen_scalar_same<Derived, symbolic::Variable>::value,
      MatrixDecisionVariable<Derived::RowsAtCompileTime,
                             Derived::ColsAtCompileTime>>::type
  GetNewVariables(const Eigen::MatrixBase<Derived>& old_variables) const {
    Eigen::MatrixBase<Derived> new_variables;
    new_variables.resize(old_variables.rows(), old_variables.cols());
    for (int i = 0; i < old_variables.rows(); ++i) {
      for (int j = 0; j < old_variables.cols(); ++j) {
        new_variables(i, j) = GetNewVariable(old_variables(i, j));
      }
    }
    return new_variables;
  }

  /**
   * The user can choose the method to pick a node for branching. We provide
   * options such as "depth first" or "min lower bound".
   * @param pick_node The option to pick a node. If the option is
   * NodeSelectionMethod::kUserDefined, then the user should also provide the
   * method to pick a node through SetUserDefinedNodeSelectionFunction.
   */
  void SetNodeSelectionMethod(NodeSelectionMethod pick_node) {
    pick_node_ = pick_node;
  }

  /**
   * Set the user-defined method to pick the branching node. This method is
   * used if the user calls
   * SetNodeSelectionMethod(NodeSelectionMethod::kUserDefined)
   */
  void SetUserDefinedNodeSelectionFunction(NodeSelectFun fun) {
    node_selection_userfun_ = fun;
  }

  /**
   * The user can choose the method to pick a variable for branching in each
   * node. We provide options such as "most ambivalent" or "least ambivalent".
   * @param pick_variable The option to pick a variable. If the option is
   * VariableSelectionMethod::kUserDefined, then the user should also provide
   * the method
   * to pick a variable through SetUserDefinedVariableSelectionFunction.
   */
  void SetVariableSelectionMethod(VariableSelectionMethod pick_variable) {
    pick_variable_ = pick_variable;
  }

  /**
   * Set the user-defined method to pick the branching variable. This method is
   * used if the user calls
   * SetVariableSelectionMethod(VariableSelectionMethod::kUserDefined).
   */
  void SetUserDefinedVariableSelectionFunction(VariableSelectFun fun) {
    variable_selection_userfun_ = fun;
  }

  /**
   * If a leaf node is fathomed, then there is no need to branch on this node
   * any more. A leaf node is fathomed is any of the following conditions are
   * satisfied:
   * 1. The optimization problem in the node is infeasible.
   * 2. The optimal cost of the node is larger than the best upper bound.
   * 3. The optimal solution to the node satisfies all the integral constraints.
   * 4. All binary variables are fixed to either 0 or 1 in this node.
   * @param leaf_node A leaf node to check if it is fathomed.
   * @pre The node should be a leaf node.
   * @throws runtime error if the precondition is not satisfied.
   */
  bool IsLeafNodeFathomed(
      const MixedIntegerBranchAndBoundNode& leaf_node) const;

  /**
   * Getter for the root node. Note that this is aliased for the lifetime of
   * this object.
   */
  MixedIntegerBranchAndBoundNode* root() { return root_.get(); }

  /** Getter for the best upper bound. */
  double best_upper_bound() const { return best_upper_bound_; }

  /** Getter for the best lower bound. */
  double best_lower_bound() const { return best_lower_bound_; }

  /**
   * Getter for the solutions.
   * Returns a list of solutions, together with the costs evaluated at the
   * solutions. The solutions are sorted in the ascending order based on the
   * cost.
   */
  const std::list<std::pair<double, Eigen::VectorXd>>& solutions() const {
    return solutions_;
  }

 private:
  // Forward declaration the tester class.
  friend class MixedIntegerBranchAndBoundTester;

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

  /**
   * Pick the branching variable in a node.
   */
  const symbolic::Variable* PickBranchingVariable(
      const MixedIntegerBranchAndBoundNode& node) const;

  /**
   * Pick the most ambivalent one as the branching variable, namely the binary
   * variable whose value is closest to 0.5.
   */
  const symbolic::Variable* PickMostAmbivalentAsBranchingVariable(
      const MixedIntegerBranchAndBoundNode& node) const;

  /**
   * Pick the least ambivalent one as the branching variable, namely the binary
   * variable whose value is closet to 0 or 1.
   */
  const symbolic::Variable* PickLeastAmbivalentAsBranchingVariable(
      const MixedIntegerBranchAndBoundNode& node) const;

  /**
   * Branch on a node, solves the optimization, and update the best lower and
   * upper bounds.
   * @param node. The node to be branched.
   * @param branching_variable. Branch on this variable in the node.
   */
  void BranchAndUpdate(MixedIntegerBranchAndBoundNode* node,
                       const symbolic::Variable& branching_variable);

  /**
   * Update the solutions (solutions_) and the best upper bound, with an
   * integral solution and its cost.
   * @param solution. The integral solution.
   * @param cost. The cost evaluated at this integral solution.
   */
  void UpdateIntegralSolution(const Eigen::Ref<const Eigen::VectorXd>& solution,
                              double cost);

  /**
   * The branch-and-bound is converged if the gap between the best upper bound
   * and the best lower bound is less than the tolerance.
   */
  bool IsConverged() const;

  /**
   * Search for an integral solution satisfying all the constraints in this
   * node, together with the integral constraints in the original mixed-integer
   * program.
   */
  void SearchIntegralSolution(const MixedIntegerBranchAndBoundNode& node);

  // The root node of the tree.
  std::unique_ptr<MixedIntegerBranchAndBoundNode> root_;

  // We re-created the decision variables in the optimization program in the
  // branch-and-bound. All nodes uses the same new set of decision variables,
  // which is different from the variables in the original mixed-integer program
  // (the one passed in the constructor of MixedIntegerBranchAndBound). This map
  // is used to find the corresponding new variable from the old variable in the
  // mixed-integer program.
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable>
      map_old_vars_to_new_vars_;

  // The best upper bound of the mixed-integer optimization optimal cost. An
  // upper bound is obtained by evaluating the cost at a solution satisfying all
  // the constraints (including the integral constraints) in the mixed-integer
  // problem.
  double best_upper_bound_;

  // The best lower bound of the mixed-integer optimization optimal cost. This
  // best lower bound is obtained by taking the minimal of the optimal cost in
  // each leaf node.
  double best_lower_bound_;

  // Solutions found so far. Each entry in this list contains both the
  // cost and the decision variable values. This list is sorted in the
  // ascending order based on the cost, and it contains at most
  // max_num_solutions_ elements.
  std::list<std::pair<double, Eigen::VectorXd>> solutions_;
  int max_num_solutions_{10};

  // The branch and bound process will terminate, when the best upper bound is
  // sufficiently close to the best lower bound, that either of the following
  // conditions is satisfied:
  // 1. (best_upper_bound_ - best_lower_bound_) / abs(best_lower_bound_) <
  // relative_gap_tol
  // 2. best_upper_bound_ - best_lower_bound_ < absolute_gap_tol_;
  double absolute_gap_tol_ = 1E-2;
  double relative_gap_tol_ = 1E-2;

  VariableSelectionMethod pick_variable_ =
      VariableSelectionMethod::kMostAmbivalent;

  NodeSelectionMethod pick_node_ = NodeSelectionMethod::kMinLowerBound;

  // The user defined function to pick a branching variable. Default is null.
  VariableSelectFun variable_selection_userfun_ = nullptr;

  // The user defined function to pick a branching node. Default is null.
  NodeSelectFun node_selection_userfun_ = nullptr;
};
}  // namespace solvers
}  // namespace drake
