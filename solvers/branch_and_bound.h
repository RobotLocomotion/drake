#pragma once

#include <list>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace solvers {
/**
 * A node in the branch-and-bound (bnb) tree.
 * The whole branch-and-bound tree solves the mixed-integer problem
 * min f(x)         (1)
 * s.t g(x) ≤ 0
 *     z ∈ {0, 1}
 * where the binary variables z are a subset of the decision variables x.
 * In this node, we will fix some binary variables to either 0 and 1, and relax
 * the rest of the binary variables to continuous variables between 0 and 1.
 * Namely we will solve the following problem with all variables being
 * continuous
 * min f(x)         (2)
 * s.t g(x) ≤ 0
 *     z_fixed = b_fixed
 *     0 ≤ z_relaxed ≤ 1
 * where z_fixed, z_relaxed is a partition of the original binary variables z.
 * z_fixed is the fixed binary variables, z_relaxed is the relaxed binary
 * variables. b_fixed is a vector containing the assigned values of the fixed
 * binary variables z_fixed, b_fixed only contains value either 0 or 1.
 *
 * Each node is created from its parent node, by fixing one binary variable to
 * either 0 or 1.
 */
class MixedIntegerBranchAndBoundNode {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MixedIntegerBranchAndBoundNode)

  /** Construct the root node from an optimization program.
   * For the mixed-integer optimization program
   * min f(x)         (1)
   * s.t g(x) ≤ 0
   *     z ∈ {0, 1}
   * we will construct a root node for this mixed-integer program. In the root
   * node, it enforces all the costs and constraints in the original program,
   * except the binary constraint z ∈ {0, 1}. Instead, it enforces the relaxed
   * constraint to 0 ≤ z ≤ 1. So the root node contains the program
   * min f(x)         (2)
   * s.t g(x) ≤ 0
   *     0 ≤ z ≤ 1
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
   * @throws std::exception if the preconditions are not met.
   */
  static std::pair<
      std::unique_ptr<MixedIntegerBranchAndBoundNode>,
      std::unordered_map<symbolic::Variable::Id, symbolic::Variable>>
  ConstructRootNode(const MathematicalProgram& prog, const SolverId& solver_id);

  /**
   * Branches on @p binary_variable, and creates two child nodes. In the left
   * child node, the binary variable is fixed to 0. In the right node, the
   * binary variable is fixed to 1. Solves the optimization program in each
   * child node.
   * @param binary_variable This binary variable is fixed to either 0 or 1 in
   * the child node.
   * @pre binary_variable is in remaining_binary_variables_;
   * @throws std::exception if the preconditions are not met.
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

  /**
   * Getter for the mathematical program result.
   */
  const MathematicalProgramResult* prog_result() const {
    return prog_result_.get();
  }

  /** Getter for the left child. */
  const MixedIntegerBranchAndBoundNode* left_child() const {
    return left_child_.get();
  }

  /** Getter for the mutable left child. */
  MixedIntegerBranchAndBoundNode* mutable_left_child() {
    return left_child_.get();
  }

  /** Getter for the right child. */
  const MixedIntegerBranchAndBoundNode* right_child() const {
    return right_child_.get();
  }

  /** Getter for the mutable right child. */
  MixedIntegerBranchAndBoundNode* mutable_right_child() {
    return right_child_.get();
  }

  /** Getter for the parent node. */
  const MixedIntegerBranchAndBoundNode* parent() const { return parent_; }

  /** Getter for the mutable parent node. */
  MixedIntegerBranchAndBoundNode* mutable_parent() { return parent_; }

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
   * @throws std::exception if the precondition is not satisfied.
   */
  bool optimal_solution_is_integral() const;

  /** Getter for solver id. */
  const SolverId& solver_id() const { return solver_id_; }

 private:
  /**
   * If the solution to a binary variable is either less than integral_tol or
   * larger than 1 - integral_tol, then we regard the solution to be binary.
   * This method set this tolerance.
   */
  void set_integral_tolerance(double integral_tol) {
    integral_tol_ = integral_tol;
  }

 private:
  // Constructs an empty node. Clone the input mathematical program to this
  // node. The child and the parent nodes are all nullptr.
  // @param prog The optimization program whose binary variable constraints are
  // all relaxed to 0 ≤ z ≤ 1.
  // @param binary_variables The list of binary variables in the mixed-integer
  // problem.
  MixedIntegerBranchAndBoundNode(
      const MathematicalProgram& prog,
      const std::list<symbolic::Variable>& binary_variables,
      const SolverId& solver_id);

  // Fix a binary variable to a binary value. Add a constraint z = 0 or z = 1 to
  // the optimization program. Remove this binary variable from the
  // remaining_binary_variables_ list; set the binary_var_ and
  // binary_var_value_.
  void FixBinaryVariable(const symbolic::Variable& binary_variable,
                         bool binary_value);

  // Check if the optimal solution to the program in this node satisfies all
  // integral constraints.
  // Only call this function AFTER the program is solved.
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
  std::unique_ptr<MathematicalProgramResult> prog_result_;
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

  // If the solution to a binary variable is either less than integral_tol or
  // larger than 1 - integral_tol, then we regard the solution to be binary.
  double integral_tol_{1E-5};
};

/**
 * Given a mixed-integer optimization problem (MIP) (or more accurately, mixed
 * binary problem), solve this problem through branch-and-bound process. We will
 * first replace all the binary variables with continuous variables, and relax
 * the integral constraint on the binary variables z ∈ {0, 1} with continuous
 * constraints 0 ≤ z ≤ 1. In the subsequent steps, at each node of the tree,
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
      const MixedIntegerBranchAndBound&)>;
  using VariableSelectFun = std::function<const symbolic::Variable*(
      const MixedIntegerBranchAndBoundNode&)>;
  /** The function signature for user defined node callback function. */
  using NodeCallbackFun = std::function<void(
      const MixedIntegerBranchAndBoundNode&, MixedIntegerBranchAndBound* bnb)>;

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
   * @throws std::exception if the precondition is not satisfied.
   */
  double GetSubOptimalCost(int nth_suboptimal_cost) const;

  /**
   * Get the n'th best integral solution for a variable.
   * The best solutions are sorted in the ascending order based on their costs.
   * Each solution is found in a separate node in the branch-and-bound tree, so
   * the values of the binary variables are different in each solution.
   * @param mip_var A variable in the original MIP.
   * @param nth_best_solution. The index of the best integral solution.
   * @pre `mip_var` is a variable in the original MIP.
   * @pre `nth_best_solution` is between 0 and solutions().size().
   * @throws std::exception if the preconditions are not satisfied.
   */
  double GetSolution(const symbolic::Variable& mip_var,
                     int nth_best_solution = 0) const;

  /**
   * Get the n'th best integral solution for some variables.
   * The best solutions are sorted in the ascending order based on their costs.
   * Each solution is found in a separate node in the branch-and-bound tree, so
   * @param mip_vars Variables in the original MIP.
   * @param nth_best_solution. The index of the best integral solution.
   * @pre `mip_vars` are variables in the original MIP.
   * @pre `nth_best_solution` is between 0 and solutions().size().
   * @throws std::exception if the preconditions are not satisfied.
   */
  template <typename Derived>
  typename std::enable_if_t<
      std::is_same_v<typename Derived::Scalar, symbolic::Variable>,
      Eigen::Matrix<double, Derived::RowsAtCompileTime,
                    Derived::ColsAtCompileTime>>
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
   * @throws std::exception if the pre-condition fails.
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
  typename std::enable_if_t<
      is_eigen_scalar_same<Derived, symbolic::Variable>::value,
      MatrixDecisionVariable<Derived::RowsAtCompileTime,
                             Derived::ColsAtCompileTime>>
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
   * @param node_selection_method The option to pick a node. If the option is
   * NodeSelectionMethod::kUserDefined, then the user should also provide the
   * method to pick a node through SetUserDefinedNodeSelectionFunction.
   */
  void SetNodeSelectionMethod(NodeSelectionMethod node_selection_method) {
    node_selection_method_ = node_selection_method;
  }

  /**
   * Set the user-defined method to pick the branching node. This method is
   * used if the user calls
   * SetNodeSelectionMethod(NodeSelectionMethod::kUserDefined).
   *
   * For example, if the user has defined a function LeftMostNode that would
   * return the left-most unfathomed node in the tree, then the user could do
   * \code{.cc}
   * MixedIntegerBranchAndBoundNode* LeftMostNodeInSubTree(
   *     const MixedIntegerBranchAndBound& branch_and_bound,
   *     const MixedIntegerBranchAndBoundNode& subtree_root) {
   *   // Starting from the subtree root, find the left most leaf node that is
   * not fathomed.
   *   blah
   * }
   *
   * MixedIntegerBranchAndBound bnb(...);
   * bnb.SetNodeSelectionMethod(
   *     MixedIntegerBranchAndBound::NodeSelectionMethod::kUserDefined);
   * // Use a lambda function as the NodeSelectionFun
   * bnb->SetUserDefinedNodeSelectionFunction([](
   *     const MixedIntegerBranchAndBound& branch_and_bound) {
   *   return LeftMostNodeInSubTree(branch_and_bound,
   * *(branch_and_bound.root()));
   * \endcode
   * A more detailed example can be found in
   * solvers/test/branch_and_bound_test.cc
   * in TestSetUserDefinedNodeSelectionFunction.
   * @note The user defined function should pick an un-fathomed leaf node for
   * branching.
   * @throws std::exception if the node is not a leaf node, or it is
   * fathomed.
   */
  void SetUserDefinedNodeSelectionFunction(NodeSelectFun fun) {
    node_selection_userfun_ = fun;
  }

  /**
   * The user can choose the method to pick a variable for branching in each
   * node. We provide options such as "most ambivalent" or "least ambivalent".
   * @param variable_selection_method The option to pick a variable. If the
   * option is VariableSelectionMethod::kUserDefined, then the user should also
   * provide the method to pick a variable through
   * SetUserDefinedVariableSelectionFunction(...).
   */
  void SetVariableSelectionMethod(
      VariableSelectionMethod variable_selection_method) {
    variable_selection_method_ = variable_selection_method;
  }

  /**
   * Set the user-defined method to pick the branching variable. This method is
   * used if the user calls
   * SetVariableSelectionMethod(VariableSelectionMethod::kUserDefined).
   *
   * For example, if the user has defined a function FirstVariable, that would
   * return the first un-fixed binary variable in this branch as
   * \code{.cc}
   * SymbolicVariable* FirstVariable(const MixedIntegerBranchAndBoundNode& node)
   * {
   *   return node.remaining_binary_variables().begin();
   * }
   * \endcode
   * The user can then set the branch-and-bound to use this function to select
   * the branching variable as
   * \code{.cc}
   * MixedIntegerBranchAndBound bnb(...);
   * bnb.SetVariableSelectionMethod(
   *     MixedIntegerBranchAndBound:VariableSelectionMethod::kUserDefined);
   * // Set VariableSelectFun by using a function pointer.
   * bnb.SetUserDefinedVariableSelectionFunction(FirstVariable);
   * \endcode
   */
  void SetUserDefinedVariableSelectionFunction(VariableSelectFun fun) {
    variable_selection_userfun_ = fun;
  }

  /** Set the flag to true if the user wants to search an integral solution
   * in each node, after the optimization problem in that node is solved.
   * The program can search for an integral solution based on the solution to
   * the optimization program in the node, by rounding the binary variables
   * to the nearest integer value, and solve for the continuous variables.
   * If a solution is obtained in this new program, then this solution is
   * an integral solution to the mixed-integer program.
   */
  void SetSearchIntegralSolutionByRounding(bool flag) {
    search_integral_solution_by_rounding_ = flag;
  }

  /**
   * The user can set a defined callback function in each node. This function is
   * called after the optimization is solved in each node.
   */
  void SetUserDefinedNodeCallbackFunction(NodeCallbackFun fun) {
    node_callback_userfun_ = fun;
  }

  /**
   * If a leaf node is fathomed, then there is no need to branch on this node
   * any more. A leaf node is fathomed is any of the following conditions are
   * satisfied:
   *
   * 1. The optimization problem in the node is infeasible.
   * 2. The optimal cost of the node is larger than the best upper bound.
   * 3. The optimal solution to the node satisfies all the integral constraints.
   * 4. All binary variables are fixed to either 0 or 1 in this node.
   *
   * @param leaf_node A leaf node to check if it is fathomed.
   * @pre The node should be a leaf node.
   * @throws std::exception if the precondition is not satisfied.
   */
  bool IsLeafNodeFathomed(
      const MixedIntegerBranchAndBoundNode& leaf_node) const;

  /**
   * Getter for the root node. Note that this is aliased for the lifetime of
   * this object.
   */
  const MixedIntegerBranchAndBoundNode* root() const { return root_.get(); }

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
  const std::multimap<double, Eigen::VectorXd>& solutions() const {
    return solutions_;
  }

  /** Setter for the absolute gap tolerance.
   * The branch-and-bound will terminate if its difference between its best
   * upper bound and best lower bound is below this gap tolerance.
   */
  void set_absolute_gap_tol(double tol) { absolute_gap_tol_ = tol; }

  /** Getter for the absolute gap tolerance. */
  double absolute_gap_tol() const { return absolute_gap_tol_; }

  /** Setter for the relative gap tolerance.
   * The branch-and-bound will terminate if
   * (best_upper_bound() - best_lower_bound()) / abs(best_lower_bound())
   * is smaller than this tolerance.
   */
  void set_relative_gap_tol(double tol) { relative_gap_tol_ = tol; }

  /** Geeter for the relative gap tolerance. */
  double relative_gap_tol() const { return relative_gap_tol_; }

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
   * The branch-and-bound has converged if the gap between the best upper bound
   * and the best lower bound is less than the tolerance.
   */
  bool HasConverged() const;

  /** Call the callback function in each node. */
  void NodeCallback(const MixedIntegerBranchAndBoundNode& node);

  /**
   * Search for an integral solution satisfying all the constraints in this
   * node, together with the integral constraints in the original mixed-integer
   * program. It will construct a new optimization program, same as the one
   * in this node, but the remaining binary variables are all rounded to
   * the binary value that is closest to the solution of the optimization
   * program in this node.
   * @note this function is only called if the following conditions are
   * satisfied:
   * 1. The optimization problem in this node is feasible.
   * 2. The optimal solution to the problem in this node is not integral.
   * 3. The user called SetSearchIntegralSolutionByRounding(true);
   *
   * @note This method will change the data field such as solutions_ and/or
   * best_upper_bound_, if an integral solution is found.
   */
  void SearchIntegralSolutionByRounding(
      const MixedIntegerBranchAndBoundNode& node);

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
  std::multimap<double, Eigen::VectorXd> solutions_;
  int max_num_solutions_{10};

  // The branch and bound process will terminate when the best upper bound is
  // sufficiently close to the best lower bound, that is, when either of the
  // following conditions is satisfied:
  // 1. (best_upper_bound_ - best_lower_bound_) / abs(best_lower_bound_) <
  // relative_gap_tol
  // 2. best_upper_bound_ - best_lower_bound_ < absolute_gap_tol_;
  double absolute_gap_tol_ = 1E-2;
  double relative_gap_tol_ = 1E-2;

  VariableSelectionMethod variable_selection_method_ =
      VariableSelectionMethod::kMostAmbivalent;

  NodeSelectionMethod node_selection_method_ =
      NodeSelectionMethod::kMinLowerBound;

  bool search_integral_solution_by_rounding_ = false;

  // The user defined function to pick a branching variable. Default is null.
  VariableSelectFun variable_selection_userfun_ = nullptr;

  // The user defined function to pick a branching node. Default is null.
  NodeSelectFun node_selection_userfun_ = nullptr;

  // The user defined callback function in each node. Default is null.
  NodeCallbackFun node_callback_userfun_ = nullptr;
};
}  // namespace solvers
}  // namespace drake
