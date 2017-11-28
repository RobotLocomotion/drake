#pragma once

#include <list>

// clang-format off
#include "scs.h"
#include "linsys/amatrix.h"
// clang-format on

#include "drake/common/symbolic.h"
namespace drake {
namespace solvers {
// Inside each node, we solve an SCS problem
// min cᵀx
// s.t Ax + s = b
//     s in K
// This node is created from its parent node, by fixing a binary variable y to
// either 0 or 1. The parent node solves the problem
// min c_primeᵀ * x_prime
// s.t A_prime * x_prime + s_prime = b_prime
//     s_prime in K_prime
// where x is obtained by removing the binary variable y from x_prime.
// Notice that the matrix A, b, c will change from node to node.
class ScsNode {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScsNode)

  ScsNode(int num_A_rows, int num_A_cols);
  /**
   * Create the root node from the SCS problem data. The constraint
   * Ax + s = b does NOT include the integral constraint on the binary
   * variables. Neither does it include the relaxed constraint on the binary
   * variable y (i.e, 0 <= y <= 1)
   * @param A
   * @param b
   * @param c
   * @param cone The life of the cone must outlive the life of the node and the
   * tree for the branch and bound.
   * @param binary_var_indices
   * @param cost_constant
   */
  static std::unique_ptr<ScsNode> ConstructRootNode(
      const AMatrix& A, const scs_float* const b, const scs_float* const c,
      const SCS_CONE& cone, const std::list<int>& binary_var_indices,
      double cost_constant);

  ~ScsNode();

  /**
   * Branches on one binary variable, and creates two child nodes. In each child
   * node, that binary variable is fixed to either 0 or 1.
   * @param binary_var_index The binary variable with this index will be fixed
   * to either 0 or 1 in the child node, and being removed from the decision
   * variables.
   */
  void Branch(int binary_var_index);

  // Solve the optimization problem in this node.
  scs_int Solve(const SCS_SETTINGS* const scs_settings,
                double best_upper_bound);

  // Getter for A matrix.
  const AMatrix* const A() const { return A_.get(); }

  // Getter for b vector.
  const scs_float* const b() const { return b_.get(); }

  // Getter for c vector, the linear coefficient of the cost.
  const scs_float* const c() const { return c_.get(); }

  // Getter for the cones.
  const SCS_CONE* const cone() const { return cone_.get(); }

  bool found_integral_sol() const { return found_integral_sol_; }

  bool larger_than_upper_bound() const { return larger_than_upper_bound_; }

  const std::list<int>& binary_var_indices() const {
    return binary_var_indices_;
  }

  int y_index() const { return y_index_; }

  int y_val() const { return y_val_; }

  double cost_constant() const { return cost_constant_; }

  ScsNode* left_child() const { return left_child_; }

  ScsNode* right_child() const { return right_child_; }

  ScsNode* parent() const { return parent_; }

  double cost() const { return cost_; }

  const SCS_SOL_VARS* const scs_sol() const { return scs_sol_.get(); }

  SCS_INFO scs_info() const { return scs_info_; }

 private:
  // We will solve the problem
  // min c_ᵀx
  // s.t A_x + s = b_
  //     s in K
  // in this node.
  // We will put the constraint 0 ≤ y ≤ 1 in the first rows of the "linear
  // cones" in A. Namely starting from cone_->f'th row in A, to cone_->f + 2N
  // row in A, are of the form
  // -y + s = 0
  // y + s = 1
  // s in positive cone
  // where N is the length of the binary_var_indices_;
  std::unique_ptr<AMatrix, void (*)(AMatrix*)> A_;
  std::unique_ptr<scs_float, void (*)(void*)> b_;
  std::unique_ptr<scs_float, void (*)(void*)> c_;
  // ScsNode does not own cone_->q, cone_->s, cone_->p. Notice that only
  // cone_->l changes between each node, the length of other constraints, such
  // as second order cone, semi-definite cone, etc, do not change.
  std::unique_ptr<SCS_CONE, void (*)(void*)> cone_;
  // This node is created from its parent node, by fixing a variable y to a
  // binary value. That variable y has index y_index_ in the parent node.
  int y_index_;
  int y_val_;
  // The optimization program can add a constant term to the cost.
  double cost_constant_;
  std::unique_ptr<SCS_SOL_VARS, void (*)(SCS_SOL_VARS*)> scs_sol_;
  SCS_INFO scs_info_;
  double cost_;
  // If a node is fathomed, then there is no need to branch on this node.
  // There are two possible cases to make a node fathomed_
  // 1. The optimal solution to the relaxed problem satisfy all the integral
  //    constraints.
  // 2. The optimal solution to the relaxed problem is larger than the current
  //    best upper bound.
  bool found_integral_sol_;
  bool larger_than_upper_bound_;
  // binary_var_indices_ are the indices of the remaining binary variables, in
  // the vector x. The indices are in the ascending order.
  std::list<int> binary_var_indices_;

  ScsNode* left_child_ = nullptr;
  ScsNode* right_child_ = nullptr;
  ScsNode* parent_;

  // If the solution is within integer_tol to an integer value, then we regard
  // the solution as taking the integer value.
  double integer_tol_ = 1E-2;
};

// Given a mixed-integer convex optimization program in SCS format
// min cᵀx
// s.t Ax + s = b
//     s in K
// And the indices of variable x that should only take binary value {0, 1},
// solve this mixed-integer optimization problem through branch-and-bound.
class ScsBranchAndBound {
 private:
  // scs_data_ includes the data on c, A, b, and the cone K. It also contains
  // the settings of the problem, such as iteration limit, accuracy, etc.
  SCS_PROBLEM_DATA* scs_data_;
  // binary_var_indices_ records the indices of all binary variables in x.
  // We suppose that binary_var_indices_ are in the ascending order.
  std::list<int> binary_var_indices_;
};
}  // namespace solvers
}  // namespace drake
