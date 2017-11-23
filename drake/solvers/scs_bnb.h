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
// s.t A_prime * x_prime + s = b_prime
//     s in K
// where x is obtained by removing the binary variable y from x_prime.
// Notice that the matrix A, b, c will change from node to node, but the cone K
// is unchanged.
class ScsNode {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScsNode)

  // Create the root node from the SCS problem data.
  ScsNode(AMatrix* A, scs_float* b, scs_float* c, const std::list<int>& binary_var_indices, double cost_constant);

  ~ScsNode();

  // Branch on one binary variable, create two child nodes
  void Branch(int binary_var_index);

  AMatrix* A() const { return A_; }

  scs_float* b() const { return b_; }

  scs_float* c() const { return c_; }

  const std::list<int>& binary_var_indices() const { return binary_var_indices_; }

  int y_index() const { return y_index_; }

  int y_val() const { return y_val_; }

  double cost_constant() const { return cost_constant_; }

  ScsNode* left_child() const { return left_child_; }

  ScsNode* right_child() const { return right_child_; }

  ScsNode* parent() const { return parent_; }

 private:
  ScsNode() {}
  AMatrix* A_;
  scs_float* b_;
  scs_float* c_;
  // This node is created from its parent node, by fixing a variable y to a
  // binary value. That variable y has index y_index_ in the parent node.
  int y_index_;
  int y_val_;
  // The optimization program can add a constant term to the cost.
  double cost_constant_;
  double upper_; // upper bound
  double lower_; // lower bound
  // binary_var_indices_ are the indices of the remaining binary variables, in
  // the vector x. The indices are in the ascending order.
  std::list<int> binary_var_indices_;

  ScsNode* left_child_ = nullptr;
  ScsNode* right_child_ = nullptr;
  ScsNode* parent_;
};
/*
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
  std::list<int>
};*/
}  // namespace solvers
}  // namespace drake
