#include "drake/solvers/scs_bnb.h"

#include <limits>

// This code is in C style, to be compatible with the SCS source code.
namespace drake {
namespace solvers {
ScsNode::ScsNode()
    : A_{static_cast<AMatrix*>(malloc(sizeof(AMatrix)))},
      b_{nullptr},
      c_{nullptr},
      scs_sol_{static_cast<SCS_SOL_VARS*>(scs_calloc(1, sizeof(SCS_SOL_VARS)))},
      cost_{NAN},
      found_integral_sol_{false},
      larger_than_upper_bound_{false}{}

ScsNode::ScsNode(AMatrix* A, scs_float* b, scs_float* c, const std::list<int>& binary_var_indices, double cost_constant)
    : A_{static_cast<AMatrix*>(malloc(sizeof(AMatrix)))},
      b_{static_cast<scs_float*>(scs_calloc(A->m, sizeof(scs_float)))},
      c_{static_cast<scs_float*>(scs_calloc(A->n, sizeof(scs_float)))},
      y_index_(-1),
      y_val_{-1},
      cost_constant_{cost_constant},
      scs_sol_{static_cast<SCS_SOL_VARS*>(scs_calloc(1, sizeof(SCS_SOL_VARS)))},
      scs_info_{0},
      cost_{NAN},
      found_integral_sol_{false},
      larger_than_upper_bound_{false},
      binary_var_indices_{binary_var_indices},
      left_child_{nullptr},
      right_child_{nullptr},
      parent_{nullptr} {
  A_->m = A->m;
  A_->n = A->n;
  const int A_nnz = A->p[A->n];
  A_->x = static_cast<scs_float*>(scs_calloc(A_nnz, sizeof(scs_float)));
  A_->i = static_cast<scs_int*>(scs_calloc(A_nnz, sizeof(scs_int)));
  A_->p = static_cast<scs_int*>(scs_calloc((A->n) + 1, sizeof(scs_int)));
  for (int i = 0; i < A_nnz; ++i) {
    A_->x[i] = A->x[i];
    A_->i[i] = A->i[i];
  }
  for (int i = 0; i < (A->n) + 1; ++i) {
    A_->p[i] = A->p[i];
  }
  for (int i = 0; i < A->m; ++i) {
    b_[i] = b[i];
  }
  for (int i = 0; i < A->n; ++i) {
    c_[i] = c[i];
  }
}

ScsNode::~ScsNode() {
  if (left_child_) {
    delete left_child_;
  }
  if (right_child_) {
    delete right_child_;
  }
  freeAMatrix(A_);
  if (b_) {
    scs_free(b_);
  }
  if (c_) {
    scs_free(c_);
  }
  freeSol(scs_sol_);
}

void ScsNode::Branch(int binary_var_index) {
  // We first compute the matrix A and b for the remaining variables.
  // We can write Ax+s = b as
  // A_bar * x_bar + s = b - a * z
  // where z is the value of the binary variable. `a` is the column in A,
  // corresponding to that binary variable. x_bar contains the remaining
  // variables.
  left_child_ = new ScsNode();
  // The A matrix of both the left and the right child nodes are the same, so we
  // compute the A matrix for left node first, and then just copy its value to
  // the right node later.
  left_child_->A_->m = this->A_->m;
  left_child_->A_->n = this->A_->n - 1;
  left_child_->A_->p = static_cast<scs_int*>(scs_calloc(left_child_->A_->n + 1, sizeof(scs_int)));
  // a_nnz is the number of num-zero entries in the column vector a.
  const int a_nnz = this->A_->p[binary_var_index + 1] - this->A_->p[binary_var_index];
  for (int i = 0; i < binary_var_index; ++i) {
    left_child_->A_->p[i] = this->A_->p[i];
  }
  for (int i = binary_var_index; i < left_child_->A_->n + 1; ++i) {
    left_child_->A_->p[i] = this->A_->p[i + 1] - a_nnz;
  }
  const int A_bar_nnz = left_child_->A_->p[left_child_->A_->n];
  left_child_->A_->i = static_cast<scs_int*>(scs_calloc(A_bar_nnz, sizeof(scs_int)));
  left_child_->A_->x = static_cast<scs_float*>(scs_calloc(A_bar_nnz, sizeof(scs_float)));
  for (int i = 0; i < this->A_->p[binary_var_index]; ++i) {
    left_child_->A_->i[i] = this->A_->i[i];
    left_child_->A_->x[i] = this->A_->x[i];
  }
  for (int i = this->A_->p[binary_var_index]; i < A_bar_nnz; ++i) {
    left_child_->A_->i[i] = this->A_->i[i + a_nnz];
    left_child_->A_->x[i] = this->A_->x[i + a_nnz];
  }
  left_child_->b_ = static_cast<scs_float*>(scs_calloc(left_child_->A_->m, sizeof(scs_float)));
  // We fix the binary variable z to 0 in the left child, and to 1 in the right child.
  for (int i = 0; i < left_child_->A_->m; ++i) {
    left_child_->b_[i] = b_[i];
  }

  // We need to remove the binary variable z from binary_var_indices of the
  // child nodes. Also the indices after z should decrement by 1.
  left_child_->binary_var_indices_ = binary_var_indices_;
  for (auto it = left_child_->binary_var_indices_.begin(); it != left_child_->binary_var_indices_.end(); ++it) {
    if ((*it) == binary_var_index) {
      left_child_->binary_var_indices_.erase(it);
      break;
    }
  }
  for (auto it = left_child_->binary_var_indices_.begin(); it != left_child_->binary_var_indices_.end(); ++it) {
    if ((*it) > binary_var_index) {
      (*it)--;
    }
  }
  // left_child_->c is the same as this->c, execpt removing the entry for the
  // binary variable z.
  left_child_->c_ = static_cast<scs_float*>(scs_calloc(left_child_->A_->n, sizeof(scs_float)));
  for (int i = 0; i < binary_var_index; ++i) {
    left_child_->c_[i] = this->c_[i];
  }
  for (int i = binary_var_index; i < left_child_->A_->n; ++i) {
    left_child_->c_[i] = this->c_[i + 1];
  }
  left_child_->cost_constant_ = this->cost_constant_;
  left_child_->parent_ = this;
  left_child_->y_index_ = binary_var_index;
  left_child_->y_val_ = 0;

  // Now copy the A matrix from left node to the right node.
  right_child_ = new ScsNode(left_child_->A_, left_child_->b_, left_child_->c_, left_child_->binary_var_indices_, this->cost_constant_ + this->c_[binary_var_index]);
  for (int i = this->A_->p[binary_var_index]; i < this->A_->p[binary_var_index + 1]; ++i) {
    right_child_->b_[this->A_->i[i]] -= this->A_->x[i];
  }
  right_child_->parent_ = this;
  right_child_->y_index_= binary_var_index;
  right_child_->y_val_ = 1;
}

scs_int ScsNode::Solve(const SCS_CONE* const cone, const SCS_SETTINGS* const scs_settings, double best_upper_bound) {
  SCS_PROBLEM_DATA* scs_problem_data = static_cast<SCS_PROBLEM_DATA*>(scs_calloc(1, sizeof(SCS_PROBLEM_DATA)));
  scs_problem_data->m = A_->m;
  scs_problem_data->n = A_->n;
  scs_problem_data->A = A_;
  scs_problem_data->b = b_;
  scs_problem_data->c = c_;
  scs_problem_data->stgs = const_cast<SCS_SETTINGS*>(scs_settings);

  SCS_WORK* scs_work = scs_init(scs_problem_data, cone, &scs_info_);

  scs_int scs_status = scs_solve(scs_work, scs_problem_data, cone, scs_sol_, &scs_info_);

  if (scs_status == SCS_SOLVED || scs_status == SCS_SOLVED_INACCURATE) {
    cost_ = scs_info_.pobj + cost_constant_;
    // If the remaining variables all take integer value, then update the upper
    // bound, otherwise, the upper bound is the same as the root.
    found_integral_sol_ = true;
    for (auto it = binary_var_indices_.begin(); it != binary_var_indices_.end(); ++it) {
      const double x_val{scs_sol_->x[*it]};
      if (x_val > integer_tol_ && x_val < 1 - integer_tol_) {
        found_integral_sol_ = false;
        break;
      }
    }
    if (cost_ > best_upper_bound) {
      larger_than_upper_bound_ = true;
    }
  } else if (scs_status == SCS_INFEASIBLE || scs_status == SCS_INFEASIBLE_INACCURATE) {
    cost_ = std::numeric_limits<double>::infinity();
  } else if (scs_status == SCS_UNBOUNDED || scs_status == SCS_UNBOUNDED_INACCURATE) {
    cost_ = -std::numeric_limits<double>::infinity();
  }
  // Free allocated memory
  scs_finish(scs_work);
  scs_free(scs_problem_data);
  return scs_status;
}
}
}