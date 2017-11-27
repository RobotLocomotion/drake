#include "drake/solvers/scs_bnb.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

// This code is in C style, to be compatible with the SCS source code.
namespace drake {
namespace solvers {
namespace {
void free_scs_pointer(void* scs_pointer) { scs_free(scs_pointer); }

}  // namespace anonymous

ScsNode::ScsNode(int num_A_rows, int num_A_cols)
    : A_{static_cast<AMatrix*>(malloc(sizeof(AMatrix))), &freeAMatrix},
      b_{static_cast<scs_float*>(scs_calloc(num_A_rows, sizeof(scs_float))),
         &free_scs_pointer},
      c_{static_cast<scs_float*>(scs_calloc(num_A_cols, sizeof(scs_float))),
         &free_scs_pointer},
      cone_{static_cast<SCS_CONE*>(scs_calloc(1, sizeof(SCS_CONE))),
            &free_scs_pointer},
      y_index_(-1),
      y_val_{-1},
      cost_constant_{0},
      scs_sol_{static_cast<SCS_SOL_VARS*>(scs_calloc(1, sizeof(SCS_SOL_VARS))),
               &freeSol},
      scs_info_{0},
      cost_{NAN},
      found_integral_sol_{false},
      larger_than_upper_bound_{false},
      binary_var_indices_{},
      left_child_{nullptr},
      right_child_{nullptr},
      parent_{nullptr} {
  A_->m = num_A_rows;
  A_->n = num_A_cols;
  A_->i = nullptr;
  A_->p = nullptr;
  A_->x = nullptr;
}

std::unique_ptr<ScsNode> ScsNode::ConstructRootNode(
    const AMatrix& A, const scs_float* const b, const scs_float* const c,
    const SCS_CONE& cone, const std::unordered_set<int>& binary_var_indices,
    double cost_constant) {
  auto root =
      std::make_unique<ScsNode>(A.m + 2 * binary_var_indices.size(), A.n);
  root->binary_var_indices_ = binary_var_indices;
  root->cost_constant_ = cost_constant;
  // We need to add the constraint 0 ≤ y ≤ 1 to Ax + s = b, where y is the
  // binary variables.
  // 0 ≤ y ≤ 1 is converted to SCS format as
  // -y + s₁ = 0
  // y + s₂ = 1
  // s₁, s₂ in the positive cone.
  root->A_->m = A.m + 2 * binary_var_indices.size();
  root->A_->n = A.n;
  const int A_nnz = A.p[A.n] + 2 * binary_var_indices.size();
  root->A_->x = static_cast<scs_float*>(scs_calloc(A_nnz, sizeof(scs_float)));
  root->A_->i = static_cast<scs_int*>(scs_calloc(A_nnz, sizeof(scs_int)));
  root->A_->p = static_cast<scs_int*>(scs_calloc((A.n) + 1, sizeof(scs_int)));
  // The linear equality constraints in A_*x + s = b_ is the same as in
  // Ax + s = b
  // We add the constraint
  // -y + s₁ = 0
  // y + s₂ = 1
  // s₁, s₂ in the positive cone.
  // to the top of the linear inequality constraints.
  // The number of columns in A_ is the same as that in A.
  root->A_->p[0] = 0;
  int binary_var_count = 0;
  for (int j = 1; j <= A.n; ++j) {
    root->A_->p[j] = root->A_->p[j - 1] + (A.p[j] - A.p[j - 1]);
    // If the variable in this column is a binary variable, then add two rows of
    // constraints to A_ * x + s = b_
    const bool is_binary_column = root->binary_var_indices_.find(j - 1) !=
                                  root->binary_var_indices_.end();
    root->A_->p[j] += is_binary_column ? 2 : 0;
    int column_nonzero_index = 0;
    for (; A.i[A.p[j - 1] + column_nonzero_index] < cone.f;
         ++column_nonzero_index) {
      root->A_->x[root->A_->p[j - 1] + column_nonzero_index] =
          A.x[A.p[j - 1] + column_nonzero_index];
      root->A_->i[root->A_->p[j - 1] + column_nonzero_index] =
          A.i[A.p[j - 1] + column_nonzero_index];
    }
    if (is_binary_column) {
      // We add the constraint
      // -y + s₁ = 0
      // y + s₂ = 1
      // s₁, s₂ in the positive cone.
      // to the top of the linear inequality constraints.
      root->A_->x[root->A_->p[j - 1] + column_nonzero_index] = -1;
      root->A_->x[root->A_->p[j - 1] + column_nonzero_index + 1] = 1;
      root->A_->i[root->A_->p[j - 1] + column_nonzero_index] =
          cone.f + 2 * binary_var_count;
      root->A_->i[root->A_->p[j - 1] + column_nonzero_index + 1] =
          cone.f + 2 * binary_var_count + 1;
    }
    const int num_added_constraints = is_binary_column ? 2 : 0;
    // The other constraints (such as second order cone, semi-definite cone,
    // etc) are un-changed.
    for (; column_nonzero_index < A.p[j] - A.p[j - 1]; ++column_nonzero_index) {
      root->A_->x[root->A_->p[j - 1] + column_nonzero_index +
                  num_added_constraints] =
          A.x[A.p[j - 1] + column_nonzero_index];
      root->A_->i[root->A_->p[j - 1] + column_nonzero_index +
                  num_added_constraints] =
          A.i[A.p[j - 1] + column_nonzero_index] +
          2 * binary_var_indices.size();
    }
    binary_var_count += is_binary_column ? 1 : 0;
  }
  for (int i = 0; i < cone.f; ++i) {
    root->b_.get()[i] = b[i];
  }

  if (std::any_of(binary_var_indices.begin(), binary_var_indices.end(),
                  [A](int i) { return i < 0 || i >= A.n; })) {
    throw std::runtime_error("binary_var_indices is out of range.");
  }

  for (int i = 0; i < static_cast<int>(binary_var_indices.size()); ++i) {
    root->b_.get()[cone.f + 2 * i] = 0;
    root->b_.get()[cone.f + 2 * i + 1] = 1;
  }
  for (int i = 0; i < A.m - cone.f; ++i) {
    root->b_.get()[cone.f + 2 * binary_var_indices.size() + i] = b[cone.f + i];
  }
  for (int i = 0; i < A.n; ++i) {
    root->c_.get()[i] = c[i];
  }
  root->cone_->f = cone.f;
  root->cone_->l = cone.l + 2 * binary_var_indices.size();
  root->cone_->q = cone.q;
  root->cone_->qsize = cone.qsize;
  root->cone_->s = cone.s;
  root->cone_->ssize = cone.ssize;
  root->cone_->ep = cone.ep;
  root->cone_->ed = cone.ed;
  root->cone_->p = cone.p;
  root->cone_->psize = cone.psize;
  return root;
}

ScsNode::~ScsNode() {
  if (left_child_) {
    delete left_child_;
  }
  if (right_child_) {
    delete right_child_;
  }
}

void ScsNode::Branch(int binary_var_index) {
  // The left child is created by fixing a binary variable z to 0, and the right
  // child is created by fixing z to 1.
  // We need to compute the constraints for the child node.
  // In the current node, the optimization problem is
  // min cᵀ * x + d
  // s.t A * x + s = b
  //     s in K
  // Suppose that in the left child, the optimization problem is
  // min cₗᵀ * xₗ + dₗ
  // s.t Aₗ * xₗ + sₗ = bₗ
  //     sₗ in Kₗ
  // and in the right child, the optimization problem is
  // min cᵣᵀ * xᵣ + dᵣ
  // s.t Aᵣ * xᵣ + sᵣ = bᵣ
  //     sᵣ in Kᵣ
  // First, in the parent node, we have a constraint 0 ≤ z ≤ 1 as a relaxation
  // of the binary constraint. We will remove the rows in A and b, corresponding
  // to this relaxed constraint 0 ≤ z ≤ 1.
  // Second, in the remaining rows of A and b, we will remove the column
  // corresponding to the binary variable z. Since the right node fixes z to 1,
  // we will also need to subtract that column from b in the right node, and add
  // c(binary_var_index) to the constant term in the cost in the right node.
  //
  // Notice that the matrix in the left node Aₗ is the same as the matrix in the
  // right node Aᵣ, so are the cones Kₗ and Kᵣ.

  /*
    // We first compute the matrix A and b for the remaining variables.
    // We can write Ax+s = b as
    // A_bar * x_bar + s = b - a * z
    // where z is the value of the binary variable. `a` is the column in A,
    // corresponding to that binary variable. x_bar contains the remaining
    // variables.
    left_child_ = new ScsNode();
    // The A matrix of both the left and the right child nodes are the same, so
  we
    // compute the A matrix for left node first, and then just copy its value to
    // the right node later.
    left_child_->A_->m = this->A_->m;
    left_child_->A_->n = this->A_->n - 1;
    left_child_->A_->p = static_cast<scs_int*>(scs_calloc(left_child_->A_->n +
  1,
  sizeof(scs_int)));
    // a_nnz is the number of num-zero entries in the column vector a.
    const int a_nnz = this->A_->p[binary_var_index + 1] -
  this->A_->p[binary_var_index];
    for (int i = 0; i < binary_var_index; ++i) {
      left_child_->A_->p[i] = this->A_->p[i];
    }
    for (int i = binary_var_index; i < left_child_->A_->n + 1; ++i) {
      left_child_->A_->p[i] = this->A_->p[i + 1] - a_nnz;
    }
    const int A_bar_nnz = left_child_->A_->p[left_child_->A_->n];
    left_child_->A_->i = static_cast<scs_int*>(scs_calloc(A_bar_nnz,
  sizeof(scs_int)));
    left_child_->A_->x = static_cast<scs_float*>(scs_calloc(A_bar_nnz,
  sizeof(scs_float)));
    for (int i = 0; i < this->A_->p[binary_var_index]; ++i) {
      left_child_->A_->i[i] = this->A_->i[i];
      left_child_->A_->x[i] = this->A_->x[i];
    }
    for (int i = this->A_->p[binary_var_index]; i < A_bar_nnz; ++i) {
      left_child_->A_->i[i] = this->A_->i[i + a_nnz];
      left_child_->A_->x[i] = this->A_->x[i + a_nnz];
    }
    left_child_->b_ = static_cast<scs_float*>(scs_calloc(left_child_->A_->m,
  sizeof(scs_float)));
    // We fix the binary variable z to 0 in the left child, and to 1 in the
  right
  child.
    for (int i = 0; i < left_child_->A_->m; ++i) {
      left_child_->b_[i] = b_[i];
    }

    // We need to remove the binary variable z from binary_var_indices of the
    // child nodes. Also the indices after z should decrement by 1.
    left_child_->binary_var_indices_ = binary_var_indices_;
    for (auto it = left_child_->binary_var_indices_.begin(); it !=
  left_child_->binary_var_indices_.end(); ++it) {
      if ((*it) == binary_var_index) {
        left_child_->binary_var_indices_.erase(it);
        break;
      }
    }
    for (auto it = left_child_->binary_var_indices_.begin(); it !=
  left_child_->binary_var_indices_.end(); ++it) {
      if ((*it) > binary_var_index) {
        (*it)--;
      }
    }
    // left_child_->c is the same as this->c, execpt removing the entry for the
    // binary variable z.
    left_child_->c_ = static_cast<scs_float*>(scs_calloc(left_child_->A_->n,
  sizeof(scs_float)));
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
    right_child_ = new ScsNode(left_child_->A_, left_child_->b_,
  left_child_->c_,
  left_child_->binary_var_indices_, this->cost_constant_ +
  this->c_[binary_var_index]);
    for (int i = this->A_->p[binary_var_index]; i < this->A_->p[binary_var_index
  +
  1]; ++i) {
      right_child_->b_[this->A_->i[i]] -= this->A_->x[i];
    }
    right_child_->parent_ = this;
    right_child_->y_index_= binary_var_index;
    right_child_->y_val_ = 1;*/
}
/*
scs_int ScsNode::Solve(const SCS_CONE* const cone, const SCS_SETTINGS* const
scs_settings, double best_upper_bound) {
  SCS_PROBLEM_DATA* scs_problem_data =
static_cast<SCS_PROBLEM_DATA*>(scs_calloc(1, sizeof(SCS_PROBLEM_DATA)));
  scs_problem_data->m = A_->m;
  scs_problem_data->n = A_->n;
  scs_problem_data->A = A_;
  scs_problem_data->b = b_;
  scs_problem_data->c = c_;
  scs_problem_data->stgs = const_cast<SCS_SETTINGS*>(scs_settings);

  SCS_WORK* scs_work = scs_init(scs_problem_data, cone, &scs_info_);

  scs_int scs_status = scs_solve(scs_work, scs_problem_data, cone, scs_sol_,
&scs_info_);

  if (scs_status == SCS_SOLVED || scs_status == SCS_SOLVED_INACCURATE) {
    cost_ = scs_info_.pobj + cost_constant_;
    // If the remaining variables all take integer value, then update the upper
    // bound, otherwise, the upper bound is the same as the root.
    found_integral_sol_ = true;
    for (auto it = binary_var_indices_.begin(); it != binary_var_indices_.end();
++it) {
      const double x_val{scs_sol_->x[*it]};
      if (x_val > integer_tol_ && x_val < 1 - integer_tol_) {
        found_integral_sol_ = false;
        break;
      }
    }
    if (cost_ > best_upper_bound) {
      larger_than_upper_bound_ = true;
    }
  } else if (scs_status == SCS_INFEASIBLE || scs_status ==
SCS_INFEASIBLE_INACCURATE) {
    cost_ = std::numeric_limits<double>::infinity();
  } else if (scs_status == SCS_UNBOUNDED || scs_status ==
SCS_UNBOUNDED_INACCURATE) {
    cost_ = -std::numeric_limits<double>::infinity();
  }
  // Free allocated memory
  scs_finish(scs_work);
  scs_free(scs_problem_data);
  return scs_status;
}*/
}
}