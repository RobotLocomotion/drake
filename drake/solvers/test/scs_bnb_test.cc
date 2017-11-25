#include "drake/solvers/scs_bnb.h"

#include <gtest/gtest.h>

#include <Eigen/SparseCore>

namespace drake {
namespace solvers {
namespace {
AMatrix* ConstructScsAmatrix(const Eigen::SparseMatrix<double>& A) {
  AMatrix* scs_A = static_cast<AMatrix*>(malloc(sizeof(AMatrix)));
  scs_A->m = A.rows();
  scs_A->n = A.cols();
  scs_A->x = static_cast<scs_float*>(scs_calloc(A.nonZeros(), sizeof(scs_float)));
  scs_A->i = static_cast<scs_int*>(scs_calloc(A.nonZeros(), sizeof(scs_int)));
  scs_A->p = static_cast<scs_int*>(scs_calloc(scs_A->n + 1, sizeof(scs_int)));
  for (int i = 0; i < A.nonZeros(); ++i) {
    scs_A->x[i] = *(A.valuePtr() + i);
    scs_A->i[i] = *(A.innerIndexPtr() + i);
  }
  for (int i = 0; i < scs_A->n + 1; ++i) {
    scs_A->p[i] = *(A.outerIndexPtr() + i);
  }
  return scs_A;
}

void IsAmatrixEqual(const AMatrix* const A1, const AMatrix* const A2, double tol) {
  EXPECT_EQ(A1->m, A2->m);
  EXPECT_EQ(A1->n, A2->n);
  for (int i = 0; i < A1->n + 1; ++i) {
    EXPECT_EQ(A1->p[i], A2->p[i]);
  }
  for (int i = 0; i < A1->p[A1->n]; ++i) {
    EXPECT_NEAR(A1->x[i], A2->x[i], tol);
    EXPECT_EQ(A1->i[i], A2->i[i]);
  }
}

void IsBinaryVarIndicesEqual(const std::list<int>& indices1, const std::list<int>& indices2) {
  EXPECT_EQ(indices1.size(), indices2.size());
  auto it2 = indices2.begin();
  for (auto it1 = indices1.begin(); it1 != indices1.end(); ++it1) {
    EXPECT_EQ(*it1, *it2);
    ++it2;
  }
}
class TestScsNode : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestScsNode)

  TestScsNode() {
    // For a mixed-integer program
    // min x(0) + 2x(1) -3x(3) + 1
    // s.t x(0) + x(1) + 2x(3) = 2
    //     x(1) - 3.1 x(2) >= 1
    //     x(2) + 1.2x(3) - x(0) <= 5
    //     x(0), x(2) are binary
    // We can convert it to the SCS form
    // min cᵀx
    // s.t Ax + s = b
    //     s in K
    // where c = [1; 2; 0; -3]
    // A = [ 1  1    0    2]
    //     [ 0 -1  3.1    0]
    //     [-1  0    1  1.2]
    // b = [2; -1; 5]
    std::vector<Eigen::Triplet<double>> A_triplets;
    A_triplets.emplace_back(0, 0, 1);
    A_triplets.emplace_back(0, 1, 1);
    A_triplets.emplace_back(0, 3, 2);
    A_triplets.emplace_back(1, 1, -1);
    A_triplets.emplace_back(1, 2, 3.1);
    A_triplets.emplace_back(2, 0, -1);
    A_triplets.emplace_back(2, 2, 1);
    A_triplets.emplace_back(2, 3, 1.2);
    Eigen::SparseMatrix<double> A(3, 4);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());
    A.makeCompressed();
    scs_A_ = ConstructScsAmatrix(A);
    binary_var_indices_ = {0, 2};

    cone_ = static_cast<SCS_CONE*>(scs_calloc(1, sizeof(SCS_CONE)));
    cone_->f = 1;
    cone_->l = 2;
  }

  ~TestScsNode() {
    freeAMatrix(scs_A_);
    if (cone_->q) {
      scs_free(cone_->q);
    }
    if (cone_->s) {
      scs_free(cone_->s);
    }
    if (cone_->p) {
      scs_free(cone_->p);
    }
    scs_free(cone_);
  }

 protected:
  AMatrix* scs_A_;
  scs_float b_[3] = {2, -1, 5};
  scs_float c_[4] = {1, 2, 0, -3};
  std::list<int> binary_var_indices_;
  SCS_CONE* cone_;
};

TEST_F(TestScsNode, TestConstructor) {
  const ScsNode root(scs_A_, b_, c_, binary_var_indices_, 1);

  EXPECT_EQ(root.y_index(), -1);
  EXPECT_EQ(root.left_child(), nullptr);
  EXPECT_EQ(root.right_child(), nullptr);
  EXPECT_EQ(root.parent(), nullptr);
  IsAmatrixEqual(scs_A_, root.A(), 0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(b_[i], root.b()[i]);
  }
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(c_[i], root.c()[i]);
  }
  EXPECT_EQ(root.cost_constant(), 1);
  EXPECT_FALSE(root.found_integral_sol());
  EXPECT_FALSE(root.larger_than_upper_bound());
}

TEST_F(TestScsNode, TestBranch) {
  ScsNode root(scs_A_, b_, c_, binary_var_indices_, 1);

  // Branch on x0
  root.Branch(0);
  EXPECT_NE(root.left_child(), nullptr);
  EXPECT_NE(root.right_child(), nullptr);
  EXPECT_EQ(root.left_child()->parent(), &root);
  EXPECT_EQ(root.right_child()->parent(), &root);

  const std::list<int> binary_var_indices_child = {1};
  IsBinaryVarIndicesEqual(root.left_child()->binary_var_indices(), binary_var_indices_child);
  IsBinaryVarIndicesEqual(root.right_child()->binary_var_indices(), binary_var_indices_child);

  // Check the cost vector c
  const scs_float c_child[3] = {2, 0, -3};
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(root.left_child()->c()[i], c_child[i]);
    EXPECT_EQ(root.right_child()->c()[i], c_child[i]);
  }
  EXPECT_EQ(root.left_child()->cost_constant(), 1);
  EXPECT_EQ(root.right_child()->cost_constant(), 2);

  // Check the right-hand side vector b
  const scs_float b_left[3] = {2, -1, 5};
  const scs_float b_right[3] = {1, -1, 6};
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(root.left_child()->b()[i], b_left[i]);
    EXPECT_EQ(root.right_child()->b()[i], b_right[i]);
  }

  // Check the left-hand side matrix A
  // A_child = [ 1    0    2]
  //           [-1  3.1    0]
  //           [ 0    1  1.2]
  std::vector<Eigen::Triplet<double>> A_child_triplets;
  A_child_triplets.emplace_back(0, 0, 1);
  A_child_triplets.emplace_back(0, 2, 2);
  A_child_triplets.emplace_back(1, 0, -1);
  A_child_triplets.emplace_back(1, 1, 3.1);
  A_child_triplets.emplace_back(2, 1, 1);
  A_child_triplets.emplace_back(2, 2, 1.2);
  Eigen::SparseMatrix<double> A_child(3, 3);
  A_child.setFromTriplets(A_child_triplets.begin(), A_child_triplets.end());
  AMatrix* scs_A_child = ConstructScsAmatrix(A_child);
  IsAmatrixEqual(scs_A_child, root.left_child()->A(), 1E-10);
  IsAmatrixEqual(scs_A_child, root.right_child()->A(), 1E-10);
  freeAMatrix(scs_A_child);

  // Check if the y_index and y_val are correct in the child nodes.
  EXPECT_EQ(root.left_child()->y_index(), 0);
  EXPECT_EQ(root.right_child()->y_index(), 0);
  EXPECT_EQ(root.left_child()->y_val(), 0);
  EXPECT_EQ(root.right_child()->y_val(), 1);
}

TEST_F(TestScsNode, TestSolve) {
  SCS_SETTINGS* settings = static_cast<SCS_SETTINGS*>(scs_malloc(sizeof(SCS_SETTINGS)));
  settings->alpha = ALPHA;
  settings->cg_rate = CG_RATE;
  settings->eps = EPS;
  settings->max_iters = MAX_ITERS;
  settings->normalize = NORMALIZE;
  settings->rho_x = RHO_X;
  settings->scale = SCALE;
  settings->verbose = VERBOSE;
  settings->warm_start = WARM_START;

  ScsNode root(scs_A_, b_, c_, binary_var_indices_, 1);

  root.Solve(cone_, settings, std::numeric_limits<double>::infinity());
  EXPECT_EQ(root.cost(), -std::numeric_limits<double>::infinity());

  scs_free(settings);
}
}  // namespace
}  // namespace solvers
}  // namespace drake