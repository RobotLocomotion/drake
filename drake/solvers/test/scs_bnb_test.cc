#include "drake/solvers/scs_bnb.h"

#include <gtest/gtest.h>

#include <Eigen/SparseCore>

namespace drake {
namespace solvers {
namespace {
std::unique_ptr<AMatrix, void (*)(AMatrix*)> ConstructScsAmatrix(
    const Eigen::SparseMatrix<double>& A) {
  AMatrix* scs_A = static_cast<AMatrix*>(malloc(sizeof(AMatrix)));
  scs_A->m = A.rows();
  scs_A->n = A.cols();
  scs_A->x =
      static_cast<scs_float*>(scs_calloc(A.nonZeros(), sizeof(scs_float)));
  scs_A->i = static_cast<scs_int*>(scs_calloc(A.nonZeros(), sizeof(scs_int)));
  scs_A->p = static_cast<scs_int*>(scs_calloc(scs_A->n + 1, sizeof(scs_int)));
  for (int i = 0; i < A.nonZeros(); ++i) {
    scs_A->x[i] = *(A.valuePtr() + i);
    scs_A->i[i] = *(A.innerIndexPtr() + i);
  }
  for (int i = 0; i < scs_A->n + 1; ++i) {
    scs_A->p[i] = *(A.outerIndexPtr() + i);
  }
  return std::unique_ptr<AMatrix, void (*)(AMatrix*)>(scs_A, &freeAMatrix);
}

Eigen::SparseMatrix<double> ScsAmatrixToEigenSparseMatrix(
    const AMatrix& scs_A) {
  Eigen::SparseMatrix<double> A(scs_A.m, scs_A.n);
  A.reserve(scs_A.p[scs_A.n]);
  A.setZero();
  for (int j = 0; j < scs_A.n; ++j) {
    for (int i = scs_A.p[j]; i < scs_A.p[j + 1]; ++i) {
      A.insert(scs_A.i[i], j) = scs_A.x[i];
    }
  }
  A.makeCompressed();
  return A;
}

void IsAmatrixEqual(const AMatrix& A1, const AMatrix& A2, double tol) {
  EXPECT_EQ(A1.m, A2.m);
  EXPECT_EQ(A1.n, A2.n);
  for (int i = 0; i < A1.n + 1; ++i) {
    EXPECT_EQ(A1.p[i], A2.p[i]);
  }
  for (int i = 0; i < A1.p[A1.n]; ++i) {
    EXPECT_NEAR(A1.x[i], A2.x[i], tol);
    EXPECT_EQ(A1.i[i], A2.i[i]);
  }
}

void IsBinaryVarIndicesEqual(const std::list<int>& indices1,
                             const std::list<int>& indices2) {
  EXPECT_EQ(indices1.size(), indices2.size());
  auto it2 = indices2.begin();
  for (auto it1 = indices1.begin(); it1 != indices1.end(); ++it1) {
    EXPECT_EQ(*it1, *it2);
    ++it2;
  }
}

// Determine if the two constraints
// A1*x+s = b1
// and
// A2*x+s = b2
// are the same constraints.
// A1,A2,b1 and b2 are obtained by relaxing the constraint
// A*x + s = b
// y ∈ {0, 1}
void IsSameRelaxedConstraint(const AMatrix& A1, const AMatrix& A2,
                             const scs_float* const b1, const scs_float* b2,
                             double tol) {
  EXPECT_EQ(A1.m, A2.m);
  EXPECT_EQ(A1.n, A2.n);
  for (int i = 0; i < A1.n + 1; ++i) {
    EXPECT_EQ(A1.p[i], A2.p[i]);
  }
  for (int i = 0; i < A1.p[A1.n]; ++i) {
    EXPECT_EQ(A1.i[i], A2.i[i]);
    EXPECT_NEAR(A1.x[i], A2.x[i], tol);
  }
  for (int i = 0; i < A1.m; ++i) {
    EXPECT_NEAR(b1[i], b2[i], tol);
  }
}

GTEST_TEST(TestSparseMatrix, TestConversion) {
  // Test if ScsAmatrixToEigenSparseMatrix is the inversion of
  // ConstructScsAmatrix
  std::vector<Eigen::SparseMatrix<double>> X;
  Eigen::SparseMatrix<double> Xi(2, 2);
  Xi.setZero();
  X.push_back(Xi);
  Xi.setIdentity();
  X.push_back(Xi);
  Xi.setZero();
  Xi.insert(0, 0) = 1;
  X.push_back(Xi);
  Xi.setZero();
  Xi.insert(1, 0) = 2;
  Xi.insert(0, 1) = 3;
  X.push_back(Xi);
  for (const auto& Xi : X) {
    EXPECT_TRUE(Xi.isApprox(
        ScsAmatrixToEigenSparseMatrix(*ConstructScsAmatrix(Xi)), 1E-10));
  }
}

void freeCone(SCS_CONE* cone) {
  if (cone) {
    if (cone->q) {
      scs_free(cone->q);
    }
    if (cone->s) {
      scs_free(cone->s);
    }
    if (cone->p) {
      scs_free(cone->p);
    }
    scs_free(cone);
  }
}

std::unique_ptr<SCS_CONE, void (*)(SCS_CONE*)> DeepCopyScsCone(
    const SCS_CONE* const cone) {
  SCS_CONE* new_cone = static_cast<SCS_CONE*>(scs_calloc(1, sizeof(SCS_CONE)));
  new_cone->f = cone->f;
  new_cone->l = cone->l;
  new_cone->qsize = cone->qsize;
  if (cone->q) {
    new_cone->q =
        static_cast<scs_int*>(scs_calloc(new_cone->qsize, sizeof(scs_int)));
    for (int i = 0; i < new_cone->qsize; ++i) {
      new_cone->q[i] = cone->q[i];
    }
  } else {
    new_cone->q = nullptr;
  }
  new_cone->ssize = cone->ssize;
  if (cone->s) {
    new_cone->s =
        static_cast<scs_int*>(scs_calloc(new_cone->ssize, sizeof(scs_int)));
    for (int i = 0; i < new_cone->ssize; ++i) {
      new_cone->s[i] = cone->s[i];
    }
  } else {
    new_cone->s = nullptr;
  }
  new_cone->ep = cone->ep;
  new_cone->ed = cone->ed;
  new_cone->psize = cone->psize;
  if (cone->p) {
    new_cone->p =
        static_cast<scs_float*>(scs_calloc(new_cone->psize, sizeof(scs_float)));
    for (int i = 0; i < new_cone->psize; ++i) {
      new_cone->p[i] = cone->p[i];
    }
  } else {
    new_cone->p = nullptr;
  }
  return std::unique_ptr<SCS_CONE, void (*)(SCS_CONE*)>(new_cone, &freeCone);
}

void IsConeEqual(const SCS_CONE& cone1, const SCS_CONE& cone2) {
  EXPECT_EQ(cone1.f, cone2.f);
  EXPECT_EQ(cone1.l, cone2.l);
  EXPECT_EQ(cone1.qsize, cone2.qsize);
  if (cone1.q) {
    EXPECT_NE(cone2.q, nullptr);
    for (int i = 0; i < cone1.qsize; ++i) {
      EXPECT_EQ(cone1.q[i], cone2.q[i]);
    }
  } else {
    EXPECT_EQ(cone2.q, nullptr);
  }
  EXPECT_EQ(cone1.ssize, cone2.ssize);
  if (cone1.s) {
    EXPECT_NE(cone2.s, nullptr);
    for (int i = 0; i < cone1.ssize; ++i) {
      EXPECT_EQ(cone1.s[i], cone2.s[i]);
    }
  } else {
    EXPECT_EQ(cone2.s, nullptr);
  }
  EXPECT_EQ(cone1.ep, cone2.ep);
  EXPECT_EQ(cone1.ed, cone2.ed);
  EXPECT_EQ(cone1.psize, cone2.psize);
  if (cone1.p) {
    EXPECT_NE(cone2.s, nullptr);
    for (int i = 0; i < cone1.psize; ++i) {
      EXPECT_EQ(cone1.p[i], cone2.p[i]);
    }
  } else {
    EXPECT_EQ(cone2.p, nullptr);
  }
}

class TestScsNode : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestScsNode)

  TestScsNode() : A_(3, 4), scs_A_{nullptr, &freeAMatrix} {
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
    A_.setFromTriplets(A_triplets.begin(), A_triplets.end());
    A_.makeCompressed();
    scs_A_ = ConstructScsAmatrix(A_);
    binary_var_indices_ = {0, 2};

    cone_ = static_cast<SCS_CONE*>(scs_calloc(1, sizeof(SCS_CONE)));
    cone_->f = 1;
    cone_->l = 2;
    cone_->q = nullptr;
    cone_->qsize = 0;
    cone_->s = nullptr;
    cone_->ssize = 0;
    cone_->ep = 0;
    cone_->ed = 0;
    cone_->p = nullptr;
    cone_->psize = 0;
  }

  ~TestScsNode() {
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

  void TestConstructRootNode(const std::list<int>& binary_var_indices) {
    const auto root = ScsNode::ConstructRootNode(*scs_A_, b_, c_, *cone_,
                                                 binary_var_indices, 1);
    EXPECT_EQ(root->y_index(), -1);
    EXPECT_EQ(root->left_child(), nullptr);
    EXPECT_EQ(root->right_child(), nullptr);
    EXPECT_EQ(root->parent(), nullptr);
    Eigen::SparseMatrix<double> root_A(
        A_.rows() + 2 * binary_var_indices.size(), A_.cols());
    std::vector<Eigen::Triplet<double>> root_A_triplets;
    for (int i = 0; i < A_.rows(); ++i) {
      for (int j = 0; j < A_.cols(); ++j) {
        if (A_.coeff(i, j) != 0) {
          root_A_triplets.emplace_back(
              i + (i >= 1 ? 2 * binary_var_indices.size() : 0), j,
              A_.coeff(i, j));
        }
      }
    }
    int binary_var_count = 0;
    for (auto it = binary_var_indices.begin(); it != binary_var_indices.end();
         ++it) {
      root_A_triplets.emplace_back(1 + binary_var_count * 2, *it, -1);
      root_A_triplets.emplace_back(1 + binary_var_count * 2 + 1, *it, 1);
      ++binary_var_count;
    }
    root_A.setFromTriplets(root_A_triplets.begin(), root_A_triplets.end());
    scs_float* root_b = new scs_float[3 + 2 * binary_var_indices.size()];
    root_b[0] = b_[0];
    for (int i = 0; i < static_cast<int>(binary_var_indices.size()); ++i) {
      root_b[1 + 2 * i] = 0;
      root_b[2 + 2 * i] = 1;
    }
    root_b[2 * binary_var_indices.size() + 1] = b_[1];
    root_b[2 * binary_var_indices.size() + 2] = b_[2];
    auto root_scs_A = ConstructScsAmatrix(root_A);

    IsSameRelaxedConstraint(*root_scs_A, *(root->A()), root_b, root->b(), 0);

    for (int i = 0; i < scs_A_->m + 2 * binary_var_indices.size(); ++i) {
      EXPECT_EQ(root_b[i], root->b()[i]);
    }
    delete[] root_b;
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(c_[i], root->c()[i]);
    }
    EXPECT_EQ(root->cost_constant(), 1);
    // Check the cones
    auto root_cone_expected = DeepCopyScsCone(cone_);
    root_cone_expected->l += 2 * binary_var_indices.size();
    IsConeEqual(*(root->cone()), *root_cone_expected);

    EXPECT_FALSE(root->found_integral_sol());
    EXPECT_FALSE(root->larger_than_upper_bound());
    IsBinaryVarIndicesEqual(root->binary_var_indices(), binary_var_indices);
  }

 protected:
  Eigen::SparseMatrix<double> A_;
  std::unique_ptr<AMatrix, void (*)(AMatrix*)> scs_A_;
  scs_float b_[3] = {2, -1, 5};
  scs_float c_[4] = {1, 2, 0, -3};
  std::list<int> binary_var_indices_;
  SCS_CONE* cone_;
};

TEST_F(TestScsNode, TestConstructor) {
  ScsNode node(2, 3);
  EXPECT_EQ(node.A()->m, 2);
  EXPECT_EQ(node.A()->n, 3);
  EXPECT_EQ(node.y_index(), -1);
  EXPECT_EQ(node.y_val(), -1);
  EXPECT_TRUE(std::isnan(node.cost()));
  EXPECT_EQ(node.cost_constant(), 0);
  EXPECT_FALSE(node.found_integral_sol());
  EXPECT_FALSE(node.larger_than_upper_bound());
  EXPECT_TRUE(node.binary_var_indices().empty());
  EXPECT_EQ(node.left_child(), nullptr);
  EXPECT_EQ(node.right_child(), nullptr);
}

TEST_F(TestScsNode, TestConstructRoot1) {
  TestConstructRootNode(binary_var_indices_);
}

TEST_F(TestScsNode, TestConstructRoot2) { TestConstructRootNode({0}); }

TEST_F(TestScsNode, TestConstructRoot3) { TestConstructRootNode({1}); }

TEST_F(TestScsNode, TestConstructRoot4) { TestConstructRootNode({3}); }

TEST_F(TestScsNode, TestConstructRootError) {
  EXPECT_THROW(ScsNode::ConstructRootNode(*scs_A_, b_, c_, *cone_, {0, 4}, 1),
               std::runtime_error);
  EXPECT_THROW(ScsNode::ConstructRootNode(*scs_A_, b_, c_, *cone_, {-1, 0}, 1),
               std::runtime_error);
  EXPECT_THROW(
      ScsNode::ConstructRootNode(*scs_A_, b_, c_, *cone_, {1, 1, 2}, 1),
      std::runtime_error);
}

TEST_F(TestScsNode, TestBranch) {
  auto root = ScsNode::ConstructRootNode(*scs_A_, b_, c_, *cone_,
                                         binary_var_indices_, 1);

  // Branch on x0
  root->Branch(0);
  EXPECT_NE(root->left_child(), nullptr);
  EXPECT_NE(root->right_child(), nullptr);
  EXPECT_EQ(root->left_child()->parent(), root.get());
  EXPECT_EQ(root->right_child()->parent(), root.get());

  const std::list<int> binary_var_indices_child = {1};
  IsBinaryVarIndicesEqual(root->left_child()->binary_var_indices(),
                          binary_var_indices_child);
  IsBinaryVarIndicesEqual(root->right_child()->binary_var_indices(),
                          binary_var_indices_child);

  // Check the cost vector c
  const scs_float c_child[3] = {2, 0, -3};
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(root->left_child()->c()[i], c_child[i]);
    EXPECT_EQ(root->right_child()->c()[i], c_child[i]);
  }
  EXPECT_EQ(root->left_child()->cost_constant(), 1);
  EXPECT_EQ(root->right_child()->cost_constant(), 2);

  // Check the right-hand side vector b
  const scs_float b_left[5] = {2, 0, 1, -1, 5};
  const scs_float b_right[5] = {1, 0, 1, -1, 6};
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(root->left_child()->b()[i], b_left[i]);
    EXPECT_EQ(root->right_child()->b()[i], b_right[i]);
  }

  // Check the left-hand side matrix A
  // A_child = [ 1    0    2]
  //           [ 0   -1    0]
  //           [ 0    1    0]
  //           [-1  3.1    0]
  //           [ 0    1  1.2]
  std::vector<Eigen::Triplet<double>> A_child_triplets;
  A_child_triplets.emplace_back(0, 0, 1);
  A_child_triplets.emplace_back(0, 2, 2);
  A_child_triplets.emplace_back(1, 1, -1);
  A_child_triplets.emplace_back(2, 1, 1);
  A_child_triplets.emplace_back(3, 0, -1);
  A_child_triplets.emplace_back(3, 1, 3.1);
  A_child_triplets.emplace_back(4, 1, 1);
  A_child_triplets.emplace_back(4, 2, 1.2);
  Eigen::SparseMatrix<double> A_child(5, 3);
  A_child.setFromTriplets(A_child_triplets.begin(), A_child_triplets.end());
  const auto scs_A_child = ConstructScsAmatrix(A_child);
  IsAmatrixEqual(*scs_A_child, *(root->left_child()->A()), 1E-10);
  IsAmatrixEqual(*scs_A_child, *(root->right_child()->A()), 1E-10);

  // Check if the y_index and y_val are correct in the child nodes.
  EXPECT_EQ(root->left_child()->y_index(), 0);
  EXPECT_EQ(root->right_child()->y_index(), 0);
  EXPECT_EQ(root->left_child()->y_val(), 0);
  EXPECT_EQ(root->right_child()->y_val(), 1);

  auto child_cone = DeepCopyScsCone(root->cone());
  child_cone->l -= 2;
  IsConeEqual(*child_cone, *(root->left_child()->cone()));
  IsConeEqual(*child_cone, *(root->right_child()->cone()));
}
/*
TEST_F(TestScsNode, TestSolve) {
  SCS_SETTINGS* settings =
static_cast<SCS_SETTINGS*>(scs_malloc(sizeof(SCS_SETTINGS)));
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
}*/
}  // namespace
}  // namespace solvers
}  // namespace drake