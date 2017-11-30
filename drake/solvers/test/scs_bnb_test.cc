#include "drake/solvers/scs_bnb.h"

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

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

void free_scs_pointer(void* scs_pointer) { scs_free(scs_pointer); }

// Store the data for a mixed-inteter optimization problem
// min cᵀx + d
// s.t Ax + s = b
//     s in cone
//     x(binary_var_indices_) are binary variables
struct MIPdata {
  MIPdata(const Eigen::SparseMatrix<double>& A, const scs_float* const b,
          const scs_float* const c, double d, const SCS_CONE& cone,
          const std::list<int>& binary_var_indices)
      : A_{ConstructScsAmatrix(A)},
        b_{static_cast<scs_float*>(scs_calloc(A_->m, sizeof(scs_float))),
           &free_scs_pointer},
        c_{static_cast<scs_float*>(scs_calloc(A_->n, sizeof(scs_float))),
           &free_scs_pointer},
        d_{d},
        cone_{DeepCopyScsCone(&cone)},
        binary_var_indices_{binary_var_indices} {
    for (int i = 0; i < A_->m; ++i) {
      b_.get()[i] = b[i];
    }
    for (int i = 0; i < A_->n; ++i) {
      c_.get()[i] = c[i];
    }
  }
  std::unique_ptr<AMatrix, void (*)(AMatrix*)> A_;
  std::unique_ptr<scs_float, void (*)(void*)> b_;
  std::unique_ptr<scs_float, void (*)(void*)> c_;
  double d_;
  std::unique_ptr<SCS_CONE, void (*)(SCS_CONE*)> cone_;
  std::list<int> binary_var_indices_;
};

// Construct the problem data for the mixed-integer linear program
// min x(0) + 2x(1) -3x(3) + 1
// s.t x(0) + x(1) + 2x(3) = 2
//     x(1) - 3.1 x(2) >= 1
//     x(2) + 1.2x(3) - x(0) <= 5
//     x(0), x(2) are binary
MIPdata ConstructMILPExample1() {
  Eigen::Matrix<double, 3, 4> A;
  // clang-format off
  A << 1, 1, 0, 2,
       0, -1, 3.1, 0,
       -1, 0, 1, 1.2;
  // clang-format on
  scs_float b[3] = {2, -1, 5};
  scs_float c[4] = {1, 2, 0, -3};
  SCS_CONE cone;
  cone.f = 1;
  cone.l = 2;
  cone.q = nullptr;
  cone.qsize = 0;
  cone.s = nullptr;
  cone.ssize = 0;
  cone.ed = 0;
  cone.ep = 0;
  cone.p = nullptr;
  cone.psize = 0;
  return MIPdata(A.sparseView(), b, c, 1, cone, {0, 2});
}

// Construct the problem data for the mixed-integer linear program
// min x₀ + 2x₁ - 3x₂ - 4x₃ + 4.5x₄ + 1
// s.t 2x₀ + x₂ + 1.5x₃ + x₄ = 4.5
//     1 ≤ 2x₀ + 4x₃ + x₄ ≤ 7
//     -2 ≤ 3x₁ + 2x₂ - 5x₃ + x₄ ≤ 7
//     -5 ≤ x₁ + x₂ + 2x₃ ≤ 10
//     -10 ≤ x₁ ≤ 10
//     x₀, x₂, x₄ are binary variables.
MIPdata ConstructMILPExample2() {
  Eigen::Matrix<double, 9, 5> A;
  // clang-format off
  A << 2, 0, 1, 1.5, 1,
      2, 0, 0, 4, 1,
      -2, 0, 0, -4, -1,
      0, 3, 2, -5, 1,
      0, -3, -2, 5, -1,
      0, 1, 1, 2, 0,
      0, -1, -1, -2, 0,
      0, 1, 0, 0, 0,
      0, -1, 0, 0, 0;
  // clang-format on
  scs_float b[9] = {4.5, 7, -1, 7, 2, 10, 5, 10, 10};
  scs_float c[5] = {1, 2, -3, -4, 4.5};
  SCS_CONE cone;
  cone.f = 1;
  cone.l = 8;
  cone.q = nullptr;
  cone.qsize = 0;
  cone.s = nullptr;
  cone.ssize = 0;
  cone.ed = 0;
  cone.ep = 0;
  cone.p = nullptr;
  cone.psize = 0;
  return MIPdata(A.sparseView(), b, c, 1, cone, {0, 2, 4});
}

std::unique_ptr<ScsNode> ConstructMILPExample1RootNode() {
  MIPdata mip_data = ConstructMILPExample1();
  return ScsNode::ConstructRootNode(*(mip_data.A_), mip_data.b_.get(),
                                    mip_data.c_.get(), *(mip_data.cone_),
                                    mip_data.binary_var_indices_, mip_data.d_);
}

std::unique_ptr<ScsNode> ConstructMILPExample2RootNode() {
  MIPdata mip_data = ConstructMILPExample2();
  return ScsNode::ConstructRootNode(*(mip_data.A_), mip_data.b_.get(),
                                    mip_data.c_.get(), *(mip_data.cone_),
                                    mip_data.binary_var_indices_, mip_data.d_);
}

void SetScsSettingToDefault(SCS_SETTINGS* settings) {
  settings->alpha = ALPHA;
  settings->cg_rate = CG_RATE;
  settings->eps = EPS;
  settings->max_iters = MAX_ITERS;
  settings->normalize = NORMALIZE;
  settings->rho_x = RHO_X;
  settings->scale = SCALE;
  settings->verbose = VERBOSE;
  settings->warm_start = WARM_START;
}

GTEST_TEST(TestScsNode, TestConstructor) {
  ScsNode node(2, 3);
  EXPECT_EQ(node.A()->m, 2);
  EXPECT_EQ(node.A()->n, 3);
  EXPECT_EQ(node.y_index(), -1);
  EXPECT_EQ(node.y_val(), -1);
  EXPECT_TRUE(std::isnan(node.cost()));
  EXPECT_EQ(node.cost_constant(), 0);
  EXPECT_FALSE(node.found_integral_sol());
  EXPECT_TRUE(node.binary_var_indices().empty());
  EXPECT_EQ(node.left_child(), nullptr);
  EXPECT_EQ(node.right_child(), nullptr);
  EXPECT_EQ(node.parent(), nullptr);
}

void TestConstructScsRootNode(const AMatrix& A, const scs_float* const b,
                              const scs_float* const c, const SCS_CONE& cone,
                              const std::list<int>& binary_var_indices,
                              double cost_constant) {
  const auto root = ScsNode::ConstructRootNode(
      A, b, c, cone, binary_var_indices, cost_constant);
  EXPECT_EQ(root->y_index(), -1);
  EXPECT_EQ(root->left_child(), nullptr);
  EXPECT_EQ(root->right_child(), nullptr);
  EXPECT_EQ(root->parent(), nullptr);
  const Eigen::SparseMatrix<double> A_sparse = ScsAmatrixToEigenSparseMatrix(A);
  std::vector<Eigen::Triplet<double>> root_A_triplets;
  for (int i = 0; i < A_sparse.rows(); ++i) {
    for (int j = 0; j < A_sparse.cols(); ++j) {
      if (A_sparse.coeff(i, j) != 0) {
        root_A_triplets.emplace_back(
            i + (i >= cone.f ? 2 * binary_var_indices.size() : 0), j,
            A_sparse.coeff(i, j));
      }
    }
  }
  int binary_var_count = 0;
  for (auto it = binary_var_indices.begin(); it != binary_var_indices.end();
       ++it) {
    root_A_triplets.emplace_back(cone.f + binary_var_count * 2, *it, -1);
    root_A_triplets.emplace_back(cone.f + binary_var_count * 2 + 1, *it, 1);
    ++binary_var_count;
  }
  Eigen::SparseMatrix<double> root_A(A.m + 2 * binary_var_indices.size(), A.n);
  root_A.setFromTriplets(root_A_triplets.begin(), root_A_triplets.end());
  scs_float* root_b = new scs_float[A.m + 2 * binary_var_indices.size()];
  for (int i = 0; i < cone.f; ++i) {
    root_b[i] = b[i];
  }
  for (int i = 0; i < static_cast<int>(binary_var_indices.size()); ++i) {
    root_b[cone.f + 2 * i] = 0;
    root_b[cone.f + 1 + 2 * i] = 1;
  }
  for (int i = cone.f; i < A.m; ++i) {
    root_b[2 * binary_var_indices.size() + i] = b[i];
  }
  auto root_scs_A = ConstructScsAmatrix(root_A);

  IsSameRelaxedConstraint(*root_scs_A, *(root->A()), root_b, root->b(), 0);

  for (int i = 0; i < root_A.rows(); ++i) {
    EXPECT_EQ(root_b[i], root->b()[i]);
  }
  delete[] root_b;
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(c[i], root->c()[i]);
  }
  EXPECT_EQ(root->cost_constant(), cost_constant);
  // Check the cones
  auto root_cone_expected = DeepCopyScsCone(&cone);
  root_cone_expected->l += 2 * binary_var_indices.size();
  IsConeEqual(*(root->cone()), *root_cone_expected);

  EXPECT_FALSE(root->found_integral_sol());
  IsBinaryVarIndicesEqual(root->binary_var_indices(), binary_var_indices);
}

GTEST_TEST(TestScsNode, TestConstructRoot1) {
  MIPdata mip_data = ConstructMILPExample1();
  TestConstructScsRootNode(*(mip_data.A_), mip_data.b_.get(), mip_data.c_.get(),
                           *(mip_data.cone_), mip_data.binary_var_indices_,
                           mip_data.d_);
}

GTEST_TEST(TestScsNode, TestConstructRoot2) {
  MIPdata mip_data = ConstructMILPExample2();
  TestConstructScsRootNode(*(mip_data.A_), mip_data.b_.get(), mip_data.c_.get(),
                           *(mip_data.cone_), mip_data.binary_var_indices_,
                           mip_data.d_);
}

GTEST_TEST(TestScsNode, TestConstructRootError) {
  MIPdata mip_data = ConstructMILPExample1();
  EXPECT_THROW(ScsNode::ConstructRootNode(*(mip_data.A_), mip_data.b_.get(),
                                          mip_data.c_.get(), *(mip_data.cone_),
                                          {0, 4}, mip_data.d_),
               std::runtime_error);
  EXPECT_THROW(ScsNode::ConstructRootNode(*(mip_data.A_), mip_data.b_.get(),
                                          mip_data.c_.get(), *(mip_data.cone_),
                                          {-1, 0}, mip_data.d_),
               std::runtime_error);
  EXPECT_THROW(ScsNode::ConstructRootNode(*(mip_data.A_), mip_data.b_.get(),
                                          mip_data.c_.get(), *(mip_data.cone_),
                                          {1, 1, 4}, mip_data.d_),
               std::runtime_error);
}

void TestBranching(ScsNode* root, int branch_var_index,
                   const std::list<int>& binary_var_indices_child,
                   const scs_float* const b_left,
                   const scs_float* const b_right, const scs_float* c_child,
                   double cost_constant_left, double cost_constant_right,
                   const AMatrix& A_child) {
  root->Branch(branch_var_index);
  EXPECT_NE(root->left_child(), nullptr);
  EXPECT_NE(root->right_child(), nullptr);
  EXPECT_EQ(root->left_child()->parent(), root);
  EXPECT_EQ(root->right_child()->parent(), root);

  IsBinaryVarIndicesEqual(root->left_child()->binary_var_indices(),
                          binary_var_indices_child);
  IsBinaryVarIndicesEqual(root->right_child()->binary_var_indices(),
                          binary_var_indices_child);

  EXPECT_EQ(root->left_child()->A()->m, root->A()->m - 2);
  EXPECT_EQ(root->right_child()->A()->m, root->A()->m - 2);
  EXPECT_EQ(root->left_child()->A()->n, root->A()->n - 1);
  EXPECT_EQ(root->right_child()->A()->n, root->A()->n - 1);

  const double tol{1E-10};
  for (int i = 0; i < root->A()->n - 1; ++i) {
    EXPECT_NEAR(root->left_child()->c()[i], c_child[i], tol);
    EXPECT_NEAR(root->right_child()->c()[i], c_child[i], tol);
  }

  EXPECT_NEAR(root->left_child()->cost_constant(), cost_constant_left, tol);
  EXPECT_NEAR(root->right_child()->cost_constant(), cost_constant_right, tol);

  for (int i = 0; i < root->A()->m - 2; ++i) {
    EXPECT_EQ(root->left_child()->b()[i], b_left[i]);
    EXPECT_EQ(root->right_child()->b()[i], b_right[i]);
  }

  IsAmatrixEqual(A_child, *(root->left_child()->A()), tol);
  IsAmatrixEqual(A_child, *(root->right_child()->A()), tol);

  // Check if the y_index and y_val are correct in the child nodes.
  EXPECT_EQ(root->left_child()->y_index(), branch_var_index);
  EXPECT_EQ(root->right_child()->y_index(), branch_var_index);
  EXPECT_EQ(root->left_child()->y_val(), 0);
  EXPECT_EQ(root->right_child()->y_val(), 1);

  auto child_cone = DeepCopyScsCone(root->cone());
  child_cone->l -= 2;
  IsConeEqual(*child_cone, *(root->left_child()->cone()));
  IsConeEqual(*child_cone, *(root->right_child()->cone()));
}

GTEST_TEST(TestScsNode, TestBranch1) {
  const auto root = ConstructMILPExample1RootNode();

  // Branch on x0
  const scs_float b_left[5] = {2, 0, 1, -1, 5};
  const scs_float b_right[5] = {1, 0, 1, -1, 6};
  const scs_float c_child[3] = {2, 0, -3};
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
  const std::list<int> binary_var_indices_child = {1};

  TestBranching(root.get(), 0, binary_var_indices_child, b_left, b_right,
                c_child, 1, 2, *scs_A_child);
}

GTEST_TEST(TestScsNode, TestBranch2) {
  const auto root = ConstructMILPExample1RootNode();

  // Branch on x2
  const scs_float b_left[5] = {2, 0, 1, -1, 5};
  const scs_float b_right[5] = {2, 0, 1, -4.1, 4};
  const scs_float c_child[3] = {1, 2, -3};
  Eigen::Matrix<double, 5, 3> A;
  // clang-format off
  A << 1, 1, 2,
      -1, 0, 0,
      1, 0, 0,
      0, -1, 0,
      -1, 0, 1.2;
  // clang-format on
  const auto scs_A_child = ConstructScsAmatrix(A.sparseView());
  const std::list<int> binary_var_indices_child = {0};

  TestBranching(root.get(), 2, binary_var_indices_child, b_left, b_right,
                c_child, 1, 1, *scs_A_child);
}

GTEST_TEST(TestScsNode, TestBranch3) {
  const auto root = ConstructMILPExample2RootNode();

  // Branch on x0
  const scs_float b_left[13] = {4.5, 0, 1, 0, 1, 7, -1, 7, 2, 10, 5, 10, 10};
  const scs_float b_right[13] = {2.5, 0, 1, 0, 1, 5, 1, 7, 2, 10, 5, 10, 10};
  const scs_float c_child[4] = {2, -3, -4, 4.5};
  Eigen::Matrix<double, 13, 4> A;
  // clang-format off
  A << 0, 1, 1.5, 1,
      0, -1, 0, 0,
      0, 1, 0, 0,
      0, 0, 0, -1,
      0, 0, 0, 1,
      0, 0, 4, 1,
      0, 0, -4, -1,
      3, 2, -5, 1,
      -3, -2, 5, -1,
      1, 1, 2, 0,
      -1, -1, -2, 0,
      1, 0, 0, 0,
      -1, 0, 0, 0;
  // clang-format on
  const auto scs_A_child = ConstructScsAmatrix(A.sparseView());
  const std::list<int> binary_var_indices_child = {1, 3};

  TestBranching(root.get(), 0, binary_var_indices_child, b_left, b_right,
                c_child, 1, 2, *scs_A_child);
}

GTEST_TEST(TestScsNode, TestBranch4) {
  const auto root = ConstructMILPExample2RootNode();

  // Branch on x2
  const scs_float b_left[13] = {4.5, 0, 1, 0, 1, 7, -1, 7, 2, 10, 5, 10, 10};
  const scs_float b_right[13] = {3.5, 0, 1, 0, 1, 7, -1, 5, 4, 9, 6, 10, 10};
  const scs_float c_child[4] = {1, 2, -4, 4.5};
  Eigen::Matrix<double, 13, 4> A;
  // clang-format off
  A << 2, 0, 1.5, 1,
      -1, 0, 0, 0,
      1, 0, 0, 0,
      0, 0, 0, -1,
      0, 0, 0, 1,
      2, 0, 4, 1,
      -2, 0, -4, -1,
      0, 3, -5, 1,
      0, -3, 5, -1,
      0, 1, 2, 0,
      0, -1, -2, 0,
      0, 1, 0, 0,
      0, -1, 0, 0;
  // clang-format on
  const auto scs_A_child = ConstructScsAmatrix(A.sparseView());
  const std::list<int> binary_var_indices_child = {0, 3};

  TestBranching(root.get(), 2, binary_var_indices_child, b_left, b_right,
                c_child, 1, -2, *scs_A_child);
}

GTEST_TEST(TestScsNode, TestBranch5) {
  const auto root = ConstructMILPExample2RootNode();

  // Branch on x4
  const scs_float b_left[13] = {4.5, 0, 1, 0, 1, 7, -1, 7, 2, 10, 5, 10, 10};
  const scs_float b_right[13] = {3.5, 0, 1, 0, 1, 6, 0, 6, 3, 10, 5, 10, 10};
  const scs_float c_child[4] = {1, 2, -3, -4};
  Eigen::Matrix<double, 13, 4> A;
  // clang-format off
  A << 2, 0, 1, 1.5,
      -1, 0, 0, 0,
      1, 0, 0, 0,
      0, 0, -1, 0,
      0, 0, 1, 0,
      2, 0, 0, 4,
      -2, 0, 0, -4,
      0, 3, 2, -5,
      0, -3, -2, 5,
      0, 1, 1, 2,
      0, -1, -1, -2,
      0, 1, 0, 0,
      0, -1, 0, 0;
  // clang-format on
  const auto scs_A_child = ConstructScsAmatrix(A.sparseView());
  const std::list<int> binary_var_indices_child = {0, 2};

  TestBranching(root.get(), 4, binary_var_indices_child, b_left, b_right,
                c_child, 1, 5.5, *scs_A_child);
}

GTEST_TEST(TestScsNode, TestBranchError) {
  const auto root = ConstructMILPExample1RootNode();

  // Branch on a variable that is NOT binary.
  EXPECT_THROW(root->Branch(1), std::runtime_error);
}

GTEST_TEST(TestScsNode, TestSolve1) {
  const auto root = ConstructMILPExample1RootNode();

  SCS_SETTINGS settings;
  SetScsSettingToDefault(&settings);
  const double tol = 1E-3;
  const scs_int scs_status = root->Solve(settings);
  EXPECT_EQ(scs_status, SCS_SOLVED);
  EXPECT_NEAR(root->cost(), 1.5, tol);
  const scs_float x_expected[4] = {0, 1, 0, 0.5};
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(x_expected[i], root->scs_sol()->x[i], tol);
  }
  EXPECT_TRUE(root->found_integral_sol());
}

GTEST_TEST(TestScsNode, TestSolveChildNodes1) {
  // Solve the left and right child nodes of the root.
  const auto root = ConstructMILPExample1RootNode();

  root->Branch(0);

  SCS_SETTINGS settings;
  SetScsSettingToDefault(&settings);
  const scs_int scs_status_l = root->left_child()->Solve(settings);
  EXPECT_EQ(scs_status_l, SCS_SOLVED);
  const double tol{1E-3};
  EXPECT_NEAR(root->left_child()->cost(), 1.5, tol);
  const scs_float x_expected_l[3] = {1, 0, 0.5};
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(x_expected_l[i], root->left_child()->scs_sol()->x[i], tol);
  }
  EXPECT_TRUE(root->left_child()->found_integral_sol());

  const scs_int scs_status_r = root->right_child()->Solve(settings);
  EXPECT_EQ(scs_status_r, SCS_SOLVED);
  EXPECT_NEAR(root->right_child()->cost(), 4, 2 * tol);
  const scs_float x_expected_r[3] = {1, 0, 0};
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(x_expected_r[i], root->right_child()->scs_sol()->x[i], tol);
  }
  EXPECT_TRUE(root->right_child()->found_integral_sol());
}
}  // namespace
}  // namespace solvers
}  // namespace drake
