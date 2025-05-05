#include "drake/multibody/contact_solvers/eigen_block_3x3_sparse_symmetric_matrix.h"

#include <Eigen/IterativeLinearSolvers>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using drake::multibody::contact_solvers::internal::
    Block3x3SparseSymmetricMatrix;
using drake::multibody::contact_solvers::internal::BlockSparsityPattern;
using drake::multibody::contact_solvers::internal::
    EigenBlock3x3SparseSymmetricMatrix;

// A small SPD test: a single 3x3 identity block.
GTEST_TEST(EigenBlockSparseWrapperTest, IdentitySingleBlock) {
  BlockSparsityPattern pattern({3}, {{0}});
  Block3x3SparseSymmetricMatrix A(pattern);
  A.SetBlock(0, 0, Eigen::Matrix3d::Identity());

  EigenBlock3x3SparseSymmetricMatrix A_eig(&A);

  Eigen::VectorXd b(3);
  b << 1.0, 2.0, 3.0;

  Eigen::ConjugateGradient<EigenBlock3x3SparseSymmetricMatrix,
                           Eigen::Lower | Eigen::Upper>
      cg;
  cg.compute(A_eig);
  EXPECT_EQ(cg.info(), Eigen::Success);

  Eigen::VectorXd x = cg.solve(b);
  EXPECT_EQ(cg.info(), Eigen::Success);
  EXPECT_TRUE(x.isApprox(b, 1e-10));
}

// A 6x6 block-diagonal SPD: [I 0; 0 2I]
GTEST_TEST(EigenBlockSparseWrapperTest, ScaledIdentityTwoBlock) {
  BlockSparsityPattern pattern({3, 3}, {{0}, {1}});
  Block3x3SparseSymmetricMatrix A(pattern);
  A.SetBlock(0, 0, Eigen::Matrix3d::Identity());
  A.SetBlock(1, 1, 2.0 * Eigen::Matrix3d::Identity());

  EigenBlock3x3SparseSymmetricMatrix A_eig(&A);

  Eigen::VectorXd b(6);
  b << 1, 2, 3, 4, 5, 6;

  Eigen::ConjugateGradient<EigenBlock3x3SparseSymmetricMatrix,
                           Eigen::Lower | Eigen::Upper,
                           Eigen::IdentityPreconditioner>
      cg;
  cg.compute(A_eig);
  EXPECT_EQ(cg.info(), Eigen::Success);

  Eigen::VectorXd x = cg.solve(b);
  EXPECT_EQ(cg.info(), Eigen::Success);

  Eigen::VectorXd expected(6);
  expected << 1, 2, 3, 2, 2.5, 3;
  EXPECT_TRUE(x.isApprox(expected, 1e-10));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
