/* clang-format off to disable clang-format-includes */
#include "drake/common/symbolic/expression/all.h"
/* clang-format on */

#include <vector>

#include <Eigen/Sparse>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;

// This is a regression test for drake#18218.
GTEST_TEST(SymbolicMatricesTest, SparseMatrices) {
  const Variable x("x");
  std::vector<Eigen::Triplet<Expression>> triplets;
  triplets.push_back(Eigen::Triplet<Expression>(1, 1, 1.1));
  triplets.push_back(Eigen::Triplet<Expression>(2, 2, x * 2.0));
  Eigen::SparseMatrix<Expression> M(3, 3);
  M.setFromTriplets(triplets.begin(), triplets.end());
  EXPECT_PRED2(ExprEqual, M.coeff(0, 0), 0.0);
  EXPECT_PRED2(ExprEqual, M.coeff(1, 1), 1.1);
  EXPECT_PRED2(ExprEqual, M.coeff(2, 2), x * 2.0);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
