#include "drake/multibody/contact_solvers/linear_operator.h"

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using Eigen::MatrixXd;
using SparseMatrixd = Eigen::SparseMatrix<double>;
using SparseVectord = Eigen::SparseVector<double>;
using Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;

// Implements a LinearOperator for testing purposes only.
// We want to test that LinearOperator for:
//   - Storing and reporting a name.
//   - Its NVI methods:
//   -   Validate what it claims to validate.
//   -   Successfully call the DoFoo() methods.
// TODO(amcastro-tri): Add unit tests to verify that LinearOperator's NVIs
// validate what they promise to validate (sizes etc.).
template <typename T>
class TestLinearOperator final : public LinearOperator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestLinearOperator)

  explicit TestLinearOperator(const std::string& name)
      : LinearOperator<T>(name) {}

  ~TestLinearOperator() = default;

  int rows() const final { return 3; }
  int cols() const final { return 2; }

  VectorX<T> ExpectedMultiplyResult() const {
    return Vector3<T>(1.0, 2.0, 3.0);
  }

 protected:
  void DoMultiply(const Eigen::Ref<const VectorX<T>>& x,
                  VectorX<T>* y) const final {
    *y = ExpectedMultiplyResult();
  };

  void DoMultiply(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                  Eigen::SparseVector<T>* y) const final {
    *y = ExpectedMultiplyResult().sparseView();
  }
};

GTEST_TEST(LinearOperator, Basics) {
  const TestLinearOperator<double> Aop("A");
  EXPECT_EQ(Aop.name(), "A");
  EXPECT_EQ(Aop.rows(), 3);
  EXPECT_EQ(Aop.cols(), 2);
}

GTEST_TEST(LinearOperator, MultiplyDense) {
  const TestLinearOperator<double> Aop("A");
  VectorXd y(3);
  Aop.Multiply(VectorXd(2), &y);
  // y's values should equal those in y_expected bit by bit.
  EXPECT_EQ(y, Aop.ExpectedMultiplyResult());
}

GTEST_TEST(LinearOperator, MultiplySparse) {
  const TestLinearOperator<double> Aop("A");
  SparseVectord y(3);
  Aop.Multiply(SparseVectord(2), &y);
  // y's values should equal those in y_expected bit by bit.
  EXPECT_EQ(VectorXd(y), Aop.ExpectedMultiplyResult());
}

GTEST_TEST(LinearOperator, MultiplyByTransposeDense) {
  const TestLinearOperator<double> Aop("A");
  VectorXd y(2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Aop.MultiplyByTranspose(VectorXd(3), &y), std::exception,
      "DoMultiplyByTranspose().*must provide an implementation.");
}

GTEST_TEST(LinearOperator, MultiplyByTransposeSparse) {
  const TestLinearOperator<double> Aop("A");
  SparseVectord y(2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Aop.MultiplyByTranspose(SparseVectord(3), &y), std::exception,
      "DoMultiplyByTranspose().*must provide an implementation.");
}

GTEST_TEST(LinearOperator, AssembleMatrixSparse) {
  const TestLinearOperator<double> Aop("A");
  SparseMatrixd Asparse(3, 2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Aop.AssembleMatrix(&Asparse), std::exception,
      "DoAssembleMatrix().*must provide an implementation.");
}

GTEST_TEST(LinearOperator, AssembleMatrixBlockSparse) {
  const TestLinearOperator<double> Aop("A");
  BlockSparseMatrix<double> Ablock;
  DRAKE_EXPECT_THROWS_MESSAGE(
      Aop.AssembleMatrix(&Ablock), std::exception,
      "DoAssembleMatrix().*must provide an implementation.");
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
