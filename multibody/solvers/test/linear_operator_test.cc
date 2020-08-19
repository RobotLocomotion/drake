#include "drake/multibody/solvers/linear_operator.h"

#include <memory>

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace solvers {
namespace {

using MatrixXd = Eigen::MatrixXd;
using SparseMatrixd = Eigen::SparseMatrix<double>;
using SparseVectord = Eigen::SparseVector<double>;
using VectorXd = Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;

// Implements the LinearOperator concept wrapping a dense Eigen matrix.
// It only implements DoMultiply() to verify that default implementations throw
// an exception.
template <typename T>
class DenseLinearOperator final : public LinearOperator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseLinearOperator)

  DenseLinearOperator(const std::string& name, const MatrixX<T>* A)
      : LinearOperator<T>(name), A_(A) {
    DRAKE_DEMAND(A != nullptr);
    tmp_.resize(A->cols());
  }

  ~DenseLinearOperator() = default;

  int rows() const { return A_->rows(); }
  int cols() const { return A_->cols(); }

 protected:
  void DoMultiply(const Eigen::Ref<const VectorX<T>>& x,
                  VectorX<T>* y) const final {
    *y = *A_ * x;
  };

  void DoMultiply(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                  Eigen::SparseVector<T>* y) const final {
    tmp_ = VectorX<T>(x);
    *y = (*A_ * tmp_).sparseView();
  }

 private:
  // scratch workspace for sparse operator implementation.
  mutable VectorX<T> tmp_;
  const MatrixX<T>* A_{nullptr};
};

GTEST_TEST(LinearOperator, Construction) {
  MatrixXd A = 2.0 * MatrixXd::Ones(3, 2);
  const DenseLinearOperator<double> Aop("A", &A);
  const LinearOperator<double>& Lop = Aop;
  EXPECT_EQ(Lop.name(), "A");
  EXPECT_EQ(Lop.rows(), 3);
  EXPECT_EQ(Lop.cols(), 2);
}

GTEST_TEST(LinearOperator, Multiply) {
  MatrixXd A = 2.0 * MatrixXd::Ones(3, 2);
  const DenseLinearOperator<double> Aop("A", &A);
  const LinearOperator<double>& Lop = Aop;
  const VectorXd x = Vector2<double>(1.0, 2.0);
  VectorXd y(3);
  Lop.Multiply(x, &y);
  const VectorXd y_expected = A * x;
  // y's values should equal those in y_expected bit by bit.
  EXPECT_EQ(y, y_expected);
}

GTEST_TEST(LinearOperator, MultiplyByTranspose) {
  MatrixXd A = 2.0 * MatrixXd::Ones(3, 2);
  const DenseLinearOperator<double> Aop("A", &A);
  const VectorXd x = Vector3<double>(1.0, 2.0, 3.0);
  VectorXd y(2);
  DRAKE_EXPECT_THROWS_MESSAGE(Aop.MultiplyByTranspose(x, &y),
                              std::runtime_error,
                              ".*must provide an implementation.");
}

GTEST_TEST(LinearOperator, AssembleMatrix) {
  MatrixXd A = 2.0 * MatrixXd::Ones(3, 2);
  const DenseLinearOperator<double> Aop("A", &A);
  SparseMatrixd Asparse(3, 2);
  DRAKE_EXPECT_THROWS_MESSAGE(Aop.AssembleMatrix(&Asparse), std::runtime_error,
                              ".*must provide an implementation.");
}

}  // namespace
}  // namespace solvers
}  // namespace multibody
}  // namespace drake
