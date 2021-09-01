#include "drake/math/linear_solve.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace math {
namespace {

template <template <typename> typename LinearSolverType, typename DerivedA,
          typename DerivedB>
void TestLinearSolve(const Eigen::MatrixBase<DerivedA>& A,
                     const Eigen::MatrixBase<DerivedB>& b) {
  const auto x = LinearSolve<LinearSolverType>(A, b);
  if constexpr (std::is_same_v<typename DerivedA::Scalar, double> &&
                std::is_same_v<typename DerivedB::Scalar, double>) {
    static_assert(std::is_same_v<typename decltype(x)::Scalar, double>,
                  "The returned  x should have scalar type = double.");
  } else {
    static_assert(!std::is_same_v<typename decltype(x)::Scalar, double>,
                  "The returned  x should have scalar type as AutoDiffScalar.");
  }
  // Now check Ax = z and A*∂x/∂z + ∂A/∂z * x = ∂b/∂z
  const auto Ax = A * x;
  Eigen::VectorXd Ax_val, b_val;
  Eigen::MatrixXd Ax_grad, b_grad;
  if constexpr (std::is_same_v<typename decltype(Ax)::Scalar, double>) {
    Ax_val = Ax;
    Ax_grad = Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 0>::Zero(
        Ax.rows(), 0);
  } else {
    Ax_val = autoDiffToValueMatrix(Ax);
    Ax_grad = autoDiffToGradientMatrix(Ax);
  }

  if constexpr (std::is_same_v<typename DerivedB::Scalar, double>) {
    b_val = b;
    b_grad = Eigen::Matrix<double, DerivedB::RowsAtCompileTime, 0>::Zero(
        b.rows(), 0);
  } else {
    b_val = autoDiffToValueMatrix(b);
    b_grad = autoDiffToGradientMatrix(b);
  }
  const double tol = 2E-13;
  EXPECT_TRUE(CompareMatrices(Ax_val, b_val, tol));
  if (b_grad.size() == 0 && Ax_grad.size() == 0) {
  } else if (b_grad.size() != 0 && Ax_grad.size() == 0) {
    EXPECT_TRUE(CompareMatrices(
        b_grad, Eigen::MatrixXd::Zero(b_grad.rows(), b_grad.cols()), tol));
  } else if (b_grad.size() == 0 && Ax_grad.size() != 0) {
    EXPECT_TRUE(CompareMatrices(
        Ax_grad, Eigen::MatrixXd::Zero(Ax_grad.rows(), Ax_grad.cols()), tol));
  } else {
    EXPECT_TRUE(CompareMatrices(Ax_grad, b_grad, tol));
  }
}

class LinearSolveTest : public ::testing::Test {
 public:
  LinearSolveTest() {
    A_val_ << 1, 3, 3, 10;
    b_val_ << 3, 5;
    Eigen::Matrix<double, 2, Eigen::Dynamic> b_grad(2, 3);
    b_grad << 1, 2, 3, 4, 5, 6;
    b_ad_ = initializeAutoDiffGivenGradientMatrix(b_val_, b_grad);
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        A_ad_(i, j).value() = A_val_(i, j);
      }
    }
    A_ad_(0, 0).derivatives() = Eigen::Vector3d(1, 2, 3);
    A_ad_(0, 1).derivatives() = Eigen::Vector3d(4, 5, 6);
    A_ad_(1, 0).derivatives() = Eigen::Vector3d(7, 8, 9);
    A_ad_(1, 1).derivatives() = Eigen::Vector3d(10, 11, 12);
  }

 protected:
  Eigen::Matrix2d A_val_;
  Eigen::Vector2d b_val_;
  Eigen::Matrix<AutoDiffXd, 2, 2> A_ad_;
  Eigen::Matrix<AutoDiffXd, 2, 1> b_ad_;
};

TEST_F(LinearSolveTest, TestDoubleAandb) {
  // Both A and b are double matrices.
  TestLinearSolve<Eigen::LLT>(A_val_, b_val_);
  TestLinearSolve<Eigen::LDLT>(A_val_, b_val_);
  TestLinearSolve<Eigen::ColPivHouseholderQR>(A_val_, b_val_);
  TestLinearSolve<Eigen::PartialPivLU>(A_val_, b_val_);
}

TEST_F(LinearSolveTest, TestAutoDiffAandDoubleB) {
  // A contains AutoDiffXd and b contains double.
  TestLinearSolve<Eigen::LLT>(A_ad_, b_val_);
  TestLinearSolve<Eigen::LDLT>(A_ad_, b_val_);
  TestLinearSolve<Eigen::ColPivHouseholderQR>(A_ad_, b_val_);
  TestLinearSolve<Eigen::PartialPivLU>(A_ad_, b_val_);
}

TEST_F(LinearSolveTest, TestDoubleAandAutoDiffB) {
  // A contains double and b contains AutoDiffXd.
  TestLinearSolve<Eigen::LLT>(A_val_, b_ad_);
  TestLinearSolve<Eigen::LDLT>(A_val_, b_ad_);
  TestLinearSolve<Eigen::ColPivHouseholderQR>(A_val_, b_ad_);
  TestLinearSolve<Eigen::PartialPivLU>(A_val_, b_ad_);
}

TEST_F(LinearSolveTest, TestNoGrad) {
  // A and b both contain AutoDiffXd but has empty gradient.
  TestLinearSolve<Eigen::LLT>(A_val_.cast<AutoDiffXd>(),
                              b_val_.cast<AutoDiffXd>());
}

TEST_F(LinearSolveTest, TestBwithGrad) {
  // Test LinearSolve with A containing empty gradient while b
  // contains meaningful gradient.
  TestLinearSolve<Eigen::LLT>(A_val_.cast<AutoDiffXd>(), b_ad_);
  TestLinearSolve<Eigen::LDLT>(A_val_.cast<AutoDiffXd>(), b_ad_);
  TestLinearSolve<Eigen::ColPivHouseholderQR>(A_val_.cast<AutoDiffXd>(), b_ad_);
  TestLinearSolve<Eigen::PartialPivLU>(A_val_.cast<AutoDiffXd>(), b_ad_);
}

TEST_F(LinearSolveTest, TestAwithGrad) {
  // Test LinearSolve with A containing gradient while b contains
  // no gradient.
  TestLinearSolve<Eigen::LLT>(A_ad_, b_val_.cast<AutoDiffXd>());
  TestLinearSolve<Eigen::LDLT>(A_ad_, b_val_.cast<AutoDiffXd>());
  TestLinearSolve<Eigen::ColPivHouseholderQR>(A_ad_, b_val_.cast<AutoDiffXd>());
  TestLinearSolve<Eigen::PartialPivLU>(A_ad_, b_val_.cast<AutoDiffXd>());
}

TEST_F(LinearSolveTest, TestAbWithGrad) {
  // Test LinearSolve with both A and b containing gradient.
  TestLinearSolve<Eigen::LLT>(A_ad_, b_ad_);
  TestLinearSolve<Eigen::LDLT>(A_ad_, b_ad_);
  TestLinearSolve<Eigen::ColPivHouseholderQR>(A_ad_, b_ad_);
  TestLinearSolve<Eigen::PartialPivLU>(A_ad_, b_ad_);
}

TEST_F(LinearSolveTest, TestAbWithMaybeEmptyGrad) {
  // Test LinearSolve with both A and b containing gradient in
  // some entries, and empty gradient in some other entries.
  A_ad_(1, 0).derivatives() = Eigen::VectorXd(0);
  b_ad_(1).derivatives() = Eigen::VectorXd(0);
  TestLinearSolve<Eigen::LLT>(A_ad_, b_ad_);
  TestLinearSolve<Eigen::LDLT>(A_ad_, b_ad_);
}

TEST_F(LinearSolveTest, TestWrongGradientSize) {
  const Eigen::LLT<Eigen::Matrix2d> linear_solver(A_val_);
  // A's gradient has inconsistent size.
  auto A_ad_error = A_ad_;
  A_ad_error(0, 1).derivatives() = Eigen::Vector2d(1, 2);
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSolve<Eigen::LLT>(A_ad_error, b_ad_),
                              ".* has size 2, while another entry has size 3");
  // b's gradient has inconsistent size.
  auto b_ad_error = b_ad_;
  b_ad_error(1).derivatives() = Eigen::Vector2d(1, 2);
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSolve<Eigen::LLT>(A_ad_, b_ad_error),
                              ".* has size 2, while another entry has size 3");
  // A and b have different number of derivatives.
  auto b_ad_error2 = b_ad_;
  b_ad_error2(0).derivatives() = Eigen::Vector4d::Ones();
  b_ad_error2(1).derivatives() = Eigen::Vector4d::Ones();
  DRAKE_EXPECT_THROWS_MESSAGE(
      LinearSolve<Eigen::LLT>(A_ad_, b_ad_error2),
      ".*A contains derivatives for 3 variables, while b contains derivatives "
      "for 4 variables");
}
}  // namespace
}  // namespace math
}  // namespace drake
