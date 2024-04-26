#include "drake/solvers/semidefinite_relaxation_internal.h"

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace internal {
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest, TestSparseKron) {
  Eigen::MatrixXd A(3, 3);
  // clang-format off
  A <<  1.77, -4.38, -3.63,
       -2.03, -0.57,  4.56,
       -1.66,  2.15, 10.02;
  // clang-format on
  Eigen::MatrixXd B(2, 2);
  // clang-format off
  B << 7.16, 1.8,
       2.39, 5.77;
  // clang-format on

  Eigen::SparseMatrix<double> C(7, 13);
  std::vector<Eigen::Triplet<double>> C_triplets;
  C_triplets.emplace_back(0, 9, 1.77);
  C_triplets.emplace_back(1, 5, -0.57);
  C_triplets.emplace_back(2, 3, 4.56);
  C_triplets.emplace_back(3, 12, -1.66);
  C_triplets.emplace_back(4, 7, 2.15);
  C_triplets.emplace_back(5, 6, 10.02);
  C_triplets.emplace_back(0, 1, -0.45);
  C.setFromTriplets(C_triplets.begin(), C_triplets.end());

  auto TestKron = [](const Eigen::SparseMatrix<double>& A_test,
                     const Eigen::SparseMatrix<double>& B_test) {
    // AXB = (B.T ⊗ A)vec(X) so we compute this as a numerical sanity check.
    const Eigen::SparseMatrix<double> M1 = B_test.transpose();
    const Eigen::SparseMatrix<double> M2 = A_test;

    Eigen::SparseMatrix<double> kron = SparseKroneckerProduct(M1, M2);
    EXPECT_EQ(kron.rows(), M1.rows() * M2.rows());
    EXPECT_EQ(kron.cols(), M1.cols() * M2.cols());
    EXPECT_EQ(kron.nonZeros(), M1.nonZeros() * M2.nonZeros());

    Eigen::MatrixXd testMatrix(A_test.cols(), B_test.rows());
    for (int i = 0; i < testMatrix.rows(); ++i) {
      for (int j = 0; j < testMatrix.cols(); ++j) {
        // put arbitrary values in testMatrix
        testMatrix(i, j) = 2 * (i + j) / (i + j + 1);
      }
    }
    Eigen::MatrixXd AXB = A_test * testMatrix * B_test;
    Eigen::VectorXd kron_vec = kron * Eigen::Map<const Eigen::VectorXd>(
                                          testMatrix.data(), testMatrix.size());
    EXPECT_TRUE(CompareMatrices(
        Eigen::Map<const Eigen::VectorXd>(AXB.data(), AXB.size()), kron_vec,
        1e-10, MatrixCompareType::absolute));
  };

  TestKron(A.sparseView(), B.sparseView());
  TestKron(B.sparseView(), A.sparseView());
  TestKron(C, A.sparseView());
  TestKron(B.sparseView(), C);
  TestKron(C, B.sparseView());
}

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest,
           TestToSymmetricMatrixFromTensorVector) {
  // Check the 2x2 ⊗  3x3 case.
  int num_elts =
      3 * 6;  // There are 3 elements in 2x2 basis and 6 in the 3x3 basis.
  VectorX<symbolic::Variable> y(num_elts);
  for (int i = 0; i < num_elts; ++i) {
    y(i) = symbolic::Variable("y_" + std::to_string(i));
  }
  Eigen::MatrixX<symbolic::Variable> Y =
      ToSymmetricMatrixFromTensorVector(y, 2, 3);
  Eigen::MatrixX<symbolic::Variable> Y_expected(6, 6);
  // clang-format off
  Y_expected << y(0), y(1),  y(2),  y(6),  y(7),  y(8),
                y(1), y(3),  y(4),  y(7),  y(9),  y(10),
                y(2), y(4),  y(5),  y(8),  y(10), y(11),
                y(6), y(7),  y(8),  y(12), y(13), y(14),
                y(7), y(9),  y(10), y(13), y(15), y(16),
                y(8), y(10), y(11), y(14), y(16), y(17);
  // clang-format on
  EXPECT_EQ(Y.rows(), 6);
  EXPECT_EQ(Y.cols(), 6);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_TRUE(Y_expected(i, j).equal_to(Y(i, j)));
    }
  }

  // Check the 4x4 ⊗  3x3 case.
  num_elts = 10 * 6;
  VectorX<symbolic::Variable> x(num_elts);
  for (int i = 0; i < num_elts; ++i) {
    x(i) = symbolic::Variable(fmt::format("x({}),", std::to_string(i)));
  }
  Eigen::MatrixX<symbolic::Variable> X =
      ToSymmetricMatrixFromTensorVector(x, 4, 3);
  Eigen::MatrixX<symbolic::Variable> X_expected(12, 12);
  // The following was checked by hand. It may seem big, but it is necessary to
  // check something this large.
  // clang-format off
  // NOLINT
  X_expected <<  x(0),  x(1),  x(2),  x(6),  x(7),  x(8), x(12), x(13), x(14), x(18), x(19), x(20),// NOLINT
                 x(1),  x(3),  x(4),  x(7),  x(9), x(10), x(13), x(15), x(16), x(19), x(21), x(22),// NOLINT
                 x(2),  x(4),  x(5),  x(8), x(10), x(11), x(14), x(16), x(17), x(20), x(22), x(23),// NOLINT
                 x(6),  x(7),  x(8), x(24), x(25), x(26), x(30), x(31), x(32), x(36), x(37), x(38),// NOLINT
                 x(7),  x(9), x(10), x(25), x(27), x(28), x(31), x(33), x(34), x(37), x(39), x(40),// NOLINT
                 x(8), x(10), x(11), x(26), x(28), x(29), x(32), x(34), x(35), x(38), x(40), x(41),// NOLINT
                x(12), x(13), x(14), x(30), x(31), x(32), x(42), x(43), x(44), x(48), x(49), x(50),// NOLINT
                x(13), x(15), x(16), x(31), x(33), x(34), x(43), x(45), x(46), x(49), x(51), x(52),// NOLINT
                x(14), x(16), x(17), x(32), x(34), x(35), x(44), x(46), x(47), x(50), x(52), x(53),// NOLINT
                x(18), x(19), x(20), x(36), x(37), x(38), x(48), x(49), x(50), x(54), x(55), x(56),// NOLINT
                x(19), x(21), x(22), x(37), x(39), x(40), x(49), x(51), x(52), x(55), x(57), x(58),// NOLINT
                x(20), x(22), x(23), x(38), x(40), x(41), x(50), x(52), x(53), x(56), x(58), x(59);// NOLINT
  // clang-format on
  EXPECT_EQ(X.rows(), 12);
  EXPECT_EQ(X.cols(), 12);
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      EXPECT_TRUE(X_expected(i, j).equal_to(X(i, j)));
    }
  }
}

GTEST_TEST(MakeSemidefiniteRelaxationInternalTest, TestWAdj) {
  Eigen::MatrixXd Y(5, 5);
  // clang-format off
  Y << -3.08, -0.84,  0.32,  0.54,  0.51,
       -0.84,  2.6 ,  1.72,  0.09,  0.79,
        0.32,  1.72,  3.09,  1.19,  0.31,
        0.54,  0.09,  1.19, -0.37, -2.57,
        0.51,  0.79,  0.31, -2.57, -0.08;
  // clang-format on
  // The lower triangular part of Y has 15 entries
  Eigen::VectorXd y_tril(15);
  y_tril = math::ToLowerTriangularColumnsFromMatrix(Y);

  Eigen::SparseMatrix<double> W_adj = GetWAdjForTril(6);
  EXPECT_EQ(W_adj.rows(), 6);
  EXPECT_EQ(W_adj.cols(), y_tril.size());
  EXPECT_EQ(W_adj.nonZeros(), 14 /* 2 * (6-1) + 4 */);

  Eigen::VectorXd result(6);
  result = W_adj * y_tril;
  const double tol{1e-10};
  EXPECT_NEAR(result(0), Y.trace(), tol);
  EXPECT_NEAR(result(1), Y(0, 0) - Y.block(1, 1, 4, 4).trace(), tol);
  for (int i = 2; i < W_adj.rows(); ++i) {
    EXPECT_NEAR(result(i), 2 * Y(0, i - 1), tol);
  }
}

namespace {
// Makes a random vector with all positive entries that is not in the Lorentz
// cone. This method does not sample uniformly over all such vectors.
Eigen::VectorXd MakeRandomPositiveOrthantNonLorentzVector(const int r,
                                                          const double scale) {
  Eigen::VectorXd ret =
      scale * (Eigen::VectorXd::Random(r) + Eigen::VectorXd::Ones(r));
  while (r > 1 && ret(0) > ret.tail(r - 1).norm()) {
    // This ensures we are not in the Lorentz cone by scaling the first entry to
    // be too small.
    ret(0) = 1 / (0.5 * ret.tail(r - 1).norm()) *
             (Eigen::VectorXd::Random(1)(0) + 1);
  }
  return ret;
}

// Returns true if the vector x has all entries which are positive
bool CheckIsPositiveOrthantVector(const Eigen::Ref<const Eigen::VectorXd>& x) {
  return (x.array() >= 0).all();
}

// Construct a random matrix A that is guaranteed to map a positive vector in m
// dimensions to another positive vector in r dimensions. This method does not
// sample uniformly over all such maps.
Eigen::MatrixXd MakeRandomPositiveOrthantPositiveMap(const int r, const int m,
                                                     const double scale) {
  Eigen::MatrixXd A(r, m);
  for (int i = 0; i < m; i++) {
    A.col(i) = MakeRandomPositiveOrthantNonLorentzVector(r, scale);
  }
  return A;
}

// Constructs a random vector that is in the Lorentz cone, but is not in the
// positive orthant. This method does not sample uniformly over all such
// vectors.
Eigen::VectorXd MakeRandomLorentzNonPositiveOrthantVector(const int r,
                                                          const double scale) {
  Eigen::VectorXd ret(r);
  if (r > 1) {
    ret.tail(r - 1) = scale * Eigen::VectorXd::Random(r - 1);
    // Ensures that ret(0) ≥ norm(ret(r-1)) and that ret(0) ≥ 0.
    do {
      ret(0) = 2 * scale * (Eigen::VectorXd::Random(1)(0) + 1);
    } while (ret(0) < ret.tail(r - 1).norm() || ret(0) < 0);
    // Ensure that ret has at least 1 negative entry so that it is not in the
    // positive orthant.
    if (ret(1) > 0) {
      ret(1) *= -1;
    }

  } else {
    ret(0) = scale * (Eigen::VectorXd::Random(1)(0) + 1);
  }
  return ret;
}

// Returns true if the vector x is in the Lorentz cone
bool CheckIsLorentzVector(const Eigen::Ref<const Eigen::VectorXd>& x) {
  if (x.rows() == 1) {
    return x(0) >= 0;
  }
  return x(0) >= x.tail(x.rows() - 1).norm();
}

// Construct a random matrix A that is guaranteed to map a Lorentz vector in m
// dimensions to another Lorentz vector in r. This method does not sample
// uniformly over all such maps.
Eigen::MatrixXd MakeRandomLorentzPositiveMap(const int r, const int m,
                                             const double scale) {
  // Create a map A: (t; x) ↦ (st; A₁x) where s is some positive scaling of t
  // and A₁ has induced matrix norm less than s. This ensures that A is a
  // Lorentz positive map, the norm |A₁x| ≤ s and therefore |A₁x| ≤ st
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(r, m);
  // Choose a random, positive scaling larger than 1/2 for the first entry.
  A(0, 0) = scale * (Eigen::VectorXd::Random(1)(0) + 1.5);

  if (r > 1 && m > 1) {
    Eigen::MatrixXd T = Eigen::MatrixXd::Random(r - 1, m - 1);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        T, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Rescale the singular values so that the matrix norm is less than s/2.
    const double svd_scaling = A(0, 0) / svd.singularValues().maxCoeff() / 2;
    const Eigen::VectorXd regularized_singular_values =
        svd_scaling * svd.singularValues();

    A.bottomRightCorner(r - 1, m - 1) =
        svd.matrixU() * regularized_singular_values.asDiagonal() *
        svd.matrixV().transpose();
  } else if (m == 1) {
    // Make the last r entries of the column matrix have norm A(0,0) / 2
    A.col(0).tail(r - 1) = Eigen::VectorXd::Random(r - 1);
    A.col(0).tail(r - 1) =
        A.col(0).tail(r - 1) / A.col(0).tail(r - 1).norm() * (A(0, 0) / 2);
  } else if (r == 1) {
    // Make the last m entries of row matrix have norm A(0,0) / 2
    A.row(0).tail(m - 1) = Eigen::VectorXd::Random(m - 1);
    A.row(0).tail(m - 1) =
        A.row(0).tail(m - 1) / A.row(0).tail(m - 1).norm() * (A(0, 0) / 2);
  }
  return A;
}

bool ProgramSolvedWithoutError(const SolutionResult& result) {
  return result == kSolutionFound || result == kInfeasibleConstraints ||
         result == kUnbounded || result == kInfeasibleOrUnbounded ||
         result == kDualInfeasible;
}

enum SupportedTestCone { kPositiveOrthant, kLorentzCone };

}  // namespace

class Cone1ByCone2SeparabilityTest
    : public ::testing::TestWithParam<std::tuple<int, int>> {
 public:
  Cone1ByCone2SeparabilityTest(const SupportedTestCone cone1,
                               const SupportedTestCone cone2)
      : prog_(), cone1_(cone1), cone2_(cone2) {
    // Seed the random generator used by Eigen.
    std::srand(99);
    int m, n;
    std::tie(m, n) = GetParam();
    X_ = prog_.NewContinuousVariables(m, n, "X");
  }

  void DoTest(const std::optional<Eigen::MatrixXd>& A_opt = std::nullopt,
              const std::optional<Eigen::MatrixXd>& B_opt = std::nullopt) {
    MatrixX<Expression> Y;
    Eigen::MatrixXd A =
        A_opt.value_or(Eigen::MatrixXd::Identity(X_.rows(), X_.rows()));
    Eigen::MatrixXd B =
        B_opt.value_or(Eigen::MatrixXd::Identity(X_.cols(), X_.cols()));
    if (!A_opt.has_value() && !B_opt.has_value()) {
      // Use the Variable variant of the function.
      Y = X_.template cast<Expression>();
      AddCone1ByCone2SeparableConstraint(X_);
    } else {
      // Use the Expression variant of the function.
      Y = A * X_ * B;
      AddCone1ByCone2SeparableConstraint(Y);
    }
    CheckCone1ByCone2SeparableConstraintsAdded(A, B);

    MakeTestVectors(A, B);

    // Y is required to be equal to a simple cone1-cone2 separable
    // tensor therefore this program should be feasible.
    Eigen::MatrixXd test_matrix = xr_cone1_ * xc_cone2_.transpose();
    EXPECT_EQ(DoTestCase(Y, test_matrix), true);

    // Y is required to be equal to a cone1-cone2 separable tensor
    // therefore this program should be feasible.
    test_matrix = (2 * xr_cone1_ * yc_cone2_.transpose() +
                   3 * yr_cone1_ * xc_cone2_.transpose());
    EXPECT_EQ(DoTestCase(Y, test_matrix), true);

    // Y is required to be equal to something not that is not
    // cone1-cone2 separable, therefore this should be infeasible.
    test_matrix = -100 * zr_not_conic_ * zc_not_conic_.transpose();
    EXPECT_EQ(DoTestCase(Y, test_matrix), false);

    // A tensor between a cone1 vector, and non-conic vector.
    // Y is required to be equal to something not that is not cone1-cone2
    // separable therefore this should be infeasible.
    test_matrix = yr_cone1_ * zc_not_conic_.transpose();
    EXPECT_EQ(DoTestCase(Y, test_matrix), false);

    // A tensor between a non-conic vector and cone2 vector.
    // Y is required to be equal to something not that is not Lorentz
    // separable therefore this should be infeasible.
    test_matrix = zr_not_conic_ * xc_cone2_.transpose();
    EXPECT_EQ(DoTestCase(Y, test_matrix), false);

    // A tensor that is cone1-cone1 separable. This is cone1-cone2 separable if
    // and only if cone1==cone2. In the case that B is not full row rank, then Y
    // will generically fail to be separable due to the subspace constraint
    // imposed by B (i.e. wc_cone1_ is not in the image of B).
    test_matrix = xr_cone1_ * wc_cone1_.transpose();
    EXPECT_EQ(DoTestCase(Y, test_matrix),
              cone1_ == cone2_ && B.cols() <= B.rows());

    // A tensor that is cone2-cone2 separable. This is cone1-cone2 separable if
    // and only if cone1==cone2. In the case that A is not full column rank,
    // then Y will generically fail to be separable due to the subspace
    // constraint imposed by A (i.e. wr_cone2_ is not in the image of A).
    test_matrix = wr_cone2_ * xc_cone2_.transpose();
    EXPECT_EQ(DoTestCase(Y, test_matrix),
              cone1_ == cone2_ && A.rows() <= A.cols());

    // A tensor that is non-conic combination of cone1-cone2 separable tensors
    // separable tensors.
    test_matrix = 2 * yr_cone1_ * xc_cone2_.transpose() -
                  1000 * xr_cone1_ * yc_cone2_.transpose();
    EXPECT_EQ(DoTestCase(Y, test_matrix), false);

    // Now clear out the constraints, so that we can repeatedly call this
    // method.
    for (const auto& constraints : prog_.GetAllConstraints()) {
      prog_.RemoveConstraint(constraints);
    }
    ASSERT_EQ(prog_.GetAllConstraints().size(), 0);
  }

 protected:
  // Adds the constraint that Z is Cone 1 by Cone 2 separable to prog_ and
  // ensures that the right number of constraints and variables have been added
  // to prog_.
  virtual void AddCone1ByCone2SeparableConstraint(
      const MatrixX<Variable>& Z) = 0;

  // Adds the constraint that Z is Cone 1 by Cone 2 separable to prog_ and
  // ensures that the right number of constraints and variables have been added
  // to prog_.
  virtual void AddCone1ByCone2SeparableConstraint(
      const MatrixX<Expression>& Z) = 0;

  // Checks that prog_ has the right number of constraints and variables to
  // indicate that a matrix is constrained to be Cone1 by Cone2 separable.
  virtual void CheckCone1ByCone2SeparableConstraintsAdded(
      const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) = 0;

  MathematicalProgram prog_;
  // The matrix we will try to write as a tensor product of elements in cone1
  // and cone2.
  MatrixX<Variable> X_;

  // A vector guaranteed to be in cone one and in the range of A.
  Eigen::VectorXd xr_cone1_;
  // A vector guaranteed to be in cone one and in the range of A.
  Eigen::VectorXd yr_cone1_;
  // A vector that is in cone one but NOT guaranteed to
  // be in the range A.
  Eigen::VectorXd wr_cone1_;
  // A vector that is in cone one but NOT guaranteed to be in
  // the range of Bᵀ.
  Eigen::VectorXd wc_cone1_;

  // A vector guaranteed to be in cone two and in the range of Bᵀ.
  Eigen::VectorXd xc_cone2_;
  // A vector guaranteed to be in cone two and in the range of Bᵀ.
  Eigen::VectorXd yc_cone2_;
  // A vector that is in cone two but NOT guaranteed to
  // be in the range A.
  Eigen::VectorXd wr_cone2_;
  // A vector that is in cone two but NOT guaranteed to
  // be in the range of Bᵀ.
  Eigen::VectorXd wc_cone2_;

  // Two vectors in neither cone.
  Eigen::VectorXd zr_not_conic_;
  Eigen::VectorXd zc_not_conic_;

 private:
  // Initialize the protected test vectors to be in the requisite cones and in
  // the image of A and Bᵀ respectively as needed.
  void MakeTestVectors(const Eigen::Ref<const Eigen::MatrixXd>& A,
                       const Eigen::Ref<const Eigen::MatrixXd>& B) {
    const double scale = 2.0;
    const auto initialize_vectors_in_cone_and_image =
        [&scale, &A, &B](const Eigen::MatrixXd& C, const SupportedTestCone cone,
                         Eigen::VectorXd* x, Eigen::VectorXd* y,
                         Eigen::VectorXd* wr, Eigen::VectorXd* wc) {
          // Notice that if C is not a cone-positive map, then x and
          // y will not necessarily be in the cone.
          switch (cone) {
            case SupportedTestCone::kPositiveOrthant:
              *x = C *
                   MakeRandomPositiveOrthantNonLorentzVector(C.cols(), scale);
              *y = C *
                   MakeRandomPositiveOrthantNonLorentzVector(C.cols(), scale);
              if (!CheckIsPositiveOrthantVector(*x) ||
                  !CheckIsPositiveOrthantVector(*y)) {
                throw std::logic_error(fmt::format(
                    "C=\n{}\n is not a positive-orthant positive map.",
                    fmt_eigen(C)));
              }
              *wr = MakeRandomPositiveOrthantNonLorentzVector(A.rows(), scale);
              *wc = MakeRandomPositiveOrthantNonLorentzVector(B.cols(), scale);
              break;
            case SupportedTestCone::kLorentzCone:
              *x = C *
                   MakeRandomLorentzNonPositiveOrthantVector(C.cols(), scale);
              *y = C *
                   MakeRandomLorentzNonPositiveOrthantVector(C.cols(), scale);
              if (!CheckIsLorentzVector(*x) || !CheckIsLorentzVector(*y)) {
                throw std::logic_error(fmt::format(
                    "C=\n{}\n is not a Lorentz positive map.", fmt_eigen(C)));
              }
              *wr = MakeRandomLorentzNonPositiveOrthantVector(A.rows(), scale);
              *wc = MakeRandomLorentzNonPositiveOrthantVector(B.cols(), scale);
              break;
          }
        };
    initialize_vectors_in_cone_and_image(A, cone1_, &xr_cone1_, &yr_cone1_,
                                         &wr_cone1_, &wc_cone1_);

    initialize_vectors_in_cone_and_image(B.transpose(), cone2_, &xc_cone2_,
                                         &yc_cone2_, &wr_cone2_, &wc_cone2_);

    const auto is_conic_vector = [](const Eigen::VectorXd& z,
                                    const SupportedTestCone cone) {
      switch (cone) {
        case SupportedTestCone::kPositiveOrthant:
          return CheckIsPositiveOrthantVector(z);
        case SupportedTestCone::kLorentzCone:
          return CheckIsLorentzVector(z);
      }
      DRAKE_UNREACHABLE();
    };
    zr_not_conic_ = Eigen::VectorXd::Random(A.rows());
    // Note that all supported cones occupy less (and typically much less) than
    // 1/2 the volume of the ambient space and therefore this loop is unlikely
    // to take very long.
    while (is_conic_vector(zr_not_conic_, cone1_) ||
           is_conic_vector(zr_not_conic_, cone2_)) {
      zr_not_conic_ = Eigen::VectorXd::Random(A.rows());
    }
    zc_not_conic_ = Eigen::VectorXd::Random(B.cols());
    // Note that all supported cones occupy less (and typically much less) than
    // 1/2 the volume of the ambient space and therefore this loop is unlikely
    // to take very long.
    while (is_conic_vector(zc_not_conic_, cone1_) ||
           is_conic_vector(zc_not_conic_, cone2_)) {
      zc_not_conic_ = Eigen::VectorXd::Random(B.cols());
    }
  }

  bool DoTestCase(const Eigen::Ref<const MatrixX<Expression>>& Y,
                  const Eigen::Ref<const Eigen::MatrixXd>& test_mat) {
    auto constraint = prog_.AddLinearEqualityConstraint(Y == test_mat);
    auto result = Solve(prog_);
    prog_.RemoveConstraint(constraint);
    DRAKE_DEMAND(ProgramSolvedWithoutError(result.get_solution_result()));

    return result.is_success();
  }

  const SupportedTestCone cone1_;

  const SupportedTestCone cone2_;
};

class PositiveOrthantByLorentzSeparabilityTest
    : public Cone1ByCone2SeparabilityTest {
 public:
  PositiveOrthantByLorentzSeparabilityTest()
      : Cone1ByCone2SeparabilityTest(SupportedTestCone::kPositiveOrthant,
                                     SupportedTestCone::kLorentzCone) {}

 private:
  void AddCone1ByCone2SeparableConstraint(const MatrixX<Variable>& Z) {
    AddMatrixIsPositiveOrthantByLorentzSeparableConstraint(Z, &prog_);
  }
  void AddCone1ByCone2SeparableConstraint(const MatrixX<Expression>& Z) {
    AddMatrixIsPositiveOrthantByLorentzSeparableConstraint(Z, &prog_);
  }
  void CheckCone1ByCone2SeparableConstraintsAdded(const Eigen::MatrixXd& A,
                                                  const Eigen::MatrixXd& B) {
    EXPECT_EQ(ssize(prog_.lorentz_cone_constraints()), A.rows());
    EXPECT_EQ(ssize(prog_.GetAllConstraints()), A.rows());
    unused(B);
  }
};

TEST_P(PositiveOrthantByLorentzSeparabilityTest,
       AddMatrixIsPositiveOrthantByLorentzSeparableConstraintVariable) {
  this->DoTest();
}

TEST_P(
    PositiveOrthantByLorentzSeparabilityTest,
    AddMatrixIsPositiveOrthantByLorentzSeparableConstraintExpressionKeepSize) {
  int m, n;
  std::tie(m, n) = GetParam();

  const double scale = 3;
  // This map needs to be a positive, positive-orthant map.
  Eigen::MatrixXd A = MakeRandomPositiveOrthantPositiveMap(m, m, scale);
  // This map needs to be a positive Lorentz map.
  Eigen::MatrixXd B = MakeRandomLorentzPositiveMap(n, n, scale);
  this->DoTest(A, B);
}

TEST_P(PositiveOrthantByLorentzSeparabilityTest,
       AddMatrixIsPositiveOrthantByLorentzSeparableConstraintChangeSize) {
  // The subspaces in some of these tests have small dimension, making them
  // numerically difficult for SDP solvers which do no preprocessing. Therefore,
  // we only solve these with Mosek as it is the only one which currently gives
  // reliable results.
  if (MosekSolver().enabled() && MosekSolver().available()) {
    int m, n;
    std::tie(m, n) = GetParam();
    const int r1 = m + 5;
    const int r2 = m - 1;
    const int c1 = n + 3;
    const int c2 = n - 2;

    const double scale = 0.75;
    // These maps need to be positive, positive-orthant maps.
    Eigen::MatrixXd A1 = MakeRandomPositiveOrthantPositiveMap(r1, m, scale);
    Eigen::MatrixXd A2 = MakeRandomPositiveOrthantPositiveMap(r2, m, scale);
    // These maps need to be positive Lorentz maps.
    Eigen::MatrixXd B1 = MakeRandomLorentzPositiveMap(n, c1, scale);
    Eigen::MatrixXd B2 = MakeRandomLorentzPositiveMap(n, c2, scale);

    this->DoTest(A1, B1);
    this->DoTest(A1, B2);
    this->DoTest(A2, B1);
    this->DoTest(A2, B2);
  }
}

INSTANTIATE_TEST_SUITE_P(test, PositiveOrthantByLorentzSeparabilityTest,
                         ::testing::Values(std::pair<int, int>{4, 5},  // m < n
                                           std::pair<int, int>{5, 4},  // m > n
                                           std::pair<int, int>{6, 6}   // m == n
                                           // There are no special cases
                                           ));  // NOLINT

class LorentzByPositiveOrthantSeparabilityTest
    : public Cone1ByCone2SeparabilityTest {
 public:
  LorentzByPositiveOrthantSeparabilityTest()
      : Cone1ByCone2SeparabilityTest(SupportedTestCone::kLorentzCone,
                                     SupportedTestCone::kPositiveOrthant) {}

 private:
  void AddCone1ByCone2SeparableConstraint(const MatrixX<Variable>& Z) {
    AddMatrixIsLorentzByPositiveOrthantSeparableConstraint(Z, &prog_);
  }
  void AddCone1ByCone2SeparableConstraint(const MatrixX<Expression>& Z) {
    AddMatrixIsLorentzByPositiveOrthantSeparableConstraint(Z, &prog_);
  }
  void CheckCone1ByCone2SeparableConstraintsAdded(const Eigen::MatrixXd& A,
                                                  const Eigen::MatrixXd& B) {
    EXPECT_EQ(ssize(prog_.lorentz_cone_constraints()), B.cols());
    EXPECT_EQ(ssize(prog_.GetAllConstraints()), B.cols());
    unused(A);
  }
};

TEST_P(LorentzByPositiveOrthantSeparabilityTest,
       AddMatrixIsLorentzByPositiveOrthantSeparableConstraintVariable) {
  this->DoTest();
}

TEST_P(
    LorentzByPositiveOrthantSeparabilityTest,
    AddMatrixIsLorentzByPositiveOrthantSeparableConstraintExpressionKeepSize) {
  int m, n;
  std::tie(m, n) = GetParam();

  const double scale = 3;
  // This map needs to be a positive, positive-orthant map.
  Eigen::MatrixXd A = MakeRandomLorentzPositiveMap(m, m, scale);
  // This map needs to be a positive Lorentz map.
  Eigen::MatrixXd B = MakeRandomPositiveOrthantPositiveMap(n, n, scale);
  this->DoTest(A, B);
}

TEST_P(LorentzByPositiveOrthantSeparabilityTest,
       AddMatrixIsLorentzByPositiveOrthantSeparableConstraintChangeSize) {
  // The subspaces in some of these tests have small dimension, making them
  // numerically difficult for SDP solvers which do no preprocessing. Therefore,
  // we only solve these with Mosek as it is the only one which currently gives
  // reliable results.
  if (MosekSolver().enabled() && MosekSolver().available()) {
    int m, n;
    std::tie(m, n) = GetParam();
    const int r1 = m + 5;
    const int r2 = m - 1;
    const int c1 = n + 3;
    const int c2 = n - 2;

    const double scale = 3;
    // These maps need to be positive, positive-orthant maps.
    Eigen::MatrixXd A1 = MakeRandomLorentzPositiveMap(r1, m, scale);
    Eigen::MatrixXd A2 = MakeRandomLorentzPositiveMap(r2, m, scale);
    // These maps need to be positive Lorentz maps.
    Eigen::MatrixXd B1 = MakeRandomPositiveOrthantPositiveMap(n, c1, scale);
    Eigen::MatrixXd B2 = MakeRandomPositiveOrthantPositiveMap(n, c2, scale);

    this->DoTest(A1, B1);
    this->DoTest(A1, B2);
    this->DoTest(A2, B1);
    this->DoTest(A2, B2);
  }
}

INSTANTIATE_TEST_SUITE_P(test, LorentzByPositiveOrthantSeparabilityTest,
                         ::testing::Values(std::pair<int, int>{4, 5},  // m < n
                                           std::pair<int, int>{5, 4},  // m > n
                                           std::pair<int, int>{6, 6}   // m == n
                                           // There are no special cases
                                           ));  // NOLINT

class LorentzByLorentzSeparabilityTest : public Cone1ByCone2SeparabilityTest {
 public:
  LorentzByLorentzSeparabilityTest()
      : Cone1ByCone2SeparabilityTest(SupportedTestCone::kLorentzCone,
                                     SupportedTestCone::kLorentzCone) {}

 private:
  void AddCone1ByCone2SeparableConstraint(const MatrixX<Variable>& Z) {
    AddMatrixIsLorentzByLorentzSeparableConstraint(Z, &prog_);
  }
  void AddCone1ByCone2SeparableConstraint(const MatrixX<Expression>& Z) {
    AddMatrixIsLorentzByLorentzSeparableConstraint(Z, &prog_);
  }
  void CheckCone1ByCone2SeparableConstraintsAdded(const Eigen::MatrixXd& A,
                                                  const Eigen::MatrixXd& B) {
    int r = A.rows();
    int c = B.cols();
    if (r >= 3 && c >= 3) {
      EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 1);
      EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);
      EXPECT_EQ(ssize(prog_.GetAllConstraints()), 2);

      const Binding<PositiveSemidefiniteConstraint> psd_constraint =
          prog_.positive_semidefinite_constraints()[0];
      EXPECT_EQ(psd_constraint.evaluator()->matrix_rows(), (r - 1) * (c - 1));
    } else if (r == 1 && c == 1) {
      // This linear constraints gets parsed as either a bounding box or linear
      // constraint, but not both.
      EXPECT_EQ(ssize(prog_.bounding_box_constraints()) +
                    ssize(prog_.linear_constraints()),
                1);
      EXPECT_EQ(ssize(prog_.GetAllConstraints()), 1);
    } else if (r <= 2 && c <= 2) {
      EXPECT_EQ(ssize(prog_.linear_constraints()), 1);
      EXPECT_EQ(ssize(prog_.GetAllConstraints()), 1);
    } else if (r > 2) {
      EXPECT_EQ(ssize(prog_.lorentz_cone_constraints()), c);
      EXPECT_EQ(ssize(prog_.GetAllConstraints()), c);
    } else {
      EXPECT_EQ(ssize(prog_.lorentz_cone_constraints()), r);
      EXPECT_EQ(ssize(prog_.GetAllConstraints()), r);
    }
  }
};

TEST_P(LorentzByLorentzSeparabilityTest,
       AddMatrixIsLorentzByLorentzSeparableConstraintVariable) {
  int m, n;
  std::tie(m, n) = GetParam();
  this->DoTest();
}

TEST_P(LorentzByLorentzSeparabilityTest,
       AddMatrixIsLorentzByLorentzSeparableConstraintExpressionKeepSize) {
  int m, n;
  std::tie(m, n) = GetParam();
  double scale = 3;
  // These maps need to be Lorentz-positive maps.
  Eigen::MatrixXd A = MakeRandomLorentzPositiveMap(m, m, scale);
  Eigen::MatrixXd B = MakeRandomLorentzPositiveMap(n, n, scale);
  this->DoTest(A, B);
}

TEST_P(LorentzByLorentzSeparabilityTest,
       AddMatrixIsLorentzByLorentzSeparableConstraintExpressionChangeSize) {
  // The subspaces in some of these tests have small dimension, making them
  // numerically difficult for SDP solvers which do no preprocessing. Therefore,
  // we only solve these with Mosek as it is the only one which currently gives
  // reliable results.
  if (MosekSolver().enabled() && MosekSolver().available()) {
    int m, n;
    std::tie(m, n) = GetParam();
    const int r1 = m + 5;
    const int r2 = std::max(m - 1, 1);
    const int c1 = n + 3;
    const int c2 = std::max(n - 2, 1);

    const double scale = 2;
    // These maps need to be positive Lorentz maps.
    Eigen::MatrixXd A1 = MakeRandomLorentzPositiveMap(r1, m, scale);
    Eigen::MatrixXd A2 = MakeRandomLorentzPositiveMap(r2, m, scale);
    Eigen::MatrixXd B1 = MakeRandomLorentzPositiveMap(n, c1, scale);
    Eigen::MatrixXd B2 = MakeRandomLorentzPositiveMap(n, c2, scale);
    this->DoTest(A1, B1);
    this->DoTest(A2, B1);
    this->DoTest(A1, B2);
    this->DoTest(A2, B2);
  }
}

INSTANTIATE_TEST_SUITE_P(
    test, LorentzByLorentzSeparabilityTest,
    ::testing::Values(std::pair<int, int>{3, 4},  // m < n
                      std::pair<int, int>{4, 3},  // m > n
                      std::pair<int, int>{5, 5},  // m == n
                      std::pair<int, int>{1, 1},  // special case m = n = 1
                      std::pair<int, int>{2, 1},  // special case m = 2, n = 1
                      std::pair<int, int>{1, 2},  // special case m = 1, m = 1
                      std::pair<int, int>{2, 2},  // special case m = 2, m = 2
                      std::pair<int, int>{1, 4},  // special case m = 1, n ≥ 3
                      std::pair<int, int>{2, 5},  // special case m = 2, n ≥ 3
                      std::pair<int, int>{3, 1},  // special case m ≥ 3, n = 1
                      std::pair<int, int>{7, 2}   // special case m ≥ 3, n = 2
                      ));                         // NOLINT

}  // namespace internal
}  // namespace solvers
}  // namespace drake
