#include "drake/fem/stiffness_matrix.h"

#include <memory>

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace fem {

class StiffnessMatrixTester {
 public:
  // The tester contains a pointer to the stiffness matrix so the stiffness
  // matrix should outlive the tester.
  explicit StiffnessMatrixTester(StiffnessMatrix<double>* stiffness_matrix)
      : stiffness_matrix_(stiffness_matrix) {}

  Eigen::SparseMatrix<double>& get_mutable_matrix() {
    return stiffness_matrix_->matrix_;
  }

 private:
  StiffnessMatrix<double>* stiffness_matrix_;
};

namespace {

class StiffnessMatrixTest : public ::testing::Test {
 protected:
  void SetUp() override {
    stiffness_matrix_ = std::make_unique<StiffnessMatrix<double>>(false);
    tester_ = std::make_unique<StiffnessMatrixTester>(stiffness_matrix_.get());
  }
  std::unique_ptr<StiffnessMatrixTester> tester_;
  std::unique_ptr<StiffnessMatrix<double>> stiffness_matrix_;
};

TEST_F(StiffnessMatrixTest, MatrixFree) {
  EXPECT_EQ(stiffness_matrix_->is_matrix_free(), false);
  stiffness_matrix_->set_matrix_free(true);
  EXPECT_EQ(stiffness_matrix_->is_matrix_free(), true);
}

TEST_F(StiffnessMatrixTest, GetMatrix) {
  stiffness_matrix_->set_matrix_free(true);
  EXPECT_THROW(stiffness_matrix_->get_matrix(), std::runtime_error);
}

TEST_F(StiffnessMatrixTest, Multiply) {
  stiffness_matrix_->set_matrix_free(false);
  auto& matrix = tester_->get_mutable_matrix();
  matrix.resize(2, 2);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.emplace_back(0, 0, 1);
  triplets.emplace_back(0, 1, 2);
  triplets.emplace_back(1, 0, 3);
  triplets.emplace_back(1, 1, 4);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  VectorX<double> x(2);
  x << -1, 2;
  VectorX<double> b = stiffness_matrix_->Multiply(x);
  VectorX<double> analytic_b(2);
  analytic_b << 3, 5;
  EXPECT_TRUE(CompareMatrices(b, analytic_b));
}

TEST_F(StiffnessMatrixTest, LinearSolve) {
  stiffness_matrix_->set_matrix_free(false);
  auto& matrix = tester_->get_mutable_matrix();
  matrix.resize(2, 2);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.emplace_back(0, 0, 2);
  triplets.emplace_back(1, 1, 4);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  VectorX<double> b(2);
  b << 1, -1;
  // Only identity preconditioner is supported.
  Eigen::ConjugateGradient<StiffnessMatrix<double>, Eigen::Lower | Eigen::Upper,
                           Eigen::IdentityPreconditioner>
      cg;
  cg.compute(*stiffness_matrix_);
  VectorX<double> x = cg.solve(b);
  VectorX<double> analytic_solution(2);
  analytic_solution << 0.5, -0.25;
  EXPECT_TRUE(CompareMatrices(x, analytic_solution));
}

}  // namespace
}  // namespace fem
}  // namespace drake
