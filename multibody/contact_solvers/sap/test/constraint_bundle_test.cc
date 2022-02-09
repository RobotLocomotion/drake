#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Makes an arbitrary non-zero Jacobian matrix where each entry is the linear
// index starting at element (0, 0). Examples:
// MakeJ(3, 2) returns:
//  |1 4|
//  |2 5|
//  |3 6|
// MakeJ(1, 3) returns:
//  |1 2 3|
MatrixXd MakeJ(int rows, int cols) {
  const int size = rows * cols;
  MatrixXd J1d = VectorXd::LinSpaced(size, 1., 1. * size);
  J1d.resize(rows, cols);
  return J1d;
}

// Make zero matrix of size rows by cols.
MatrixXd MakeZ(int rows, int cols) {
  return MatrixXd::Zero(rows, cols);
}

// Constraint for unit testing where we assume the number of dofs for `clique`
// is clique + 1. We use this constraint to test that the constraint bundel
// properly composes projections.
class TestConstraint final : public SapConstraint<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestConstraint);

  TestConstraint(int clique, int size, double param)
      : SapConstraint<double>(clique, MakeJ(size, clique + 1)), param_(param) {}

  // @throws if the number of rows in J0 and J1 is different from three.
  TestConstraint(int clique0, int clique1, int size, double param)
      : SapConstraint<double>(clique0, clique1, MakeJ(size, clique0 + 1),
                              MakeJ(size, clique1 + 1)),
        param_(param) {}

  // Implements a fake projection operation where the result is given by:
  //   gamma = param * R * y, and
  //   dPdy = param * R
  // This is to verify input and output arguments are properly sliced by
  // ConstraintBundle.
  void Project(const Eigen::Ref<const VectorX<double>>& y,
               const Eigen::Ref<const VectorX<double>>& R,
               EigenPtr<VectorX<double>> gamma,
               MatrixX<double>* dPdy) const final {
    *gamma = param_ * (R.asDiagonal() * y);
    if (dPdy != nullptr) {
      *dPdy = param_ * R.asDiagonal();
    }
  };

  // Not used in this test.
  VectorX<double> CalcBiasTerm(const double&, const double&) const final {
    DRAKE_UNREACHABLE();
  }

  // Not used in this test.
  VectorX<double> CalcDiagonalRegularization(const double& time_step,
                                             const double& wi) const final {
    DRAKE_UNREACHABLE();
  }

 private:
  double param_{0.0};
};

namespace {

GTEST_TEST(SapConstraintBundle, Construction) {
  // clang-format off
  const MatrixXd J = (MatrixXd(11, 6) <<
    MakeJ(3,1), MakeZ(3, 2), MakeJ(3, 3),
    MakeJ(8,1), MakeJ(8, 2), MakeZ(8, 3)).finished();
  // clang-format on

  std::vector<std::unique_ptr<SapConstraint<double>>> owned_constraints;
  owned_constraints.push_back(
      std::make_unique<TestConstraint>(0, 2, 1, 1.0));
  owned_constraints.push_back(
      std::make_unique<TestConstraint>(0, 2, 2, 2.0));
  owned_constraints.push_back(
      std::make_unique<TestConstraint>(0, 1, 3, 3.0));            
  owned_constraints.push_back(
      std::make_unique<TestConstraint>(0, 1, 2, 4.0));
  owned_constraints.push_back(
      std::make_unique<TestConstraint>(0, 1, 3, 5.0));

  std::vector<const SapConstraint<double>*> constraints;
  for (auto& c : owned_constraints) {
    constraints.push_back(c.get());
  }

  BlockSparseMatrixBuilder<double> builder(2, 3, 6);
  builder.PushBlock(0, 0, J.block(0, 0, 3, 1));
  builder.PushBlock(0, 2, J.block(0, 3, 3, 3));
  builder.PushBlock(1, 0, J.block(3, 0, 8, 1));
  builder.PushBlock(1, 1, J.block(3, 1, 8, 2));
  BlockSparseMatrix<double> Jblock = builder.Build();

  const VectorXd vhat = VectorXd::LinSpaced(11, 1.0, 11.0);
  const VectorXd R = 1.5 * VectorXd::LinSpaced(11, 1.0, 11.0);

  SapConstraintBundle<double> bundle(std::move(Jblock), VectorXd(vhat),
                                     VectorXd(R), std::move(constraints));

  EXPECT_EQ(bundle.num_constraints(), 5);
  EXPECT_EQ(bundle.J().rows(), 11);
  EXPECT_EQ(bundle.J().cols(), 6);

  // Verify multiplication by Jacobian.
  const VectorXd v = VectorXd::LinSpaced(6, 1.0, 6.0);  // arbitrary non-zero v.
  const VectorXd Jv_expected = J * v;
  VectorXd Jv(11);
  bundle.MultiplyByJacobian(v, &Jv);
  EXPECT_TRUE(CompareMatrices(Jv, Jv_expected));

  // Verify multiplication by Jacobian transpose.
  // arbitrary non-zero vector tau.
  const VectorXd tau = VectorXd::LinSpaced(11, 1.0, 11.0);
  const VectorXd JTtau_expected = J.transpose() * tau;
  VectorXd JTtau(6);
  bundle.MultiplyByJacobianTranspose(tau, &JTtau);
  PRINT_VAR(JTtau.transpose());
  EXPECT_TRUE(CompareMatrices(JTtau, JTtau_expected));

  // Verify CalcUnprojectedImpulses() produces y = −R⁻¹⋅(vc−v̂).
  const VectorXd vc = VectorXd::LinSpaced(11, -1.0, 5.0);
  VectorXd y(11);
  bundle.CalcUnprojectedImpulses(vc, &y);
  const VectorXd y_expected = R.cwiseInverse().asDiagonal() * (vhat - vc);
  PRINT_VARn(y.transpose());
  EXPECT_TRUE(CompareMatrices(y, y_expected));

  // Each constraint projects the corresponding segment yᵢ of y into the convex
  // set Cᵢ defined by the constraint using the norm defined by the positive
  // diagonal matrix Rᵢ. In this test we do not verify the projection function,
  // but we only verify that the bundel correctly composes each projection
  // function into the full projection P of the entire bundle.
  // TestConstraint implements the test projection (not really a projection but
  // not relevant for this test) gammaᵢ = paramᵢ * Rᵢ * yᵢ. At construction, we
  // specified paramᵢ = i + 1.

  // clang-format off
  const VectorXd all_params = (VectorXd(11) <<
    1.,          // constraint 0, param = 1., of size 1.
    2., 2.,      // constraint 1, param = 2., of size 2.
    3., 3., 3.,  // constraint 2, param = 3., of size 3.
    4., 4.,      // constraint 3, param = 4., of size 2.
    5., 5., 5.   // constraint 4, param = 5., of size 3.
  ).finished();
  // clang-format on

  const VectorXd gamma_expected = all_params.array() * R.array() * y.array();

  VectorXd gamma(11);
  std::vector<MatrixXd> dPdy(5);
  bundle.ProjectImpulses(y, R, &gamma, &dPdy);
  PRINT_VAR(gamma.transpose());
  EXPECT_TRUE(CompareMatrices(gamma, gamma_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // For this case we expect dPdyᵢ = paramᵢ * Rᵢ.
  std::vector<MatrixXd> dPdy_expected;  
  int offset = 0;
  const VectorXi constraint_size = (VectorXi(5) << 1, 2, 3, 2, 3).finished();
  for (int i = 0; i < 5; ++i) {
    const int size = constraint_size(i);
    const double param = i + 1;
    const MatrixXd dPdy_i = R.segment(offset, size).asDiagonal() * param;
    offset += size;
    dPdy_expected.push_back(dPdy_i);
  }

  for (int i = 0; i < bundle.num_constraints(); ++i) {
    PRINT_VARn(dPdy[i]);
    EXPECT_TRUE(CompareMatrices(dPdy[i], dPdy_expected[i],
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));                                
  }

  // Verify the computation of the Hessian. For this case we expect
  // Gᵢ = paramᵢ * Iᵢ₊₁.
  std::vector<MatrixXd> G(11);
  gamma.setZero();  // trash previous computation.
  bundle.ProjectImpulsesAndCalcConstraintsHessian(y, R, &gamma, &G);
  EXPECT_TRUE(CompareMatrices(gamma, gamma_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // G = ∇²ℓ = d²ℓ/dvc² = dP/dy⋅R⁻¹ = 
  const VectorXd& Rinv = bundle.Rinv();
  EXPECT_EQ(Rinv.size(), 11);
  std::vector<MatrixXd> G_expected(5);
  offset = 0;
  for (int i = 0; i < 5; ++i) {
    const int size = constraint_size(i);
    const double param = i + 1;
    G_expected[i].resize(size, size);
    G_expected[i] = param * MatrixXd::Identity(size, size);
    offset += size;
  }
  for (int i = 0; i < bundle.num_constraints(); ++i) {
    PRINT_VARn(G[i]);
    EXPECT_TRUE(CompareMatrices(G[i], G_expected[i],
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));                                
  }
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake