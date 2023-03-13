#include "drake/multibody/contact_solvers/sap/sap_constraint_bundle.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Makes an arbitrary non-zero Jacobian matrix where each entry is the linear
// index starting at element (0, 0). Examples:
// MakeJacobian(3, 2) returns:
//  |1 4|
//  |2 5|
//  |3 6|
// MakeJacobian(1, 3) returns:
//  |1 2 3|
MatrixXd MakeJacobian(int rows, int cols) {
  const int size = rows * cols;
  MatrixXd J1d = VectorXd::LinSpaced(size, 1., 1. * size);
  J1d.resize(rows, cols);
  return J1d;
}

// Constraint for unit testing where we assume the number of dofs for `clique`
// is clique + 1. We use this constraint to test that the constraint bundle
// properly composes projections.
// The constraint has a single numeric parameter `param` to define an arbitrary
// projection function within Project() as gamma = param * R * y.
class TestConstraint final : public SapConstraint<double> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestConstraint);

  TestConstraint(int clique, int size, double param)
      : SapConstraint<double>(clique, VectorXd::Zero(size),
                              MakeJacobian(size, clique + 1)),
        param_(param) {}

  TestConstraint(int clique0, int clique1, int size, double param)
      : SapConstraint<double>(clique0, clique1, VectorXd::Zero(size),
                              MakeJacobian(size, clique0 + 1),
                              MakeJacobian(size, clique1 + 1)),
        param_(param) {}

  // Implements a fake projection operation where the result is given by:
  //   gamma = param * R * y, and
  //   dPdy = param * R
  // This is only meant to verify input and output arguments are properly
  // sliced by Constraintbundle.
  void Project(const Eigen::Ref<const VectorX<double>>& y,
               const Eigen::Ref<const VectorX<double>>& R,
               EigenPtr<VectorX<double>> gamma,
               MatrixX<double>* dPdy) const final {
    *gamma = param_ * (R.asDiagonal() * y);
    if (dPdy != nullptr) {
      *dPdy = param_ * R.asDiagonal();
    }
  };

  // Implements a non-zero bias term with arbitrary non-zero values.
  // More precisely vhat = -2.0 * [1, num_constraint_equations()].
  VectorX<double> CalcBiasTerm(const double&, const double&) const final {
    return -2.0 * VectorX<double>::LinSpaced(num_constraint_equations(), 1.0,
                                             num_constraint_equations());
  }

  // Implements regularization with arbitrary non-zero values.
  // More precisely, R = [1, num_constraint_equations()].
  VectorX<double> CalcDiagonalRegularization(const double& time_step,
                                             const double& wi) const final {
    return VectorX<double>::LinSpaced(num_constraint_equations(), 1.0,
                                      num_constraint_equations());
  }

  std::unique_ptr<SapConstraint<double>> Clone() const final {
    return std::make_unique<TestConstraint>(*this);
  }

 private:
  double param_{0.0};
};

namespace {

// This testing fixture sets up an arbitrary contact problem with three cliques
// and seven constraints. It then makes a SapConstraintBundle for that problem
// that we test in the specific test instances below.
class SapConstraintBundleTest : public ::testing::Test {
 public:
  void SetUp() override {
    const double time_step = 1.0e-3;
    std::vector<MatrixXd> A = {MatrixXd::Ones(1, 1), MatrixXd::Ones(2, 2),
                               MatrixXd::Ones(3, 3)};
    VectorXd v_star = VectorXd::LinSpaced(6, 1., 6.);
    problem_ = std::make_unique<SapContactProblem<double>>(
        time_step, std::move(A), std::move(v_star));
    // First cluster of constraints between cliques 0 and 2.
    problem_->AddConstraint(std::make_unique<TestConstraint>(0, 2, 1, 1.0));
    problem_->AddConstraint(std::make_unique<TestConstraint>(2, 0, 2, 2.0));
    // A second cluster of constraints between cliques 0 and 1.
    problem_->AddConstraint(std::make_unique<TestConstraint>(0, 1, 3, 3.0));
    problem_->AddConstraint(std::make_unique<TestConstraint>(0, 1, 2, 4.0));
    problem_->AddConstraint(std::make_unique<TestConstraint>(0, 1, 3, 5.0));
    // A third cluster only involving clique 1.
    problem_->AddConstraint(std::make_unique<TestConstraint>(1, 4, 6.0));
    problem_->AddConstraint(std::make_unique<TestConstraint>(1, 2, 7.0));

    delassus_diagonal_.resize(7);
    delassus_diagonal_ = VectorXd::LinSpaced(7, 1., 7.0);

    bundle_ = std::make_unique<SapConstraintBundle<double>>(problem_.get(),
                                                            delassus_diagonal_);
  }

 protected:
  std::unique_ptr<SapContactProblem<double>> problem_;
  VectorXd delassus_diagonal_;
  std::unique_ptr<SapConstraintBundle<double>> bundle_;
};

TEST_F(SapConstraintBundleTest, VerifyJacobian) {
  EXPECT_EQ(bundle_->num_constraints(), problem_->num_constraints());
  EXPECT_EQ(bundle_->num_constraint_equations(),
            problem_->num_constraint_equations());
  EXPECT_EQ(bundle_->J().rows(), problem_->num_constraint_equations());
  EXPECT_EQ(bundle_->J().cols(), problem_->num_velocities());

  // Build the expected block sparse Jacobian.
  const PartialPermutation& p = problem_->graph().participating_cliques();
  BlockSparseMatrixBuilder<double> builder(3, 3, 5);
  // Cluster of constraints between cliques 0 and 2.
  MatrixXd J_cluster0_clique0(3, 1);
  J_cluster0_clique0
      << problem_->get_constraint(0).first_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(1).second_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(0, p.permuted_index(0), J_cluster0_clique0);
  MatrixXd J_cluster0_clique2(3, 3);
  J_cluster0_clique2
      << problem_->get_constraint(0).second_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(1).first_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(0, p.permuted_index(2), J_cluster0_clique2);

  // Cluster of constraints between cliques 0 and 1.
  MatrixXd J_cluster1_clique0(8, 1);
  J_cluster1_clique0
      << problem_->get_constraint(2).first_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(3).first_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(4).first_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(1, p.permuted_index(0), J_cluster1_clique0);
  MatrixXd J_cluster1_clique1(8, 2);
  J_cluster1_clique1
      << problem_->get_constraint(2).second_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(3).second_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(4).second_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(1, p.permuted_index(1), J_cluster1_clique1);

  // Cluster with only clique1.
  MatrixXd J_cluster2_clique1(6, 2);
  J_cluster2_clique1
      << problem_->get_constraint(5).first_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(6).first_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(2, p.permuted_index(1), J_cluster2_clique1);

  BlockSparseMatrix<double> Jblock = builder.Build();

  EXPECT_EQ(bundle_->J().MakeDenseMatrix(), Jblock.MakeDenseMatrix());
}

// Verify CalcUnprojectedImpulses() produces y = −R⁻¹⋅(vc−v̂).
TEST_F(SapConstraintBundleTest, CalcUnprojectedImpulses) {
  const VectorXd& R = bundle_->R();
  const VectorXd& vhat = bundle_->vhat();
  const VectorXd vc =
      VectorXd::LinSpaced(problem_->num_constraint_equations(), -1.0, 5.0);
  VectorXd y(problem_->num_constraint_equations());
  bundle_->CalcUnprojectedImpulses(vc, &y);
  const VectorXd y_expected = R.cwiseInverse().asDiagonal() * (vhat - vc);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

TEST_F(SapConstraintBundleTest, ProjectImpulses) {
  // Each constraint projects the corresponding segment yᵢ of y into the convex
  // set Cᵢ defined by the constraint using the norm defined by the positive
  // diagonal matrix Rᵢ. In this test we do not verify the projection function,
  // but we only verify that the bundle correctly composes each projection
  // function into the full projection P for the entire bundle.
  // TestConstraint implements the test projection (not really a projection but
  // not relevant for this test) gammaᵢ = paramᵢ * Rᵢ * yᵢ. At construction, we
  // specified paramᵢ = i + 1.

  // clang-format off
  const VectorXd all_params = (VectorXd(problem_->num_constraint_equations()) <<
    1.,              // constraint 0, param = 1., of size 1.
    2., 2.,          // constraint 1, param = 2., of size 2.
    3., 3., 3.,      // constraint 2, param = 3., of size 3.
    4., 4.,          // constraint 3, param = 4., of size 2.
    5., 5., 5.,      // constraint 4, param = 5., of size 3.
    6., 6., 6., 6.,  // constraint 5, param = 6., of size 4.
    7., 7.)          // constraint 6, param = 7., of size 2.
  .finished();
  // clang-format on

  const auto& R = bundle_->R();
  const VectorXd y =
      VectorXd::LinSpaced(problem_->num_constraint_equations(), 1., 17.);
  const VectorXd gamma_expected = all_params.array() * R.array() * y.array();

  VectorXd gamma(problem_->num_constraint_equations());
  std::vector<MatrixXd> dPdy(problem_->num_constraints());
  bundle_->ProjectImpulses(y, &gamma, &dPdy);
  EXPECT_TRUE(CompareMatrices(gamma, gamma_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // For this case we expect dPdyᵢ = paramᵢ * Rᵢ.
  std::vector<MatrixXd> dPdy_expected;
  int offset = 0;
  const VectorXi constraint_size =
      (VectorXi(problem_->num_constraints()) << 1, 2, 3, 2, 3, 4, 2).finished();
  for (int i = 0; i < problem_->num_constraints(); ++i) {
    const int size = constraint_size(i);
    const double param = i + 1;
    const MatrixXd dPdy_i = R.segment(offset, size).asDiagonal() * param;
    offset += size;
    dPdy_expected.push_back(dPdy_i);
  }

  for (int i = 0; i < bundle_->num_constraints(); ++i) {
    EXPECT_TRUE(CompareMatrices(dPdy[i], dPdy_expected[i],
                                std::numeric_limits<double>::epsilon(),
                                MatrixCompareType::relative));
  }

  // Verify the computation of the Hessian. For this case we expect
  // Gᵢ = paramᵢ * Iᵢ₊₁.
  std::vector<MatrixXd> G(problem_->num_constraints());
  bundle_->ProjectImpulsesAndCalcConstraintsHessian(y, &gamma, &G);
  EXPECT_TRUE(CompareMatrices(gamma, gamma_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // G = ∇²ℓ = d²ℓ/dvc² = dP/dy⋅R⁻¹
  std::vector<MatrixXd> G_expected(problem_->num_constraints());
  offset = 0;
  for (int i = 0; i < problem_->num_constraints(); ++i) {
    const int size = constraint_size(i);
    const double param = i + 1;
    G_expected[i].resize(size, size);
    G_expected[i] = param * MatrixXd::Identity(size, size);
    offset += size;
  }
  for (int i = 0; i < bundle_->num_constraints(); ++i) {
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
