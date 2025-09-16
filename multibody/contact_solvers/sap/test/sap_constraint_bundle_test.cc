#include "drake/multibody/contact_solvers/sap/sap_constraint_bundle.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
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
// projection function such that gamma = -param * vc.
template <typename T>
class TestConstraint final : public SapConstraint<T> {
 public:
  TestConstraint(int clique, int size, T param)
      : SapConstraint<T>({clique, MakeJacobian(size, clique + 1)}, {}),
        param_(param) {}

  TestConstraint(int clique0, int clique1, int size, T param)
      : SapConstraint<T>({clique0, MakeJacobian(size, clique0 + 1), clique1,
                          MakeJacobian(size, clique1 + 1)},
                         {}),
        param_(param) {}

  // Implements a fake projection operation where the result is given by:
  //   cost = 0.5 * param * ||vc||^2
  //   gamma = -param * vc, and
  //   G = param * Id
  // This is only meant to verify input and output arguments are properly
  // sliced by Constraintbundle.
  std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const final {
    // Data to store constraint velocities vc only.
    return AbstractValue::Make(VectorX<T>(this->num_constraint_equations()));
  }
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* data) const final {
    data->get_mutable_value<VectorX<T>>() = vc;
  }
  T DoCalcCost(const AbstractValue& data) const final {
    const VectorX<T> vc = data.get_value<VectorX<T>>();
    return 0.5 * param_ * vc.squaredNorm();
  }
  void DoCalcImpulse(const AbstractValue& data,
                     EigenPtr<VectorX<T>> gamma) const final {
    const VectorX<T> vc = data.get_value<VectorX<T>>();
    *gamma = -param_ * vc;
  }
  void DoCalcCostHessian(const AbstractValue& data, MatrixX<T>* G) const final {
    const VectorX<T> vc = data.get_value<VectorX<T>>();
    const int nk = vc.size();
    *G = param_ * MatrixX<T>::Identity(nk, nk);
  }

 private:
  TestConstraint(const TestConstraint&) = default;

  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<TestConstraint<T>>(new TestConstraint<T>(*this));
  }

  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    throw std::runtime_error("DoToDouble() not needed for these unit tests.");
  }

  T param_{0.0};
};

namespace {

// This testing fixture sets up an arbitrary contact problem with three cliques
// and seven constraints. It then makes a SapConstraintBundle for that problem
// that we test in the specific test instances below.
class SapConstraintBundleTest : public ::testing::Test {
 public:
  void SetUp() override {
    const double time_step = 1.0e-3;
    std::vector<MatrixX<AutoDiffXd>> A = {MatrixX<AutoDiffXd>::Ones(1, 1),
                                          MatrixX<AutoDiffXd>::Ones(2, 2),
                                          MatrixX<AutoDiffXd>::Ones(3, 3)};
    VectorX<AutoDiffXd> v_star = VectorX<AutoDiffXd>::LinSpaced(6, 1., 6.);
    problem_ = std::make_unique<SapContactProblem<AutoDiffXd>>(
        time_step, std::move(A), std::move(v_star));
    // First cluster of constraints between cliques 0 and 2.
    problem_->AddConstraint(
        std::make_unique<TestConstraint<AutoDiffXd>>(0, 2, 1, 1.0));
    problem_->AddConstraint(
        std::make_unique<TestConstraint<AutoDiffXd>>(2, 0, 2, 2.0));
    // A second cluster of constraints between cliques 0 and 1.
    problem_->AddConstraint(
        std::make_unique<TestConstraint<AutoDiffXd>>(0, 1, 3, 3.0));
    problem_->AddConstraint(
        std::make_unique<TestConstraint<AutoDiffXd>>(0, 1, 2, 4.0));
    problem_->AddConstraint(
        std::make_unique<TestConstraint<AutoDiffXd>>(0, 1, 3, 5.0));
    // A third cluster only involving clique 1.
    problem_->AddConstraint(
        std::make_unique<TestConstraint<AutoDiffXd>>(1, 4, 6.0));
    problem_->AddConstraint(
        std::make_unique<TestConstraint<AutoDiffXd>>(1, 2, 7.0));

    delassus_diagonal_.resize(17);
    delassus_diagonal_ = VectorX<AutoDiffXd>::LinSpaced(17, 1., 7.0);

    bundle_ = std::make_unique<SapConstraintBundle<AutoDiffXd>>(
        problem_.get(), delassus_diagonal_);
  }

 protected:
  std::unique_ptr<SapContactProblem<AutoDiffXd>> problem_;
  VectorX<AutoDiffXd> delassus_diagonal_;
  std::unique_ptr<SapConstraintBundle<AutoDiffXd>> bundle_;
};

TEST_F(SapConstraintBundleTest, VerifyJacobian) {
  EXPECT_EQ(bundle_->num_constraints(), problem_->num_constraints());
  EXPECT_EQ(bundle_->num_constraint_equations(),
            problem_->num_constraint_equations());
  EXPECT_EQ(bundle_->J().rows(), problem_->num_constraint_equations());
  EXPECT_EQ(bundle_->J().cols(), problem_->num_velocities());

  // Build the expected block sparse Jacobian.
  const PartialPermutation& p = problem_->graph().participating_cliques();
  BlockSparseMatrixBuilder<AutoDiffXd> builder(3, 3, 5);
  // Cluster of constraints between cliques 0 and 2.
  MatrixX<AutoDiffXd> J_cluster0_clique0(3, 1);
  J_cluster0_clique0
      << problem_->get_constraint(0).first_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(1).second_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(0, p.permuted_index(0), J_cluster0_clique0);
  MatrixX<AutoDiffXd> J_cluster0_clique2(3, 3);
  J_cluster0_clique2
      << problem_->get_constraint(0).second_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(1).first_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(0, p.permuted_index(2), J_cluster0_clique2);

  // Cluster of constraints between cliques 0 and 1.
  MatrixX<AutoDiffXd> J_cluster1_clique0(8, 1);
  J_cluster1_clique0
      << problem_->get_constraint(2).first_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(3).first_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(4).first_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(1, p.permuted_index(0), J_cluster1_clique0);
  MatrixX<AutoDiffXd> J_cluster1_clique1(8, 2);
  J_cluster1_clique1
      << problem_->get_constraint(2).second_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(3).second_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(4).second_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(1, p.permuted_index(1), J_cluster1_clique1);

  // Cluster with only clique1.
  MatrixX<AutoDiffXd> J_cluster2_clique1(6, 2);
  J_cluster2_clique1
      << problem_->get_constraint(5).first_clique_jacobian().MakeDenseMatrix(),
      problem_->get_constraint(6).first_clique_jacobian().MakeDenseMatrix();
  builder.PushBlock(2, p.permuted_index(1), J_cluster2_clique1);

  BlockSparseMatrix<AutoDiffXd> Jblock = builder.Build();

  EXPECT_EQ(bundle_->J().MakeDenseMatrix(), Jblock.MakeDenseMatrix());
}

TEST_F(SapConstraintBundleTest, VerifyGradients) {
  const AutoDiffXd time_step = 0.02;
  SapConstraintBundleData data =
      bundle_->MakeData(time_step, delassus_diagonal_);
  const VectorXd vc = VectorXd::LinSpaced(17, -1.0, 2.5);  // Arbitrary values.
  const VectorX<AutoDiffXd> vc_ad = drake::math::InitializeAutoDiff(vc);
  bundle_->CalcData(vc_ad, &data);
  const AutoDiffXd cost = bundle_->CalcCost(data);
  VectorX<AutoDiffXd> gamma(vc.size());
  bundle_->CalcImpulses(data, &gamma);
  const VectorXd minus_cost_gradient = -cost.derivatives();
  EXPECT_TRUE(CompareMatrices(gamma, minus_cost_gradient,
                              20 * std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // Extract Hessian computed by automatic differentiation of gamma.
  MatrixXd Gdense = -math::ExtractGradient(gamma, gamma.size());
  std::vector<MatrixXd> minus_gamma_gradient(problem_->num_constraints());
  int offset = 0;
  for (int k = 0; k < problem_->num_constraints(); ++k) {
    const auto& c = problem_->get_constraint(k);
    const int nk = c.num_constraint_equations();
    MatrixXd& Gk = minus_gamma_gradient[k];
    Gk.resize(nk, nk);
    Gk = Gdense.block(offset, offset, nk, nk);
    offset += nk;
  }

  std::vector<MatrixX<AutoDiffXd>> G(problem_->num_constraints());
  bundle_->CalcImpulsesAndConstraintsHessian(data, &gamma, &G);
  for (int k = 0; k < problem_->num_constraints(); ++k) {
    const MatrixXd Gk = math::ExtractValue(G[k]);
    EXPECT_TRUE(CompareMatrices(Gk, minus_gamma_gradient[k],
                                20 * std::numeric_limits<double>::epsilon(),
                                MatrixCompareType::relative));
  }
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
