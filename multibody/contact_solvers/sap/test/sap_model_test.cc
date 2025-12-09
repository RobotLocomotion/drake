#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

class SapModelTester {
 public:
  template <typename T>
  static const VectorX<T>& delassus_diagonal(const SapModel<T>& model) {
    return model.const_model_data_.delassus_diagonal;
  }
};

namespace {

// With SAP we can model implicit springs using constraints. For these
// constraint the projection is the identity, i.e. γ = P(y) = y.
// For testing purposes, this is a simple constraint that models a spring
// between a particle mass and the origin. The spring has stiffness k and
// damping d = tau_d * k, where tau_d is the dissipation time scale. That is,
// the force applied by this constraint on the mass is γ/δt = −k⋅x − d⋅v, where
// x is the (3D) position of the mass and v its (3D) velocity.
template <typename T>
class SpringConstraint final : public SapConstraint<T> {
 public:
  struct Data {
    Vector3<T> vc{Vector3d::Constant(kNaN)};
    T R{kNaN};
    Vector3<T> v_hat{Vector3d::Constant(kNaN)};
  };

  // Model a spring attached to `clique`, expected to be a 3D particle.
  SpringConstraint(int clique, Vector3<T> x0, T k, T tau_d)
      // N.B. For this constraint the Jacobian is the identity matrix.
      : SapConstraint<T>({clique, Matrix3<T>::Identity()}, {}),
        x0_(std::move(x0)),
        k_(k),
        tau_d_(tau_d) {}

  std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step, const Eigen::Ref<const VectorX<T>>&) const final {
    using std::max;
    Data data;
    // Bias and regularization setup so that:
    //   γ = y = -δt⋅(k⋅x + d⋅v) = −R⁻¹⋅(v−v̂).
    data.R = 1. / (time_step * (time_step + tau_d_) * k_);
    data.v_hat = -x0_ / (time_step + tau_d_);
    return AbstractValue::Make(data);
  }

  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* data) const final {
    data->get_mutable_value<Data>().vc = vc;
  }

  T DoCalcCost(const AbstractValue& data) const final {
    const Vector3<T>& vc = data.get_value<Data>().vc;
    const Vector3<T>& v_hat = data.get_value<Data>().v_hat;
    const T& R = data.get_value<Data>().R;
    const T cost = 0.5 / R * (vc - v_hat).squaredNorm();
    return cost;
  }

  void DoCalcImpulse(const AbstractValue& data,
                     EigenPtr<VectorX<T>> gamma) const final {
    const Vector3<T>& vc = data.get_value<Data>().vc;
    const Vector3<T>& v_hat = data.get_value<Data>().v_hat;
    const T& R = data.get_value<Data>().R;
    *gamma = -(vc - v_hat) / R;
  }

  void DoCalcCostHessian(const AbstractValue& data, MatrixX<T>* G) const final {
    const T& R = data.get_value<Data>().R;
    *G = Matrix3<T>::Identity() / R;
  }

 private:
  SpringConstraint(const SpringConstraint&) = default;

  std::unique_ptr<SapConstraint<T>> DoClone() const override {
    return std::unique_ptr<SpringConstraint<T>>(new SpringConstraint<T>(*this));
  }

  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    throw std::runtime_error("DoToDouble() not used in these unit tests.");
  }

  VectorX<T> x0_;  // Previous time step configuration.
  T k_{0.0};       // Stiffness, in N/m.
  T tau_d_{0.0};   // Dissipation time scale, in seconds.
};

// Sets up a simple problem for two 3D particles, six DOFs.
// The first mass is connected to the origin by a spring-damper while the second
// mass is free.
// The problem is setup as two distinct cliques, one for each mass. The
// spring-damper is modeled as a SpringConstraint.
// Since only the first mass is constrained, we know that the SapModel will only
// consider the dynamics of the first mass connected to the origin by the
// spring-damper.
template <typename T>
class SpringMassModel {
 public:
  SpringMassModel() = default;

  // Data accessors.
  double mass1() const { return mass1_; }
  double mass2() const { return mass2_; }
  double time_step() const { return time_step_; }
  double gravity() const { return gravity_; }

  // Make a SapContactProblem for this model at the state described by positions
  // q and velocities v. q.head<3>() and v.head<3>() correspond to the state for
  // the first mass while q.tail<3>() and v.tail<3>() correspond to the state
  // for the second mass.
  std::unique_ptr<SapContactProblem<T>> MakeContactProblem(
      const Vector6<T>& q, const Vector6<T>& v) {
    std::vector<MatrixX<T>> A = {mass1_ * Matrix3<T>::Identity(),
                                 mass2_ * Matrix3<T>::Identity()};
    const Vector6<T> g =
        gravity_ *
        (Vector6<T>() << Vector3<T>::UnitZ(), Vector3<T>::UnitZ()).finished();
    VectorX<T> v_star = v - time_step_ * g;
    auto problem = std::make_unique<SapContactProblem<T>>(
        time_step_, std::move(A), std::move(v_star));
    problem->AddConstraint(std::make_unique<SpringConstraint<T>>(
        0, q.template head<3>(), stiffness_, dissipation_time_scale_));
    return problem;
  }

 protected:
  double time_step_{1.0e-3};
  double mass1_{1.5};
  double mass2_{3.0};
  double stiffness_{100.0};
  double dissipation_time_scale_{0.1};
  double gravity_{10.0};
};

// This fixture setsup a SpringMassModel to test SapModel.
class SpringMassTest : public ::testing::Test {
 public:
  void SetUp() override { MakeModel(Vector6d::Zero(), Vector6d::Zero()); }
  void MakeModel(const VectorXd& q0, const VectorXd& v0) {
    sap_problem_ = model_.MakeContactProblem(q0, v0);
    // Sanity check problem sizes.
    EXPECT_EQ(sap_problem_->num_cliques(), 2);
    EXPECT_EQ(sap_problem_->num_velocities(), 6);
    EXPECT_EQ(sap_problem_->num_constraints(), 1);
    EXPECT_EQ(sap_problem_->num_constraint_equations(), 3);
    sap_model_ = std::make_unique<SapModel<double>>(sap_problem_.get());
    context_ = sap_model_->MakeContext();
  }

 protected:
  SpringMassModel<double> model_;
  std::unique_ptr<SapContactProblem<double>> sap_problem_;
  std::unique_ptr<SapModel<double>> sap_model_;
  std::unique_ptr<systems::Context<double>> context_;
};

TEST_F(SpringMassTest, Sizes) {
  // While the SapProblem has two cliques, six velocities, in the SapModel only
  // one clique with three velocities participates. The second clique is not
  // connected by any constraint.
  EXPECT_EQ(sap_model_->num_cliques(), 1);
  EXPECT_EQ(sap_model_->num_velocities(), 3);
  EXPECT_EQ(sap_model_->num_constraints(), 1);
  EXPECT_EQ(sap_model_->num_constraint_equations(), 3);
}

// Here we verify the correctness of the permutation for velocities. Since only
// the first clique participates, we expect the permutation to only extract the
// velocities corresponding to this first clique.
TEST_F(SpringMassTest, VelocitiesPermutation) {
  const Vector6d v = Vector6d::LinSpaced(6, 1.0, 6.0);
  Vector3d v1;
  sap_model_->velocities_permutation().Apply(v, &v1);
  const Vector3d v1_expected(1., 2., 3.);
  EXPECT_EQ(v1, v1_expected);
}

// Since only the first clique participates, we expect the problem data to
// correspond to that of the first spring only.
TEST_F(SpringMassTest, ProblemData) {
  // We setup the problem with two distinct initial velocities so that v* for
  // each clique is also distinct and we can tell them apart.
  const Vector3d v1(1., 2., 3.);
  const Vector3d v2(4., 5., 6.);
  const Vector6d v = (Vector6d() << v1, v2).finished();
  MakeModel(Vector6d::Zero(), v);
  EXPECT_EQ(sap_model_->time_step(), sap_problem_->time_step());
  // We expect the mass matrix for the first mass only since it is the only one
  // connected by a constraint.
  const std::vector<MatrixXd> A = {model_.mass1() * Matrix3d::Identity()};
  EXPECT_EQ(sap_model_->dynamics_matrix(), A);
  const Vector3d v_star =
      v1 - model_.time_step() * model_.gravity() * Vector3d::UnitZ();
  EXPECT_EQ(sap_model_->v_star(), v_star);
  const Vector3d p_star = model_.mass1() * v_star;
  EXPECT_EQ(sap_model_->p_star(), p_star);

  // Verify diagonal approximation of the Delassus operator.
  // For this case, J = I₃ and M = m₁⋅I₃. Therefore W = J⋅M⁻¹⋅Jᵀ = I₃/m₁.
  // Then the diagonal approximation is ‖W‖ᵣₘₛ = ‖W‖/3 = (m₁√3)⁻¹.
  const VectorXd W_diag = SapModelTester::delassus_diagonal(*sap_model_);
  const VectorXd W_diag_expected =
      Vector3d::Constant(1.0 / model_.mass1() / sqrt(3.0));
  EXPECT_TRUE(CompareMatrices(W_diag, W_diag_expected, kEpsilon,
                              MatrixCompareType::relative));
}

TEST_F(SpringMassTest, StateAccess) {
  const Vector3d v(1., 2., 3.);
  sap_model_->SetVelocities(v, context_.get());
  EXPECT_EQ(sap_model_->GetVelocities(*context_), v);
}

TEST_F(SpringMassTest, EvalConstraintVelocities) {
  const Vector3d v(1., 2., 3.);
  sap_model_->SetVelocities(v, context_.get());
  EXPECT_EQ(sap_model_->EvalConstraintVelocities(*context_), v);
}

TEST_F(SpringMassTest, EvalMomentumGain) {
  const Vector3d v(1., 2., 3.);
  sap_model_->SetVelocities(v, context_.get());
  const VectorXd& v_star = sap_model_->v_star();
  const VectorXd momentum_gain = model_.mass1() * (v - v_star);
  EXPECT_EQ(sap_model_->EvalMomentumGain(*context_), momentum_gain);
}

TEST_F(SpringMassTest, EvalMomentumCost) {
  const Vector3d v(1., 2., 3.);
  sap_model_->SetVelocities(v, context_.get());
  const VectorXd& v_star = sap_model_->v_star();
  const double momentum_cost =
      0.5 * model_.mass1() * (v - v_star).squaredNorm();
  EXPECT_EQ(sap_model_->EvalMomentumCost(*context_), momentum_cost);
}

// Fake constraint used for unit testing, see DummyModel.
template <typename T>
class DummyConstraint final : public SapConstraint<T> {
 public:
  DummyConstraint(int clique, MatrixX<T> J, VectorX<T> R, VectorX<T> v_hat)
      // N.B. For this constraint the Jacobian is the identity matrix.
      : SapConstraint<T>({clique, std::move(J)}, {}),
        R_(std::move(R)),
        v_hat_(std::move(v_hat)) {}

  DummyConstraint(int clique1, MatrixX<T> J1, int clique2, MatrixX<T> J2,
                  VectorX<T> R, VectorX<T> v_hat)
      // N.B. For this constraint the Jacobian is the identity matrix.
      : SapConstraint<T>({clique1, std::move(J1), clique2, std::move(J2)}, {}),
        R_(std::move(R)),
        v_hat_(std::move(v_hat)) {}

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
    const VectorX<T>& vc = data.get_value<VectorX<T>>();
    VectorX<T> gamma(vc.size());
    this->CalcImpulse(data, &gamma);
    const T cost = 0.5 * gamma.dot(R_.asDiagonal() * gamma);
    return cost;
  }
  void DoCalcImpulse(const AbstractValue& data,
                     EigenPtr<VectorX<T>> gamma) const final {
    const VectorX<T>& vc = data.get_value<VectorX<T>>();
    *gamma = -(R_.cwiseInverse().asDiagonal() * (vc - v_hat_));
  }
  void DoCalcCostHessian(const AbstractValue&, MatrixX<T>* G) const final {
    *G = R_.cwiseInverse().asDiagonal();
  }

 private:
  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<DummyConstraint<T>>(new DummyConstraint<T>(*this));
  }

  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    throw std::runtime_error("DoToDouble() not used in these unit tests.");
  }

  VectorX<T> R_;
  VectorX<T> v_hat_;
};

// Class to build a fake SapContactProblem. The requirements for these unit
// tests are:
//  - Non trivial numerical values, i.e. different from zero or identity
//    matrices.
//  - Non trivial graph.
//  - Non trivial projections, to validate gradients.
//  - Though numerical values are arbitrary, still they verify the problem's
//    requirements. E.g.: dynamics matrix A is SPD and regularization is
//    positive.
//  - All values are known so that we can extract them to verify the results.
template <typename T>
class DummyModel {
 public:
  // Hardcoded fixed sizes for this model.
  static constexpr int kNumCliques{3};
  static constexpr int kNumConstraints{4};
  static constexpr int kNumTotalConstraintEquations{16};
  static constexpr int kNumTotalParticipatingDofs{9};

  DummyModel() {
    // Arbitrary non-identity SPD matrices to build the dynamics matrix A.
    // clang-format off
    const Eigen::Matrix2d S22 =
      (Eigen::Matrix2d() << 2, 1,
                            1, 2).finished();
    const Eigen::Matrix3d S33 =
      (Eigen::Matrix3d() << 4, 1, 2,
                            1, 5, 3,
                            2, 3, 6).finished();
    const Eigen::Matrix4d S44 =
      (Eigen::Matrix4d() << 7, 1, 2, 3,
                            1, 8, 4, 5,
                            2, 4, 9, 6,
                            3, 5, 6, 10).finished();
    // clang-format on
    dynamics_matrix_ = {S22, S33, S44};
    v_star_ = VectorX<T>::LinSpaced(num_velocities_, 1., 1. * num_velocities_);
  }

  // Data accessors.
  int num_velocities() const { return num_velocities_; }
  double time_step() const { return time_step_; }
  const std::vector<MatrixX<T>>& dynamics_matrix() const {
    return dynamics_matrix_;
  }
  const VectorX<T>& v_star() const { return v_star_; }

  std::unique_ptr<SapContactProblem<T>> MakeContactProblem() {
    auto problem = std::make_unique<SapContactProblem<T>>(
        time_step_, dynamics_matrix_, v_star_);

    // N.B. Constraints below are added in a somewhat arbitrary order. However,
    // we make sure that these constraints reference cliques in an order
    // that is not contiguous. That is, below we reference cliques 1 and 2
    // first, followed by clique 0. This will lead to a SAP model with a
    // non-trivial permutation of cliques (not identity).
    // Similarly, while this arrangement of constraints leads to two clusters,
    // we make sure we don't add the constraints for cluster 0 first followed by
    // those for cluster 1. This also causes a non-trivial permutation of the
    // impulses.
    // In total, we have a test model with non-trivial permutations that stress
    // tests SapModel's implementations better.
    // We verify this invariant later on when the model is created.

    // This will lead to cluster 0, constraining cliques 1 and 2.
    {
      MatrixX<T> J1 = MakeJacobian(5, 3);
      MatrixX<T> J2 = MakeJacobian(5, 4);
      VectorX<T> R = VectorX<T>::LinSpaced(5, 1., 5.);
      VectorX<T> v_hat = 100.0 * R;
      problem->AddConstraint(std::make_unique<DummyConstraint<T>>(
          1, std::move(J1), 2, std::move(J2), std::move(R), std::move(v_hat)));
    }

    // This will lead to cluster 1, constraining clique 0 with itself.
    {
      MatrixX<T> J = MakeJacobian(3, 2);
      VectorX<T> R = VectorX<T>::LinSpaced(3, 1., 3.);
      VectorX<T> v_hat = Vector3d(1., 2., 0.2);
      problem->AddConstraint(std::make_unique<DummyConstraint<T>>(
          0, std::move(J), std::move(R), std::move(v_hat)));
    }

    // This constraint will get added to the already created cluster 0 above.
    {
      MatrixX<T> J1 = MakeJacobian(5, 3);
      MatrixX<T> J2 = MakeJacobian(5, 4);
      VectorX<T> R = VectorX<T>::LinSpaced(5, 1., 5.);
      VectorX<T> v_hat = 100.0 * R;
      problem->AddConstraint(std::make_unique<DummyConstraint<T>>(
          1, std::move(J1), 2, std::move(J2), std::move(R), std::move(v_hat)));
    }

    // This constraint will get added to the already created cluster 1 above.
    {
      MatrixX<T> J = MakeJacobian(3, 2);
      VectorX<T> R = VectorX<T>::LinSpaced(3, 1., 3.);
      VectorX<T> v_hat = Vector3d(1., 2., 0.2);
      problem->AddConstraint(std::make_unique<DummyConstraint<T>>(
          0, std::move(J), std::move(R), std::move(v_hat)));
    }

    return problem;
  }

  // Makes the constraints Jacobian J. The order of the velocities and impulses
  // is given as in the original model (not as permuted in SapModel later on).
  MatrixXd MakeModelJacobian() const {
    MatrixXd J = MatrixXd::Zero(kNumTotalConstraintEquations,
                                kNumTotalParticipatingDofs);

    // Constraint 0. Between clique 1 and 2, cluster 0.
    J.block<5, 3>(0, 2) = MakeJacobian(5, 3);
    J.block<5, 4>(0, 5) = MakeJacobian(5, 4);

    // Constraint 1. Within clique 0, cluster 1.
    J.block<3, 2>(5, 0) = MakeJacobian(3, 2);

    // Constraint 2. Between clique 1 and 2, cluster 0.
    J.block<5, 3>(8, 2) = MakeJacobian(5, 3);
    J.block<5, 4>(8, 5) = MakeJacobian(5, 4);

    // Constraint 3. Within clique 0, cluster 1.
    J.block<3, 2>(13, 0) = MakeJacobian(3, 2);

    return J;
  }

 private:
  // Makes an arbitrary non-zero Jacobian matrix where each entry is the linear
  // index starting at element (0, 0). Examples:
  // MakeJacobian(3, 2) returns:
  //  |1 4|
  //  |2 5|
  //  |3 6|
  // MakeJacobian(1, 3) returns:
  //  |1 2 3|
  MatrixXd MakeJacobian(int rows, int cols) const {
    const int size = rows * cols;
    MatrixXd J1d = VectorXd::LinSpaced(size, 1., 1. * size);
    J1d.resize(rows, cols);
    return J1d;
  }

  double time_step_{1.0e-3};
  const int num_velocities_{9};
  std::vector<MatrixX<T>> dynamics_matrix_;
  VectorX<T> v_star_;
};

// Testing fixture that creates a SapModel for a DummyModel.
// Tests in the SpringMassTest fixture allow us to compute the Delassus operator
// approximation by hand for testing. Also, not all DOFs participate, which
// allows us to test the case of partial DOFs permutations in the model.
// In this new DummyModelTest fixture, we introduce non-trivial numeric values
// of the Jacobian matrices, bias terms and regularization. This allows us to
// perform tests on operations such as cost and gradients using non-trivial
// numerical values.
class DummyModelTest : public ::testing::Test {
 public:
  void SetUp() override {
    sap_problem_ = dummy_model_.MakeContactProblem();
    // Sanity check problem sizes.
    EXPECT_EQ(sap_problem_->num_cliques(), dummy_model_.kNumCliques);
    EXPECT_EQ(sap_problem_->num_velocities(),
              dummy_model_.kNumTotalParticipatingDofs);
    EXPECT_EQ(sap_problem_->num_constraints(), dummy_model_.kNumConstraints);
    EXPECT_EQ(sap_problem_->num_constraint_equations(),
              dummy_model_.kNumTotalConstraintEquations);
    sap_model_ = std::make_unique<SapModel<double>>(sap_problem_.get());
    context_ = sap_model_->MakeContext();

    // To make these tests more interesting and stress implementations better,
    // MakeContactProblem() above adds constraints and cliques so that
    // permutations are different from the identity permutation. Here we verify
    // this invariant.
    auto make_identity_permutation = [](int size) {
      std::vector<int> identity_permutation(size);
      std::iota(identity_permutation.begin(), identity_permutation.end(), 0);
      return identity_permutation;
    };
    const std::vector<int> identity_cliques_permutation =
        make_identity_permutation(sap_problem_->num_cliques());
    ASSERT_NE(
        sap_model_->problem().graph().participating_cliques().permutation(),
        identity_cliques_permutation);
    const std::vector<int> identity_velocities_permutation =
        make_identity_permutation(sap_problem_->num_velocities());
    ASSERT_NE(sap_model_->velocities_permutation().permutation(),
              identity_velocities_permutation);
    const std::vector<int> identity_impulses_permutation =
        make_identity_permutation(sap_problem_->num_constraint_equations());
    ASSERT_NE(sap_model_->impulses_permutation().permutation(),
              identity_impulses_permutation);

    // Extract model data.
    const int nv = sap_model_->num_velocities();
    v_star_.resize(nv);
    sap_model_->velocities_permutation().Apply(dummy_model_.v_star(), &v_star_);
    const int num_cliques =
        sap_problem_->graph().participating_cliques().permuted_domain_size();
    dynamics_matrix_.resize(num_cliques);
    sap_problem_->graph().participating_cliques().Apply(
        dummy_model_.dynamics_matrix(), &dynamics_matrix_);
    A_ = MatrixXd::Zero(nv, nv);
    int offset = 0;
    for (const auto& Ablock : dynamics_matrix_) {
      const int size = Ablock.rows();
      A_.block(offset, offset, size, size) = Ablock;
      offset += size;
    }

    // The constraint bundle is tested elsewhere. Therefore we use it here to
    // obtain the data we need for this test.
    J_ = sap_model_->constraints_bundle().J().MakeDenseMatrix();

    // For testing, we make the Jacobian matrix with indexes as specified in the
    // original model.
    J_not_permuted_ = dummy_model_.MakeModelJacobian();
  }

  VectorXd arbitrary_v() const {
    return (VectorXd(dummy_model_.kNumTotalParticipatingDofs) << 0.1, 0.2, 0.3,
            0.4, 0.5, 0.6, 0.7, 0.8, 0.9)
        .finished();
  }

  // Computes the Hessian of the model for the state currently stored in
  // context_. This method helps us unit test
  // SapModel::EvalConstraintsHessian().
  MatrixXd CalcDenseHessian() const {
    MatrixXd H = MatrixXd::Zero(sap_model_->num_velocities(),
                                sap_model_->num_velocities());
    // Build A into H, i.e. H = A.
    int clique_start = 0;
    for (const auto& Ab : sap_model_->dynamics_matrix()) {
      const int clique_size = Ab.rows();
      H.block(clique_start, clique_start, clique_size, clique_size) = Ab;
      clique_start += clique_size;
    }

    // Add regularizer contribution, Jᵀ⋅G⋅J.
    const std::vector<MatrixXd>& G =
        sap_model_->EvalConstraintsHessian(*context_);
    MatrixXd GJ(sap_model_->num_constraint_equations(),
                sap_model_->num_velocities());
    int offset = 0;
    for (int i = 0; i < sap_model_->num_constraints(); ++i) {
      const MatrixXd& Gi = G[i];
      const int ni = Gi.rows();
      GJ.middleRows(offset, ni) = Gi * J_.middleRows(offset, ni);
      offset += ni;
    }
    // H = A + Jᵀ⋅G⋅J.
    H += J_.transpose() * GJ;

    return H;
  }

  // Compute diagonal approximation of the Delassus operator for the
  // SapContactProblem in this test. We use a slightly different implementation
  // from that in SapModel. While SapModel computes the Delassus operator
  // approximation with the constraints ordering dictated by the contact graph,
  // this implementation computes it with the ordering dictated by the
  // SapProblem and permutes the result towards the end. These differences in
  // the implementations are believed to be enough to consider these two results
  // independent of each other. Only a bug in one of them could make the results
  // mismatch.
  VectorXd CalcDelassusDiagonalApproximation() const {
    // First compute the block diagonal approximation of the Delassus operator.
    // Entries in W_approximation are in the original problem's constraint
    // order.
    std::vector<MatrixXd> W_approximation(sap_problem_->num_constraints());
    for (int i = 0; i < sap_problem_->num_constraints(); ++i) {
      // N.B. Traverse constraints in their original order as defined in the
      // contact problem.
      const SapConstraint<double>& constraint = sap_problem_->get_constraint(i);
      const int ni = constraint.num_constraint_equations();
      W_approximation[i].setZero(ni, ni);
      {
        const int c = constraint.first_clique();
        const MatrixXd& A = sap_problem_->dynamics_matrix()[c];
        const VectorXd& A_diag_inv = A.diagonal().cwiseInverse();
        const MatrixXd& J =
            constraint.first_clique_jacobian().MakeDenseMatrix();
        W_approximation[i] += J * A_diag_inv.asDiagonal() * J.transpose();
      }
      if (constraint.num_cliques() == 2) {
        const int c = constraint.second_clique();
        const MatrixXd& A = sap_problem_->dynamics_matrix()[c];
        const VectorXd& A_diag_inv = A.diagonal().cwiseInverse();
        const MatrixXd& J =
            constraint.second_clique_jacobian().MakeDenseMatrix();
        W_approximation[i] += J * A_diag_inv.asDiagonal() * J.transpose();
      }
    }

    // We make cluster_to_problem_index store constraint indexes in the order
    // specified by the SapModel (by clusters).
    std::vector<int> cluster_to_problem_index(sap_problem_->num_constraints());
    const ContactProblemGraph& graph = sap_problem_->graph();
    int i_cluster = 0;
    for (const auto& cluster : graph.clusters()) {
      for (int i_problem : cluster.constraint_index()) {
        cluster_to_problem_index[i_cluster++] = i_problem;
      }
    }

    // Now we compute a diagonal scaling for each constraints by taking the RMS
    // norm of the diagonal block for that constraint.
    // W_diagonal_approximation must be indexed in cluster order.
    VectorXd W_diagonal_approximation =
        VectorXd::Zero(sap_problem_->num_constraint_equations());
    int offset = 0;
    // Traverse constraints in cluster order using cluster_to_problem_index.
    for (int cluster_index = 0; cluster_index < sap_problem_->num_constraints();
         ++cluster_index) {
      const int problem_index =
          cluster_to_problem_index[cluster_index];  // Original index in the
                                                    // problem.
      const SapConstraint<double>& constraint =
          sap_problem_->get_constraint(problem_index);
      const int ni = constraint.num_constraint_equations();
      EXPECT_TRUE(W_approximation[problem_index].rows() == ni);
      W_diagonal_approximation.segment(offset, ni)
          .setConstant(W_approximation[problem_index].norm() /
                       W_approximation[problem_index].rows());
      offset += ni;
    }

    // According to the documentation of
    // SapModel::CalcDelassusDiagonalApproximation(), entries in the Delassus
    // operator diagonal approximation should be ordered by clusters in the
    // graph, not by their original index in the problem. Here we make sure
    // that, for this problem, clusters are not ordered. This way we ensure the
    // test wouldn't accidentally pass because W_diagonal_approximation happens
    // to be trivially ordered by clusters.
    EXPECT_FALSE(std::is_sorted(cluster_to_problem_index.begin(),
                                cluster_to_problem_index.end()));

    return W_diagonal_approximation;
  }

 protected:
  DummyModel<double> dummy_model_;
  std::unique_ptr<SapContactProblem<double>> sap_problem_;
  std::unique_ptr<SapModel<double>> sap_model_;
  std::unique_ptr<systems::Context<double>> context_;

  // Problem data.
  VectorXd v_star_;
  std::vector<MatrixXd> dynamics_matrix_;
  MatrixXd A_;
  MatrixXd J_;
  MatrixXd J_not_permuted_;
};

// Verifies model data.
TEST_F(DummyModelTest, VerifyData) {
  EXPECT_EQ(sap_model_->time_step(), dummy_model_.time_step());
  EXPECT_EQ(sap_model_->v_star(), v_star_);
  EXPECT_TRUE(CompareMatrices(sap_model_->p_star(), A_ * v_star_, kEpsilon,
                              MatrixCompareType::relative));
  const VectorXd Ainv_sqrt = A_.diagonal().cwiseInverse().cwiseSqrt();
  EXPECT_TRUE(CompareMatrices(sap_model_->inv_sqrt_dynamics_matrix(), Ainv_sqrt,
                              kEpsilon, MatrixCompareType::relative));
  VectorXd W_diag_expected = CalcDelassusDiagonalApproximation();
  const VectorXd W_diag = SapModelTester::delassus_diagonal(*sap_model_);
  EXPECT_TRUE(CompareMatrices(W_diag, W_diag_expected, kEpsilon,
                              MatrixCompareType::relative));
}

// To test the permutation on impulses, in this test we compute the constraints
// velocities vc by two different methods:
//  1. vc is computed with EvalConstraintVelocities(), the "expected" value.
//  2. vc is computed with the test Jacobian J_not_permuted_ with velocities and
//     impulses indexes as ordered in the original problem, and the we apply the
//     permutation on impulses.
// We expect these two methods to lead to the same result when the permutation
// is correct.
TEST_F(DummyModelTest, ImpulsesPermutation) {
  const VectorXd v_permuted = arbitrary_v();
  sap_model_->SetVelocities(v_permuted, context_.get());
  // We need velocities v in the original ordering to be consistent with the
  // ordering in J_not_permuted_.
  VectorXd v(v_permuted.size());
  sap_model_->velocities_permutation().ApplyInverse(v_permuted, &v);

  // We generate the expected result with a call to EvalConstraintVelocities().
  const VectorXd& vc_expected = sap_model_->EvalConstraintVelocities(*context_);

  // Constraints velocities in the original order specified in the model.
  const VectorXd vc = J_not_permuted_ * v;

  // To make things more interesting, we verify we are not working with the
  // identity permutation.
  EXPECT_FALSE(
      CompareMatrices(vc, vc_expected, kEpsilon, MatrixCompareType::relative));

  // We now verify the result of applying the permutation.
  VectorXd vc_permuted(vc.size());
  sap_model_->impulses_permutation().Apply(vc, &vc_permuted);
  EXPECT_TRUE(CompareMatrices(vc_permuted, vc_expected, kEpsilon,
                              MatrixCompareType::relative));
}

TEST_F(DummyModelTest, GetMutableVelocities) {
  // Arbitrary velocity value set with SetVelocities().
  const VectorXd v1 = arbitrary_v();
  sap_model_->SetVelocities(v1, context_.get());
  EXPECT_TRUE(CompareMatrices(sap_model_->GetVelocities(*context_), v1));

  // Arbitrary velocity value set with GetMutableVelocities().
  const VectorXd v2 = -3.14 * arbitrary_v();
  sap_model_->GetMutableVelocities(context_.get()) = v2;
  EXPECT_TRUE(CompareMatrices(sap_model_->GetVelocities(*context_), v2));
}

TEST_F(DummyModelTest, EvalMomentum) {
  const VectorXd v = arbitrary_v();
  sap_model_->SetVelocities(v, context_.get());
  const VectorXd& p = sap_model_->EvalMomentum(*context_);
  const VectorXd p_expected = A_ * v;
  EXPECT_TRUE(
      CompareMatrices(p, p_expected, kEpsilon, MatrixCompareType::relative));
}

TEST_F(DummyModelTest, MultiplyByDynamicsMatrix) {
  const VectorXd v = arbitrary_v();
  VectorXd p(sap_model_->num_velocities());
  sap_model_->MultiplyByDynamicsMatrix(v, &p);
  const VectorXd p_expected = A_ * v;
  EXPECT_TRUE(
      CompareMatrices(p, p_expected, kEpsilon, MatrixCompareType::relative));
}

TEST_F(DummyModelTest, MomentumCost) {
  const VectorXd v = arbitrary_v();
  sap_model_->SetVelocities(v, context_.get());
  const double expected_cost =
      0.5 * (v - v_star_).transpose() * A_ * (v - v_star_);
  const double cost = sap_model_->EvalMomentumCost(*context_);
  EXPECT_NEAR(cost, expected_cost, kEpsilon * expected_cost);
}

TEST_F(DummyModelTest, ConstraintVelocities) {
  const VectorXd v = arbitrary_v();
  sap_model_->SetVelocities(v, context_.get());
  const VectorXd& vc = sap_model_->EvalConstraintVelocities(*context_);
  const VectorXd vc_expected = J_ * v;
  EXPECT_TRUE(
      CompareMatrices(vc, vc_expected, kEpsilon, MatrixCompareType::relative));
}

TEST_F(DummyModelTest, Impulses) {
  // Generate reference values. Since the bundle is separately unit tested, we
  // use it here to obtain the expected values.
  const VectorXd v = arbitrary_v();
  sap_model_->SetVelocities(v, context_.get());
  const auto& bundle = sap_model_->constraints_bundle();
  const VectorXd& vc = sap_model_->EvalConstraintVelocities(*context_);
  const VectorXd not_used(vc.size());
  SapConstraintBundleData data =
      bundle.MakeData(sap_model_->time_step(), not_used);
  bundle.CalcData(vc, &data);
  VectorXd gamma_expected(sap_model_->num_constraint_equations());
  bundle.CalcImpulses(data, &gamma_expected);

  // Impulses.
  const VectorXd& gamma = sap_model_->EvalImpulses(*context_);
  EXPECT_TRUE(CompareMatrices(gamma, gamma_expected, kEpsilon,
                              MatrixCompareType::relative));

  // Generalized impulses.
  const VectorXd& j = sap_model_->EvalGeneralizedImpulses(*context_);
  const VectorXd j_expected = J_.transpose() * gamma_expected;
  EXPECT_TRUE(
      CompareMatrices(j, j_expected, kEpsilon, MatrixCompareType::relative));
}

TEST_F(DummyModelTest, PrimalCost) {
  // Since the bundle is separately unit tested, we
  // use it to obtain the expected values of constraints cost.
  const VectorXd v = arbitrary_v();
  sap_model_->SetVelocities(v, context_.get());
  const auto& bundle = sap_model_->constraints_bundle();
  const VectorXd& vc = sap_model_->EvalConstraintVelocities(*context_);
  const VectorXd not_used(vc.size());
  SapConstraintBundleData data =
      bundle.MakeData(sap_model_->time_step(), not_used);
  bundle.CalcData(vc, &data);
  const double constraints_cost = bundle.CalcCost(data);
  const double expected_cost =
      0.5 * (v - v_star_).transpose() * A_ * (v - v_star_) + constraints_cost;

  // Verify the value of the cost returned by the model.
  const double cost = sap_model_->EvalCost(*context_);
  EXPECT_NEAR(cost, expected_cost, kEpsilon * expected_cost);
}

TEST_F(DummyModelTest, CostGradients) {
  // Use automatic differentiation to obtain a reference value to test the
  // gradient computation.
  DummyModel<AutoDiffXd> dummy_model_ad;
  auto sap_problem_ad = dummy_model_ad.MakeContactProblem();
  // Sanity check problem sizes.
  EXPECT_EQ(sap_problem_ad->num_cliques(), dummy_model_.kNumCliques);
  EXPECT_EQ(sap_problem_ad->num_velocities(),
            dummy_model_.kNumTotalParticipatingDofs);
  EXPECT_EQ(sap_problem_ad->num_constraints(), dummy_model_.kNumConstraints);
  EXPECT_EQ(sap_problem_ad->num_constraint_equations(),
            dummy_model_.kNumTotalConstraintEquations);
  auto sap_model_ad =
      std::make_unique<SapModel<AutoDiffXd>>(sap_problem_ad.get());
  auto context_ad = sap_model_ad->MakeContext();
  const VectorXd v = arbitrary_v();
  VectorX<AutoDiffXd> v_ad = drake::math::InitializeAutoDiff(v);
  sap_model_ad->SetVelocities(v_ad, context_ad.get());
  // AutoDiffXd computation of the gradient.
  const AutoDiffXd& cost_ad = sap_model_ad->EvalCost(*context_ad);
  const VectorXd cost_ad_gradient = cost_ad.derivatives();
  // AutoDiffXd computation of the Hessian.
  const VectorX<AutoDiffXd>& gradient_ad =
      sap_model_ad->EvalCostGradient(*context_ad);
  const VectorXd gradient_ad_value = math::ExtractValue(gradient_ad);
  const MatrixXd gradient_ad_gradient = math::ExtractGradient(gradient_ad);

  // Compute the analytical gradient.
  sap_model_->SetVelocities(v, context_.get());
  // Validate cost and its gradient.
  const double cost = sap_model_->EvalCost(*context_);
  const VectorXd& cost_gradient = sap_model_->EvalCostGradient(*context_);
  EXPECT_NEAR(cost, cost_ad.value(), 2.0 * kEpsilon * cost_ad.value());
  EXPECT_TRUE(CompareMatrices(cost_gradient, cost_ad_gradient, 2 * kEpsilon,
                              MatrixCompareType::relative));

  // Validate gradient and its gradient (Hessian of the cost).
  EXPECT_TRUE(CompareMatrices(cost_gradient, gradient_ad_value, kEpsilon,
                              MatrixCompareType::relative));

  // Unit test the validity of the constraints Hessian G by directly forming the
  // Hessian in velocities H = A + Jᵀ⋅G⋅J.
  const MatrixXd cost_hessian = CalcDenseHessian();
  EXPECT_TRUE(CompareMatrices(cost_hessian, gradient_ad_gradient, kEpsilon,
                              MatrixCompareType::relative));
}

// Unit test the computation of the Hessian factorization.
TEST_F(DummyModelTest, EvalHessianFactorization) {
  const VectorXd v = arbitrary_v();
  sap_model_->SetVelocities(v, context_.get());

  // Two arbitrary rhs.
  MatrixXd b(v.size(), 2);
  b << 2.5 * v, -1.2 * v;

  // Copute x = H⁻¹⋅b using the factorization.
  const HessianFactorizationCache& H =
      sap_model_->EvalHessianFactorizationCache(*context_);
  MatrixXd x = b;
  H.SolveInPlace(&x);

  // Compute expected solution.
  const MatrixXd H_expected = CalcDenseHessian();
  MatrixXd x_expected = H_expected.ldlt().solve(b);

  EXPECT_TRUE(CompareMatrices(x, x_expected, 8 * kEpsilon,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
