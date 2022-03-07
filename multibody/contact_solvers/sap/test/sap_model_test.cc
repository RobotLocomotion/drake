#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include <gtest/gtest.h>

#include "drake/multibody/contact_solvers/sap/sap_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

// With SAP we can model implicit springs using constraints. For these
// constraint the projection is the identity, i.e. γ = P(y) = y.
// For testing purposes, this is a simple constraints that models a spring
// between a particle mass and the origin. The spring has stiffness k and
// damping d = tau_d * k, where tau_d is the dissipation time scale. That is,
// the force applied by this constraint on the mass is γ/δt = −k⋅x − d⋅v, where
// x is the (3D) position of the mass and v its (3D) velocity.
template <typename T>
class SpringConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringConstraint);

  // Model a spring attached to `clique`, expected to be a 3D particle.
  explicit SpringConstraint(int clique, Vector3<T> x, T k, T tau_d)
      // N.B. For this constraint the Jacobian is the identity matrix.
      : SapConstraint<T>(clique, std::move(x), Matrix3<T>::Identity()),
        k_(k),
        tau_d_(tau_d) {}

  // Bias and regularization setup so that:
  //   γ = y = -δt⋅(k⋅x + d⋅v) = −R⁻¹⋅(v−v̂).
  VectorX<T> CalcBiasTerm(const T& time_step, const T&) const final {
    return -this->constraint_function() / (time_step + tau_d_);
  }
  VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                        const T&) const final {
    const T R = 1. / (time_step * (time_step + tau_d_) * k_);
    return Vector3<T>(R, R, R);
  }

  // For this constraint the projection is the identity operation.
  void Project(const Eigen::Ref<const VectorX<double>>& y,
               const Eigen::Ref<const VectorX<double>>& R,
               EigenPtr<VectorX<double>> gamma,
               MatrixX<double>* dPdy) const final {
    (*gamma) = y;
    if (dPdy != nullptr) dPdy->setIdentity(3, 3);
  };

 private:
  T k_{0.0};      // Stiffness, in N/m.
  T tau_d_{0.0};  // Dissipation time scale, in N⋅s/m.
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
  // each clique is also distinct and we can tell them appart.
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

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
