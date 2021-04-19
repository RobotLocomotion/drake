#include "drake/multibody/contact_solvers/pgs_solver.h"

#include <memory>

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/sparse_linear_operator.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using SparseMatrixd = Eigen::SparseMatrix<double>;
using Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;
constexpr double kTol = 1e-14;
constexpr int kMaxIterations = 50;

/* A test against analytic solution for a simple contact problem:
 A single point in 3D is in contact with velocity (-1, -1, -1) in the contact
 space without contact force, where the first two components are in arbtrary
 orthogonal directions in the contact plane and the last component is the in the
 normal direction to the contact plane. The friction coefficient is set to be
 large enough to prevent any slipping. Hence, the expected post-solve contact
 velocity is (0, 0, 0). With an arbitrarily prescribed SPD tangent matrix A, the
 contact impulse can also be analytically calculated. The test confirms that the
 results from the PGS solver matches the analytically calculated contact
 velocity and impulse. */
class PgsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    SetSystemDynamicsData();

    PgsSolverParameters params;
    params.abs_tolerance = kTol;
    params.rel_tolerance = kTol;
    params.max_iterations = kMaxIterations;
    pgs_.set_parameters(params);
  }

  // Set the point contact data for a situation with no contact points.
  void SetZeroPointContactData() {
    jacobian_.resize(0, 0);
    Jc_ = std::make_unique<SparseLinearOperator<double>>("Jc", &jacobian_);
    stiffness_.resize(0);
    dissipation_.resize(0);
    penetration_depth_.resize(0);
    mu_.resize(0);
    point_data_ = std::make_unique<PointContactData<double>>(
        &penetration_depth_, Jc_.get(), &stiffness_, &dissipation_, &mu_);
  }

  // Set the point contact data with a single contact point with the given
  // friction coefficient.
  void SetSinglePointContactData(double friction_coeff) {
    jacobian_.resize(3, 3);
    {
      std::vector<Triplet> triplets;
      triplets.emplace_back(0, 0, 1.0);
      triplets.emplace_back(1, 1, 1.0);
      triplets.emplace_back(2, 2, 1.0);
      jacobian_.setFromTriplets(triplets.begin(), triplets.end());
      jacobian_.makeCompressed();
    }
    Jc_ = std::make_unique<SparseLinearOperator<double>>("Jc", &jacobian_);
    // Stiffness dissipation and penetration depth are not used in the PGS
    // solver and are thus set to NAN.
    stiffness_.resize(1);
    stiffness_(0) = NAN;
    dissipation_.resize(1);
    dissipation_(0) = NAN;
    penetration_depth_.resize(1);
    penetration_depth_(0) = NAN;
    mu_ = friction_coeff * VectorXd::Ones(1);
    point_data_ = std::make_unique<PointContactData<double>>(
        &penetration_depth_, Jc_.get(), &stiffness_, &dissipation_, &mu_);
  }

  void SetSystemDynamicsData() {
    v_star_ = -1.0 * VectorXd::Ones(3);
    Ainv_tmp_.resize(3, 3);
    {
      std::vector<Triplet> triplets;
      // Arbitrary entries for the tangent matrix.
      triplets.emplace_back(0, 0, 12.0);
      triplets.emplace_back(1, 1, 4.0);
      triplets.emplace_back(2, 2, 3.0);
      Ainv_tmp_.setFromTriplets(triplets.begin(), triplets.end());
      Ainv_tmp_.makeCompressed();
    }
    Ainv_ = std::make_unique<SparseLinearOperator<double>>("Ainv", &Ainv_tmp_);
    dynamics_data_ =
        std::make_unique<SystemDynamicsData<double>>(Ainv_.get(), &v_star_);
  }

  // Check the solver status to verify the solver has converged as expected.
  void VerifySolverStats() const {
    const PgsSolverStats& solver_stats = pgs_.get_solver_stats();
    EXPECT_TRUE(solver_stats.iterations < kMaxIterations);
    // ‖vc‖∞ <= 1, so the velocity error is smaller than
    // abs_tolerance + 1 * rel_tolerance = 2 * kTol.
    EXPECT_LT(solver_stats.vc_err, 2 * kTol);
    // The ‖W‖ᵣₘₛ > 1 and ‖gamma‖∞ <= 1/3, so the impulse error is smaller than
    // abs_tolerance / ‖W‖ᵣₘₛ + ‖gamma‖∞ * rel_tolerance < 2 * kTol.
    EXPECT_LT(solver_stats.gamma_err, 2 * kTol);
  }

  VectorXd v_star_;
  PgsSolver<double> pgs_;
  std::unique_ptr<SparseLinearOperator<double>> Ainv_;
  std::unique_ptr<SystemDynamicsData<double>> dynamics_data_;
  std::unique_ptr<PointContactData<double>> point_data_;
  VectorXd penetration_depth_;
  std::unique_ptr<SparseLinearOperator<double>> Jc_;
  SparseMatrixd jacobian_;
  SparseMatrixd Ainv_tmp_;
  VectorXd stiffness_;
  VectorXd dissipation_;
  VectorXd mu_;
};

/* The friction coefficient is set to be large enough to prevent any slipping.
 Hence, the expected post-solve contact velocity is (0, 0, 0). With the
 arbitrarily prescribed SPD tangent matrix A, the contact impulse can also be
 analytically calculated. The test confirms that the results from the PGS solver
 matches the analytically calculated contact velocity and impulse for a contact
 in stiction. */
TEST_F(PgsTest, Stiction) {
  // Set up a large friction coefficient to ensure stiction.
  SetSinglePointContactData(1000);
  // Abitrary initial guess.
  Vector3<double> v_guess(0.12, 0.34, 0.56);
  // dt is unused in PGS and is thus set to NAN.
  const double dt = NAN;
  ContactSolverResults<double> result;
  EXPECT_EQ(
      pgs_.SolveWithGuess(dt, *dynamics_data_, *point_data_, v_guess, &result),
      ContactSolverStatus::kSuccess);
  EXPECT_TRUE(
      CompareMatrices(result.ft, Vector2<double>(1. / 12., 1. / 4.), kTol));
  EXPECT_TRUE(CompareMatrices(result.fn, Vector1<double>(1. / 3.), kTol));
  EXPECT_TRUE(CompareMatrices(result.v_next, Vector3<double>(0, 0, 0), kTol));
  EXPECT_TRUE(CompareMatrices(
      result.tau_contact, Vector3<double>(1. / 12., 1. / 4., 1. / 3.), kTol));
  VerifySolverStats();
}

/* The friction coefficient is set to 0 so that the tangential velocity doesn't
 change in during contact. Hence, the expected post-solve contact velocity is
 (-1.0, -1.0, 0). With the arbitrarily prescribed SPD tangent matrix A, the
 contact impulse can also be analytically calculated. The test confirms that the
 results from the PGS solver matches the analytically calculated contact
 velocity and impulse for a sliding contact. */
TEST_F(PgsTest, Sliding) {
  SetSinglePointContactData(0);
  // Abitrary initial guess.
  Vector3<double> v_guess(0.12, 0.34, 0.56);
  // dt is unused in PGS and is thus set to NAN.
  const double dt = NAN;
  ContactSolverResults<double> result;
  EXPECT_EQ(
      pgs_.SolveWithGuess(dt, *dynamics_data_, *point_data_, v_guess, &result),
      ContactSolverStatus::kSuccess);
  EXPECT_TRUE(CompareMatrices(result.ft, Vector2<double>(0, 0), kTol));
  EXPECT_TRUE(CompareMatrices(result.fn, Vector1<double>(1. / 3.), kTol));
  EXPECT_TRUE(
      CompareMatrices(result.v_next, Vector3<double>(-1.0, -1.0, 0), kTol));
  EXPECT_TRUE(CompareMatrices(result.tau_contact,
                              Vector3<double>(0, 0, 1. / 3.), kTol));
  VerifySolverStats();
}

/* The solver exits early if there the number of contact is zero. Tests that the
 contact solver result in this case is as expected. */
TEST_F(PgsTest, NoContact) {
  // Solve a non trivial problem first.
  const Vector3<double> v_guess(0.12, 0.34, 0.56);
  SetSinglePointContactData(0);
  const double dt = NAN;
  ContactSolverResults<double> result;
  pgs_.SolveWithGuess(dt, *dynamics_data_, *point_data_, v_guess, &result);

  // Now solve a problem with no contact.
  SetZeroPointContactData();
  EXPECT_EQ(
      pgs_.SolveWithGuess(dt, *dynamics_data_, *point_data_, v_guess, &result),
      ContactSolverStatus::kSuccess);
  EXPECT_EQ(result.ft.size(), 0);
  EXPECT_EQ(result.fn.size(), 0);
  EXPECT_TRUE(
      CompareMatrices(result.v_next, Vector3<double>(-1.0, -1.0, -1.0), kTol));
  EXPECT_TRUE(
      CompareMatrices(result.tau_contact, Vector3<double>::Zero(), kTol));
  VerifySolverStats();
}
}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
