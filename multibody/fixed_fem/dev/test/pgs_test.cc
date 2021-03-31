#include <memory>

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/sparse_linear_operator.h"
#include "drake/multibody/fixed_fem/dev/pgs_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using SparseMatrixd = Eigen::SparseMatrix<double>;
using Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;
const double kTol = 1e-14;

/* A test against analytic solution for a simple contact problem:
 A single point in 3D is in contact with velocity (-1, -1, -1) in the contact
 space without contact force, where the first two components are in arbtrary
 orthogonal directions in the contact plane and the last component is the in the
 normal direction to the contact plane. The friction coefficient is set to be
 large enough to prevent any slipping. Hence, the expected post-solve contact
 velocity is (0, 0, 0). With an arbitrary prescribed tangent matrix A, the
 contact impulse can also be analytically calculated. The test confirms that the
 results from the PGS solver matches the analytically calculated contact
 velocity and impulse. */
class PgsTest : public ::testing::Test {
 protected:
  void SetUp() override {
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
    penetration_depth_.resize(1);
    penetration_depth_(0) = -1;
    v_free_ = -1.0 * VectorXd::Ones(3);

    Ainv_tmp_.resize(3, 3);
    {
      std::vector<Triplet> triplets;
      triplets.emplace_back(0, 0, 12.0);
      triplets.emplace_back(1, 1, 4.0);
      triplets.emplace_back(2, 2, 3.0);
      Ainv_tmp_.setFromTriplets(triplets.begin(), triplets.end());
      Ainv_tmp_.makeCompressed();
    }
    Ainv_ = std::make_unique<SparseLinearOperator<double>>("Ainv", &Ainv_tmp_);
    dynamics_data_ =
        std::make_unique<SystemDynamicsData<double>>(Ainv_.get(), &v_free_);
    stiffness_ = VectorXd::Zero(1);
    dissipation_ = VectorXd::Zero(1);
    mu_ = 1000.0 * VectorXd::Ones(1);
    point_data_ = std::make_unique<PointContactData<double>>(
        &penetration_depth_, Jc_.get(), &stiffness_, &dissipation_, &mu_);

    PgsSolverParameters params;
    params.abs_tolerance = kTol;
    params.rel_tolerance = kTol;
    params.max_iterations = 50;
    pgs_.set_parameters(params);
  }

  VectorXd v_free_;
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

TEST_F(PgsTest, Solve) {
  // Abitrary initial guess.
  Vector3<double> v_guess(0.12, 0.34, 0.56);
  const double dt = 0.1;
  ContactSolverResults<double> result;
  EXPECT_EQ(
      pgs_.SolveWithGuess(dt, *dynamics_data_, *point_data_, v_guess, &result),
      ContactSolverStatus::kSuccess);
  EXPECT_TRUE(
      CompareMatrices(result.ft, Vector2<double>(1. / 12., 1. / 4.), kTol));
  EXPECT_TRUE(CompareMatrices(result.fn, Vector1<double>(1. / 3.), kTol));
  EXPECT_TRUE(CompareMatrices(result.v_next, Vector3<double>(0, 0, 0), kTol));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
