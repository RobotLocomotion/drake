#include "drake/multibody/contact_solvers/sap_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/block_sparse_linear_operator.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/system_dynamics_data.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

class SapSolverTester {
 public:
  using PreProcessedData = SapSolver<double>::PreProcessedData;

  static void PackContactResults(const PreProcessedData& data,
                                 const VectorXd& v, const VectorXd& vc,
                                 const VectorXd& gamma,
                                 ContactSolverResults<double>* result) {
    SapSolver<double>::PackContactResults(data, v, vc, gamma, result);
  }

  static PreProcessedData PreProcessData(
      const SapSolver<double>& solver, double time_step,
      const SystemDynamicsData<double>& dynamics_data,
      const PointContactData<double>& contact_data) {
    return solver.PreProcessData(time_step, dynamics_data, contact_data);
  }
};

GTEST_TEST(SapSolver, PackContactResults) {
  // Setup minimum problem data.
  const int nv = 6;
  const int nc = 3;
  SapSolverTester::PreProcessedData data(1.0e-3, nv, nc);
  MatrixXd Jdense = MatrixXd::Ones(3 * nc, nv);
  BlockSparseMatrixBuilder<double> builder(1, 1, 1);
  builder.PushBlock(0, 0, Jdense);
  data.J = builder.Build();

  // Arbitrary solution vectors.
  const VectorXd v = VectorXd::LinSpaced(nv, 0, nv - 1);
  const VectorXd vc = VectorXd::LinSpaced(3 * nc, 0, 3 * nc - 1);
  const VectorXd gamma = -VectorXd::LinSpaced(3 * nc, 0, 3 * nc - 1);
  ContactSolverResults<double> results;
  SapSolverTester::PackContactResults(data, v, vc, gamma, &results);

  // Verify results were properly packed.
  EXPECT_EQ(results.v_next, v);
  const VectorXd fc_expected = gamma / data.time_step;
  VectorXd results_fc(3 * nc);
  MergeNormalAndTangent(results.fn, results.ft, &results_fc);
  EXPECT_EQ(results_fc, fc_expected);
  VectorXd results_vc(3 * nc);
  MergeNormalAndTangent(results.vn, results.vt, &results_vc);
  EXPECT_EQ(results_vc, vc);
  const VectorXd tauc_expected = Jdense.transpose() * gamma / data.time_step;
  EXPECT_TRUE(CompareMatrices(results.tau_contact, tauc_expected,
                              std::numeric_limits<double>::epsilon()));
}

// We place data consumed by the contact solver in a single struct.
struct ProblemData {
  ProblemData(int nv, int nc) {
    M.resize(nv, nv);
    J.resize(3 * nc, nv);
    v_star.resize(nv);
    phi0.resize(nc);
    stiffness.resize(nc);
    dissipation.resize(nc);
    mu.resize(nc);
  }

  double time_step;

  // For this problem we have that A = M, i.e. the momentum matrix equals the
  // mass matrix. Mass matrix in different formats:
  MatrixXd M;
  BlockSparseMatrix<double> Ablock;
  std::unique_ptr<BlockSparseLinearOperator<double>> Mop;

  // Jacobian in different formats:
  MatrixXd J;
  BlockSparseMatrix<double> Jblock;
  std::unique_ptr<BlockSparseLinearOperator<double>> Jop;

  // Free motion velocities.
  VectorXd v_star;

  // Contact data.
  VectorXd phi0;
  VectorXd stiffness;
  VectorXd dissipation;
  VectorXd mu;

  // Contact solver data.
  std::unique_ptr<SystemDynamicsData<double>> dynamics_data;
  std::unique_ptr<PointContactData<double>> contact_data;
};

// Simple problem configuration where 3 particles (with three translational DOFs
// each) are stacked on top of each other; particle 2 is on the ground, particle
// 1 is placed on top of particle 2 and particle 0 is placed on top of particle
// 1. The problem is setup with arbitrary (though non-zero) values in order to
// verify the computation of intermediate quantities such as the pre-processed
// data in SAP.
class ParticlesStackProblem {
 public:
  static constexpr int num_particles = 3;   // Number of particles.
  static constexpr int num_velocities = 9;  // Three particles in 3D.
  static constexpr int num_contacts = 3;    // Vertical stack.

  // Arbitrary non-zero free-motion velocities.
  VectorXd MakeFreeMotionVelocities() const {
    return VectorXd::LinSpaced(num_velocities, 0, num_velocities);
  }

  // Arbitrary particle masses.
  VectorXd GetParticleMasses() const {
    return 1.5 * VectorXd::LinSpaced(num_particles, 1, num_particles);
  }

  // Mass matrix for the system of three particles.
  void CalcMassMatrix(BlockSparseMatrix<double>* M) const {
    const VectorXd masses = GetParticleMasses();
    BlockSparseMatrixBuilder<double> builder(3, 3, 3);
    // Each particle is a block.
    for (int p = 0; p < num_particles; ++p) {
      builder.PushBlock(p, p, masses[p] * Matrix3d::Identity());
    }
    *M = builder.Build();
  }

  // Contact Jacobian representing the configuration in which the particles form
  // a stack. Particle 2 on the ground, particle 1 on 2 and particle 0 on 1.
  void MakeContactJacobian(BlockSparseMatrix<double>* J) const {
    const MatrixXd Jblock = MatrixXd::Identity(3, 3);
    BlockSparseMatrixBuilder<double> builder(num_contacts, num_particles, 5);
    // Contact between particles 0 and 1.
    builder.PushBlock(0, 0, Jblock);
    builder.PushBlock(0, 1, Jblock);
    // Contact between particles 1 and 2.
    builder.PushBlock(1, 1, Jblock);
    builder.PushBlock(1, 2, Jblock);
    // Contact between particle 2 and the ground.
    builder.PushBlock(2, 2, Jblock);
    *J = builder.Build();
  }

  std::unique_ptr<ProblemData> MakeProblemData() const {
    // Set system dynamics data:
    auto data = std::make_unique<ProblemData>(num_velocities, num_contacts);
    data->time_step = 5.0e-3;
    CalcMassMatrix(&data->Ablock);
    data->M = data->Ablock.MakeDenseMatrix();
    data->Mop =
        std::make_unique<BlockSparseLinearOperator<double>>("M", &data->Ablock);

    data->v_star = MakeFreeMotionVelocities();

    data->dynamics_data = std::make_unique<SystemDynamicsData<double>>(
        data->Mop.get(), nullptr, &data->v_star);

    MakeContactJacobian(&data->Jblock);
    data->J = data->Jblock.MakeDenseMatrix();
    data->Jop =
        std::make_unique<BlockSparseLinearOperator<double>>("J", &data->Jblock);

    data->phi0.setLinSpaced(num_contacts, 1., num_contacts);
    data->stiffness.setLinSpaced(num_contacts, 1., num_contacts);
    data->dissipation.setLinSpaced(num_contacts, 1., num_contacts);
    data->mu.setLinSpaced(num_contacts, 1., num_contacts);

    data->contact_data = std::make_unique<PointContactData<double>>(
        &data->phi0, data->Jop.get(), &data->stiffness, &data->dissipation,
        &data->mu);

    return data;
  }
};

GTEST_TEST(SapSolver, PreProcessedData) {
  ParticlesStackProblem problem;
  auto problem_data = problem.MakeProblemData();
  SapSolver<double> sap;
  SapSolverParameters parameters;
  sap.set_parameters(parameters);
  SapSolverTester::PreProcessedData data = SapSolverTester::PreProcessData(
      sap, problem_data->time_step, *problem_data->dynamics_data,
      *problem_data->contact_data);

  EXPECT_EQ(data.time_step, problem_data->time_step);
  EXPECT_EQ(data.nv, problem.num_velocities);
  EXPECT_EQ(data.nc, problem.num_contacts);
  EXPECT_EQ(data.mu, problem_data->mu);
  EXPECT_EQ(data.J.MakeDenseMatrix(), problem_data->J);
  EXPECT_EQ(data.A.MakeDenseMatrix(), problem_data->M);
  EXPECT_EQ(data.At[0], (problem_data->M.block<3, 3>(0, 0)));
  EXPECT_EQ(data.At[1], (problem_data->M.block<3, 3>(3, 3)));
  EXPECT_EQ(data.At[2], (problem_data->M.block<3, 3>(6, 6)));
  const VectorXd expected_inv_sqrt_A =
      problem_data->M.diagonal().cwiseInverse().cwiseSqrt();
  EXPECT_TRUE(CompareMatrices(data.inv_sqrt_A, expected_inv_sqrt_A,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
  EXPECT_EQ(data.v_star, problem_data->v_star);  // This should just be a copy.
  EXPECT_TRUE(CompareMatrices(
      data.p_star, problem_data->M * problem_data->v_star,
      std::numeric_limits<double>::epsilon(), MatrixCompareType::relative));

  // For this simple configuration, we can verify analytically that the
  // effective (inverse of the) mass of each contact point is: sqrt(3)/3*(1/m1 +
  // 1/m2), for the contact between particles masses m1 and m2. For the ground,
  // we take the mass to be infinity.
  const VectorXd masses = problem.GetParticleMasses();
  const double Wdiag0 = std::sqrt(3) / 3 * (1.0 / masses(0) + 1.0 / masses(1));
  const double Wdiag1 = std::sqrt(3) / 3 * (1.0 / masses(1) + 1.0 / masses(2));
  const double Wdiag2 = std::sqrt(3) / 3 * (1.0 / masses(2));
  const Vector3d Wdiag_expected(Wdiag0, Wdiag1, Wdiag2);
  EXPECT_TRUE(CompareMatrices(data.delassus_diagonal, Wdiag_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // Verify regularization parameters.
  const VectorXd& k = problem_data->stiffness.array();
  const VectorXd& c = problem_data->dissipation.array();
  const double dt = problem_data->time_step;
  const VectorXd taud = c.array() / k.array();
  const VectorXd Rn_expected =
      ((dt + taud.array()) * k.array() * dt).cwiseInverse();
  const VectorXd Rt_expected = parameters.sigma * Wdiag_expected;
  VectorXd R_expected(3 * problem.num_contacts);
  for (int i = 0; i < problem.num_contacts; ++i) {
    R_expected.segment<3>(3 * i) =
        Vector3d(Rt_expected(i), Rt_expected(i), Rn_expected(i));
  }
  EXPECT_TRUE(CompareMatrices(data.R, R_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
