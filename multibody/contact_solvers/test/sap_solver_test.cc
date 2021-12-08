#include "drake/multibody/contact_solvers/sap_solver.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/block_sparse_linear_operator.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/system_dynamics_data.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
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

// TODO: re-enable this test when ContactResults allows more general data.
#if 0
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
#endif

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
  EXPECT_EQ(data.constraints_bundle->J().MakeDenseMatrix(), problem_data->J);
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
  const VectorXd& k = problem_data->stiffness;
  const VectorXd& c = problem_data->dissipation;
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

/* Model of a "pizza saver":

  ^ y               C
  |                 ◯
  |                /\
  ----> x         /  \
               b /    \ a
                /      \
               /        \
              ◯----------◯
              A     c    B

It is modeled as an equilateral triangle with a contact point at each of the
legs. The total mass of the pizza saver is m and its rotational inertia about
the triangle's barycenter is I.
If h is the height of the triangle from any of its sides, the distance from
any point to the triangle's center is 2h/3. The height h relates to the length
a of a side by h = 3/2/sqrt(3) a.
The generalized positions vector for this case is q = [x, y, theta], with
theta = 0 for the triangle in the configuration shown in the schematic. */
class PizzaSaverProblem {
 public:
  static constexpr int num_velocities = 4;
  static constexpr int num_contacts = 3;

  // @param dt Discrete time step.
  // @param mass Total mass of the system, in Kg.
  // @param radius Radius of the circle circumscribing the triangular pizza
  // saver, in m.
  // @param mu Coefficient of dynamic friction.
  // @param mu Stiffness with the ground, in N/m.
  // @param taud Dissipation time scale, in seconds.
  PizzaSaverProblem(double dt, double mass, double radius, double mu, double k,
                    double taud)
      : time_step_(dt),
        m_(mass),
        R_(radius),
        mu_(mu),
        stiffness_(k),
        taud_(taud) {
    // The radius of the circumscribed circle R is the distance from each
    // contact point to the triangle's center.
    // We model the pizza saver as three point masses m/3 at each contact
    // point and thus the moment of inertia is I = 3 * (m/3 R²) = m R²:
    I_ = m_ * R_ * R_;
  }

  // Pizza saver model with default mass m = 1 Kg and radius = 1.0 m.
  PizzaSaverProblem(double dt, double mu, double k, double taud)
      : PizzaSaverProblem(dt, 1.0, 1.0, mu, k, taud) {}

  double time_step() const { return time_step_; }

  // Mass of each point mass, Kg. Total mass is three times mass().
  double mass() const { return m_; }

  double radius() const { return R_; }

  double rotational_inertia() const { return I_; }

  // Acceleration of gravity, m/s².
  double g() const { return g_; }

  void CalcMassMatrix(MatrixXd* M) const {
    M->resize(num_velocities, num_velocities);
    // clang-format off
    *M << m_,  0,  0,  0,
           0, m_,  0,  0,
           0,  0, m_,  0,
           0,  0,  0, I_;
    // clang-format on
  }

  void CalcContactJacobian(double theta, MatrixXd* Jc) const {
    Jc->resize(3 * num_contacts, num_velocities);

    const double c = cos(theta);
    const double s = sin(theta);

    // 3D rotation matrix of the body frame B in the world frame W.
    // clang-format off
    Matrix3d R_WB;
    R_WB << c, s, 0,
           -s, c, 0,
            0, 0, 1;
    // clang-format on

    // Position of each contact point in the body frame B.
    const Vector3d p_BoA(-sqrt(3) / 2.0, -0.5, 0.0);
    const Vector3d p_BoB(sqrt(3) / 2.0, -0.5, 0.0);
    const Vector3d p_BoC(0.0, 1.0, 0.0);

    // Position of each contact point in the world frame W.
    const Vector3d p_BoA_W = R_WB * p_BoA;
    const Vector3d p_BoB_W = R_WB * p_BoB;
    const Vector3d p_BoC_W = R_WB * p_BoC;

    // clang-format off
    // Point A
    Jc->block(0, 0, 3, num_velocities) << 1, 0, 0, -p_BoA_W.y(),
                                          0, 1, 0,  p_BoA_W.x(),
                                          0, 0, 1, 0;

    // Point B
    Jc->block(3, 0, 3, num_velocities) << 1, 0, 0, -p_BoB_W.y(),
                                          0, 1, 0, p_BoB_W.x(),
                                          0, 0, 1, 0;

    // Point C
    Jc->block(6, 0, 3, num_velocities) << 1, 0, 0, -p_BoC_W.y(),
                                          0, 1, 0, p_BoC_W.x(),
                                          0, 0, 1, 0;
    // clang-format on
  }

  // Makes the problem data to advance the dynamics of the pizza saver from
  // state x0 = [q0, v0], with applied forces tau = (fx, fy, fz, Mz).
  std::unique_ptr<ProblemData> MakeProblemData(const VectorXd& q0,
                                               const VectorXd& v0,
                                               const VectorXd& tau) const {
    // Set system dynamics data:
    auto data = std::make_unique<ProblemData>(num_velocities, num_contacts);
    CalcMassMatrix(&data->M);
    {
      // A single block size.
      BlockSparseMatrixBuilder<double> builder(1, 1, 1);
      builder.PushBlock(0, 0, data->M);
      data->Ablock = builder.Build();
    }
    data->Mop =
        std::make_unique<BlockSparseLinearOperator<double>>("M", &data->Ablock);

    data->v_star.resize(num_velocities);
    data->v_star = v0 + time_step_ * data->M.ldlt().solve(tau);

    data->dynamics_data = std::make_unique<SystemDynamicsData<double>>(
        data->Mop.get(), nullptr, &data->v_star);

    // Set contact data:
    // Some arbitrary orientation. This particular case has symmetry of
    // revolution (meaning the result is independent of angle theta).
    // const double theta = M_PI / 5;
    CalcContactJacobian(q0(3), &data->J);
    {
      // A single block size.
      BlockSparseMatrixBuilder<double> builder(1, 1, 1);
      builder.PushBlock(0, 0, data->J);
      data->Jblock = builder.Build();
    }
    data->Jop =
        std::make_unique<BlockSparseLinearOperator<double>>("J", &data->Jblock);

    data->phi0.setConstant(num_contacts, q0(2));
    data->stiffness.setConstant(num_contacts, stiffness_);
    data->dissipation.setConstant(num_contacts, taud_ * stiffness_);
    data->mu.setConstant(num_contacts, mu_);

    data->contact_data = std::make_unique<PointContactData<double>>(
        &data->phi0, data->Jop.get(), &data->stiffness, &data->dissipation,
        &data->mu);

    return data;
  }

 private:
  // Helper method for NaN initialization.
  static constexpr double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // The physical parameters of the model. They are initialized with NaN for a
  // quick detection of uninitialized values.
  double time_step_{nan()};  // Discrete time step.

  // Pizza saver parameters.
  double m_{nan()};  // Mass of the pizza saver.
  double R_{nan()};  // Distance from COM to any contact point.
  double I_{nan()};  // Rotational inertia about the z axis.

  // Contact parameters:
  double mu_{nan()};         // Friction coefficient.
  double stiffness_{nan()};  // Contact stiffness k.
  double taud_{nan()};       // Linear dissipation time scale: c = taud * k.

  const double g_{10.0};  // Acceleration of gravity.
};

ContactSolverResults<double> AdvanceNumSteps(
    const PizzaSaverProblem& problem, const VectorXd& tau, int num_steps,
    const SapSolverParameters& params) {
  SapSolver<double> sap;
  sap.set_parameters(params);
  ContactSolverResults<double> result;
  // Arbitrary non-zero guess to stress the solver.
  VectorXd v_guess(problem.num_velocities);
  v_guess << 1.0, 2.0, 3.0, 4.0;

  const double theta = M_PI / 5;  // Arbitrary orientation.
  VectorXd q = Vector4d(0.0, 0.0, 0.0, theta);
  VectorXd v = VectorXd::Zero(problem.num_velocities);

  for (int i = 0; i < num_steps; ++i) {
    const auto data = problem.MakeProblemData(q, v, tau);
    const ContactSolverStatus status =
        sap.SolveWithGuess(problem.time_step(), *data->dynamics_data,
                           *data->contact_data, v_guess, &result);
    EXPECT_EQ(status, ContactSolverStatus::kSuccess);
    v = result.v_next;
    q += problem.time_step() * v;

    // Verify the number of times cache entries were updated.
    const SapSolver<double>::SolverStats& stats = sap.get_statistics();

    // N.B. We only count iterations that perform factorizations. Since impulses
    // need to be computed in order to evaluate the termination criteria (before
    // a factorization might be carried out), the expected number of gradients
    // update equals the number of iterations plus one.
    EXPECT_EQ(stats.num_gradients_cache_updates, stats.num_iters + 1);

    // Impulses are evaluated:
    //  - At the very beggining of an iteration, to evaluate stopping criteria
    //    (num_iters+1 since the last iteration does not perform factorization.)
    //  - At the very beggining of line search iterations, i.e. num_iters.
    //  - Once per line search iteration.
    //    Therefore we expect impulses to be evaluated
    //    num_line_search_iters+2*num_iters+1 times.
    EXPECT_EQ(stats.num_impulses_cache_updates,
              stats.num_line_search_iters + 2 * stats.num_iters + 1);
  }

  return result;
}

// Solve a problem with no applied torque. In this case contact forces should
// balance weight.
GTEST_TEST(PizzaSaver, NoAppliedTorque) {
  const double dt = 0.01;
  const double mu = 1.0;
  const double k = 1.0e4;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.beta = 0;  // No near-rigid regime.
  params.ls_max_iterations = 40;

  const Vector4d tau(0.0, 0.0, -problem.mass() * problem.g(), 0.0);
  const ContactSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 30, params);

  // N.B. The accuracy of the solutions is significantly higher when using exact
  // line search.
  // TODO(amcastro-tri): Tighten tolerances for runs with exact line search.

  // Expected generalized force.
  Vector4d tau_expected(0.0, 0.0, problem.mass() * problem.g(), 0.0);

  EXPECT_TRUE(CompareMatrices(result.v_next,
                              VectorXd::Zero(problem.num_velocities),
                              params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(result.tau_contact, tau_expected,
                              params.rel_tolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.ft,
                              VectorXd::Zero(2 * problem.num_contacts),
                              params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(
      result.fn,
      VectorXd::Constant(problem.num_contacts, tau_expected(2) / 3.0),
      params.rel_tolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.vt,
                              VectorXd::Zero(2 * problem.num_contacts),
                              params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(result.vn, VectorXd::Zero(problem.num_contacts),
                              params.rel_tolerance));
}

// This tests the solver when we apply a moment Mz about COM to the pizza
// saver. If Mz < mu * m * g * R, the saver should be in stiction (that is,
// the sliding velocity should be smaller than the regularization parameter).
// Otherwise the saver will start sliding. For this setup the transition
// occurs at M_transition = mu * m * g * R = 5.0
GTEST_TEST(PizzaSaver, Stiction) {
  const double dt = 0.01;
  const double mu = 1.0;
  const double k = 1.0e4;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.beta = 0;  // No near-rigid regime.
  params.ls_max_iterations = 40;

  const double Mz = 3.0;
  const Vector4d tau(0.0, 0.0, -problem.mass() * problem.g(), Mz);
  const ContactSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 30, params);

  // Expected generalized force.
  const Vector4d tau_expected(0.0, 0.0, problem.mass() * problem.g(), -Mz);

  // Maximum expected slip. See Castro et al. 2021 for details.
  const double max_slip_expected = params.sigma * dt * problem.g();

  EXPECT_TRUE(CompareMatrices(result.v_next.head<3>(), Vector3d::Zero(),
                              params.abs_tolerance));
  EXPECT_TRUE(CompareMatrices(result.tau_contact, tau_expected,
                              params.rel_tolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      result.fn,
      VectorXd::Constant(problem.num_contacts, tau_expected(2) / 3.0),
      params.rel_tolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.vn, VectorXd::Zero(problem.num_contacts),
                              params.abs_tolerance));
  const double friction_force_expected = Mz / problem.radius() / 3.0;
  for (int i = 0; i < 3; ++i) {
    const double slip = result.vt.segment<2>(2 * i).norm();
    const double friction_force = result.ft.segment<2>(2 * i).norm();
    const double normal_force = result.fn(i);
    EXPECT_LE(friction_force, mu * normal_force);
    EXPECT_NEAR(friction_force, friction_force_expected, params.rel_tolerance);
    EXPECT_LE(slip, max_slip_expected);
  }
}

// Test case with zero friction coefficient.
GTEST_TEST(PizzaSaver, NoFriction) {
  const double dt = 0.01;
  const double mu = 0.0;
  const double k = 1.0e4;
  const double taud = dt;
  const int num_steps = 30;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.beta = 0;  // No near-rigid regime.
  params.ls_max_iterations = 40;

  const double fx = 1.0;
  const Vector4d tau(fx, 0.0, -problem.mass() * problem.g(), 0.0);
  const ContactSolverResults<double> result =
      AdvanceNumSteps(problem, tau, num_steps, params);

  // Expected generalized force.
  const Vector4d tau_expected(0.0, 0.0, problem.mass() * problem.g(), 0.0);

  const double vx_expected = num_steps * dt * fx / problem.mass();

  EXPECT_TRUE(CompareMatrices(result.v_next,
                              Vector4d(vx_expected, 0.0, 0.0, 0.0),
                              2.0 * params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(result.tau_contact, tau_expected,
                              2.0 * params.rel_tolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.ft,
                              VectorXd::Zero(2 * problem.num_contacts),
                              std::numeric_limits<double>::epsilon()));
  EXPECT_TRUE(CompareMatrices(
      result.fn,
      VectorXd::Constant(problem.num_contacts, tau_expected(2) / 3.0),
      2.0 * params.rel_tolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.vn, VectorXd::Zero(problem.num_contacts),
                              params.abs_tolerance));
  for (int i = 0; i < 3; ++i) {
    const auto vt = result.vt.segment<2>(2 * i);
    EXPECT_TRUE(CompareMatrices(vt, Vector2d(vx_expected, 0.0),
                                2.0 * params.rel_tolerance,
                                MatrixCompareType::relative));
  }
}

// This tests the solver when we apply a moment Mz about COM to the pizza
// saver. If Mz > mu * m * g * R, the saver will slide.
// occurs at M_transition = mu * m * g * R = 5.0
GTEST_TEST(PizzaSaver, Sliding) {
  const double dt = 0.01;
  const double mu = 0.5;
  const double k = 1.0e4;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.beta = 0;  // No near-rigid regime.
  params.ls_max_iterations = 40;

  const double Mz = 6.0;
  const double weight = problem.mass() * problem.g();
  const double fn_expected = weight / 3.0;
  const double ft_expected = mu * fn_expected;
  const double friction_torque_expected = 3.0 * ft_expected * problem.radius();

  const Vector4d tau(0.0, 0.0, -weight, Mz);

  const ContactSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 10, params);

  // Expected generalized force.
  const Vector4d tau_expected(0.0, 0.0, problem.mass() * problem.g(),
                              -friction_torque_expected);

  EXPECT_GT(result.v_next(3), 0.0);  // It's accelerating.
  EXPECT_NEAR(tau_expected.head<2>().norm(), 0.0, params.abs_tolerance);
  EXPECT_GT(std::abs(tau_expected(2)), 0.0);
  EXPECT_LT(std::abs(tau_expected(3)), Mz);
  for (int i = 0; i < 3; ++i) {
    const double friction_force = result.ft.segment<2>(2 * i).norm();
    const double normal_force = result.fn(i);
    EXPECT_NEAR(friction_force, mu * normal_force,
                5.0 * std::numeric_limits<double>::epsilon());
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
