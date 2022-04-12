#include "drake/multibody/contact_solvers/sap/sap_solver.h"

// TODO(amcastro-tri): While many of tests in this file verify the correctness
// of the solution, it might not be enough to ensure the entire implementation
// is correct. E.g. miscalculation of the Hessian could lead to slower
// convergence. Consider more fine grained unit tests, especially for the
// Hessian and other gradients.

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/block_sparse_linear_operator.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
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

constexpr double kEps = std::numeric_limits<double>::epsilon();
// Suggested value for the dimensionless parameter used in the regularization of
// friction, see [Castro et al. 2021].
constexpr double kDefaultSigma = 1.0e-3;

/* Model of a "pizza saver":

  ^ y
  |                 ◯ C
  |                /\
  ----> x         /  \
               a /    \ a
                /      \
               /        \
            A ◯----------◯ B
                   a

It is modeled as an equilateral triangle with a contact point at each of the
vertices. The total mass of the pizza saver is m and its rotational inertia
about the triangle's barycenter is I. If h is the height of the triangle from
any of its sides, the distance from any point to the triangle's center is 2h/3.
The height h relates to the length a of a side by h = sqrt(3)/2 a. The
generalized positions vector for this case is q = [x, y, z, theta], with theta =
0 for the triangle in the configuration shown in the schematic. */
class PizzaSaverProblem {
 public:
  static constexpr int kNumVelocities = 4;
  static constexpr int kNumContacts = 3;

  // @param dt Discrete time step.
  // @param mass Total mass of the system, in Kg.
  // @param radius Radius of the circle circumscribing the triangular pizza
  // saver, in m.
  // @param mu Coefficient of dynamic friction.
  // @param k Stiffness with the ground, in N/m.
  // @param taud Dissipation time scale, in seconds.
  PizzaSaverProblem(double dt, double mass, double radius, double mu, double k,
                    double taud)
      : time_step_(dt),
        m_(mass),
        radius_(radius),
        mu_(mu),
        stiffness_(k),
        taud_(taud) {
    // The radius of the circumscribed circle R is the distance from each
    // contact point to the triangle's center.
    // We model the pizza saver as three point masses m/3 at each contact
    // point and thus the moment of inertia is I = 3 * (m/3 R²) = m R²:
    I_ = m_ * radius_ * radius_;
  }

  // Pizza saver model with default mass m = 3 Kg and radius = 1.5 m.
  PizzaSaverProblem(double dt, double mu, double k, double taud)
      : PizzaSaverProblem(dt, 3.0, 1.5, mu, k, taud) {}

  double time_step() const { return time_step_; }

  // Mass of each point mass, Kg. Total mass is three times mass().
  double mass() const { return m_; }

  double radius() const { return radius_; }

  double rotational_inertia() const { return I_; }

  double mu() const { return mu_; }

  // Acceleration of gravity, m/s².
  double g() const { return g_; }

  void CalcMassMatrix(MatrixXd* M) const {
    M->resize(kNumVelocities, kNumVelocities);
    // clang-format off
    *M << m_,  0,  0,  0,
           0, m_,  0,  0,
           0,  0, m_,  0,
           0,  0,  0, I_;
    // clang-format on
  }

  void CalcContactJacobian(double theta, MatrixXd* Jc) const {
    Jc->resize(3 * kNumContacts, kNumVelocities);

    const double c = std::cos(theta);
    const double s = std::sin(theta);

    // 3D rotation matrix of the body frame B in the world frame W.
    // clang-format off
    Matrix3d R_WB;
    R_WB << c, s, 0,
           -s, c, 0,
            0, 0, 1;
    // clang-format on

    // Position of each contact point in the body frame B.
    const Vector3d p_BoA = radius_ * Vector3d(-sqrt(3) / 2.0, -0.5, 0.0);
    const Vector3d p_BoB = radius_ * Vector3d(sqrt(3) / 2.0, -0.5, 0.0);
    const Vector3d p_BoC = radius_ * Vector3d(0.0, 1.0, 0.0);

    // Position of each contact point in the world frame W.
    const Vector3d p_BoA_W = R_WB * p_BoA;
    const Vector3d p_BoB_W = R_WB * p_BoB;
    const Vector3d p_BoC_W = R_WB * p_BoC;

    // clang-format off
    // Point A
    Jc->middleRows<3>(0) << 1, 0, 0, -p_BoA_W.y(),
                            0, 1, 0,  p_BoA_W.x(),
                            0, 0, 1, 0;

    // Point B
    Jc->middleRows<3>(3) << 1, 0, 0, -p_BoB_W.y(),
                            0, 1, 0, p_BoB_W.x(),
                            0, 0, 1, 0;

    // Point C
    Jc->middleRows<3>(6) << 1, 0, 0, -p_BoC_W.y(),
                            0, 1, 0, p_BoC_W.x(),
                            0, 0, 1, 0;
    // clang-format on
  }

  // Makes contact problem to advance the dynamics of the pizza saver from
  // state x0 = [q0, v0], with applied forces tau = (fx, fy, fz, Mz).
  // beta is the dimensionless near-rigid regime parameter and sigma the
  // dimensionless parameter for the regularization of friction, see [Castro et
  // al. 2021] for details.
  std::unique_ptr<SapContactProblem<double>> MakeContactProblem(
      const VectorXd& q0, const VectorXd& v0, const VectorXd& tau, double beta,
      double sigma) const {
    std::unique_ptr<SapContactProblem<double>> problem =
        MakeContactProblemWithoutConstraints(q0, v0, tau);

    // Add contact constraints.
    const double phi0 = q0(2);
    const SapFrictionConeConstraint<double>::Parameters parameters{
        mu_, stiffness_, taud_, beta, 1.0e-3};
    MatrixXd J;  // Full system Jacobian for the three contacts.
    CalcContactJacobian(q0(3), &J);
    problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
        0, J.middleRows(0, 3), phi0, parameters));
    problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
        0, J.middleRows(3, 3), phi0, parameters));
    problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
        0, J.middleRows(6, 3), phi0, parameters));

    return problem;
  }

  std::unique_ptr<SapContactProblem<double>>
  MakeContactProblemWithoutConstraints(const VectorXd& q0, const VectorXd& v0,
                                       const VectorXd& tau) const {
    // For this problem there is a single clique for the pizza saver.
    std::vector<MatrixXd> A(1);
    CalcMassMatrix(&A[0]);

    // Compute free motion velocities.
    VectorXd v_star = v0 + time_step_ * A[0].ldlt().solve(tau);

    auto problem = std::make_unique<SapContactProblem<double>>(
        time_step_, std::move(A), std::move(v_star));

    return problem;
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
  double m_{nan()};       // Mass of the pizza saver.
  double radius_{nan()};  // Distance from COM to any contact point.
  double I_{nan()};       // Rotational inertia about the z axis.

  // Contact parameters:
  double mu_{nan()};         // Friction coefficient.
  double stiffness_{nan()};  // Contact stiffness k.
  double taud_{nan()};       // Linear dissipation time scale: c = taud * k.

  const double g_{10.0};  // Acceleration of gravity.
};

SapSolverResults<double> AdvanceNumSteps(const PizzaSaverProblem& problem,
                                         const VectorXd& tau, int num_steps,
                                         const SapSolverParameters& params,
                                         double beta = 1.0,
                                         bool cost_criterion_reached = false) {
  SapSolver<double> sap;
  sap.set_parameters(params);
  SapSolverResults<double> result;
  // Arbitrary non-zero guess to stress the solver.
  // N.B. We need non-zero values when cost_criterion_reached = true since
  // v_guess = 0 would be close to the steady state solution and SAP would
  // satisfy the optimality even when very tight tolerances are specified.
  VectorXd v_guess(problem.kNumVelocities);
  v_guess << 1.0, 2.0, 3.0, 4.0;

  const double theta = M_PI / 5;  // Arbitrary orientation.
  VectorXd q = Vector4d(0.0, 0.0, 0.0, theta);
  VectorXd v = VectorXd::Zero(problem.kNumVelocities);

  for (int i = 0; i < num_steps; ++i) {
    const auto contact_problem =
        problem.MakeContactProblem(q, v, tau, beta, kDefaultSigma);
    const SapSolverStatus status =
        sap.SolveWithGuess(*contact_problem, v_guess, &result);
    EXPECT_EQ(status, SapSolverStatus::kSuccess);
    v = result.v;
    q += problem.time_step() * v;

    // Verify the number of times cache entries were updated.
    const SapSolver<double>::SolverStats& stats = sap.get_statistics();

    if (cost_criterion_reached) {
      EXPECT_TRUE(stats.cost_criterion_reached);
    } else {
      EXPECT_TRUE(stats.optimality_criterion_reached);
    }
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
  const PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.ls_max_iterations = 40;
  const double beta = kEps;  // No near-rigid regime.

  const Vector4d tau(0.0, 0.0, -problem.mass() * problem.g(), 0.0);
  const SapSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 30, params, beta);

  // N.B. The accuracy of the solutions is significantly higher when using exact
  // line search.
  // TODO(amcastro-tri): Tighten tolerances for runs with exact line search.

  // Expected generalized impulse.
  const Vector3d per_contact_impulse =
      dt * problem.mass() * problem.g() * Vector3d::UnitZ() / 3.0;
  const Vector4d j_expected(0.0, 0.0, dt * problem.mass() * problem.g(), 0.0);

  EXPECT_TRUE(CompareMatrices(result.v, VectorXd::Zero(problem.kNumVelocities),
                              params.rel_tolerance));
  EXPECT_TRUE((result.j - j_expected).norm() / j_expected.norm() <
              params.rel_tolerance);
  const VectorXd gamma_expected =
      per_contact_impulse.replicate(problem.kNumContacts, 1);
  EXPECT_TRUE(CompareMatrices(result.gamma, gamma_expected,
                              2.0 * params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(result.vc,
                              VectorXd::Zero(3 * problem.kNumContacts),
                              params.rel_tolerance));
}

// We verify SAP returns immediately when the exact guess is provided. To stress
// test this case, the maximum number of iterations is set to zero. We expect
// SAP to return the guess as the solution.
GTEST_TEST(PizzaSaver, ConvergenceWithExactGuess) {
  const double dt = 0.01;
  const double mu = 1.0;
  const double k = 1.0e4;
  const double taud = dt;
  const PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.ls_max_iterations = 40;
  params.max_iterations = 0;
  const double beta = kEps;  // No near-rigid regime.

  const double weight = problem.mass() * problem.g();
  const Vector4d tau(0.0, 0.0, -weight, 0.0);

  // We set the initial position so that the (three) compliant point contact
  // forces balance weight.
  const double z = -weight / k / 3.0;
  const double theta = M_PI / 5;  // Arbitrary orientation.
  VectorXd q = Vector4d(0.0, 0.0, z, theta);
  VectorXd v = VectorXd::Zero(problem.kNumVelocities);

  const auto contact_problem =
      problem.MakeContactProblem(q, v, tau, beta, kDefaultSigma);
  SapSolver<double> sap;
  sap.set_parameters(params);
  SapSolverResults<double> result;
  const SapSolverStatus status =
      sap.SolveWithGuess(*contact_problem, v, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);

  // Verify no iterations were performed.
  const SapSolver<double>::SolverStats& stats = sap.get_statistics();
  EXPECT_EQ(stats.num_iters, 0);

  // The guess was not even touched but directly copied into the results.
  EXPECT_EQ(result.v, v);
}

// Makes a problem for which we know the solution will be in stiction. If Mz <
// mu * m * g * R, the saver should be in stiction (that is, the sliding
// velocity should be smaller than the regularization parameter). Otherwise the
// saver will start sliding. For this setup the transition occurs at
// M_transition = mu * m * g * R = 30.
PizzaSaverProblem MakeStictionProblem() {
  const double dt = 0.01;
  const double mu = 2. / 3.;
  const double k = 1.0e4;
  const double taud = dt;
  return PizzaSaverProblem(dt, mu, k, taud);
}

// Make a stiction problem with MakeStictionProblem() and verify the solution is
// accurate to within `relative_tolerance`. `params` specifies SAP's parameters.
// `cost_criterion_reached` specifies if we expect the cost criterion to be
// reached for all time steps.
void VerifyStictionSolution(const SapSolverParameters& params,
                            double relative_tolerance,
                            bool cost_criterion_reached) {
  PizzaSaverProblem problem = MakeStictionProblem();

  const double Mz = 20.0;
  const Vector4d tau(0.0, 0.0, -problem.mass() * problem.g(), Mz);
  const double beta = 1.0;  // Enable near-rigid regime.
  const SapSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 40, params, beta, cost_criterion_reached);

  // Expected generalized impulse.
  const Vector4d j_expected =
      problem.time_step() *
      Vector4d(0.0, 0.0, problem.mass() * problem.g(), -Mz);

  // Maximum expected slip. See Castro et al. 2021 for details (Eq. 22).
  const double max_slip_expected =
      kDefaultSigma * problem.time_step() * problem.g();

  EXPECT_TRUE(CompareMatrices(result.v.head<3>(), Vector3d::Zero(), 1.0e-10));
  EXPECT_TRUE((result.j - j_expected).norm() / j_expected.norm() <
              relative_tolerance);
  VectorXd gamma_normal(problem.kNumContacts);
  ExtractNormal(result.gamma, &gamma_normal);
  EXPECT_TRUE(CompareMatrices(
      gamma_normal,
      VectorXd::Constant(problem.kNumContacts, j_expected(2) / 3.0),
      relative_tolerance, MatrixCompareType::relative));
  VectorXd vc_normal(problem.kNumContacts);
  ExtractNormal(result.vc, &vc_normal);
  EXPECT_TRUE(CompareMatrices(vc_normal, VectorXd::Zero(problem.kNumContacts),
                              1.0e-10));
  const double friction_impulse_expected =
      problem.time_step() * Mz / problem.radius() / 3.0;

  VectorXd gamma_friction(2 * problem.kNumContacts);
  ExtractTangent(result.gamma, &gamma_friction);
  VectorXd vc_tangent(2 * problem.kNumContacts);
  ExtractTangent(result.vc, &vc_tangent);
  for (int i = 0; i < 3; ++i) {
    const double slip = vc_tangent.segment<2>(2 * i).norm();
    const double friction_impulse = gamma_friction.segment<2>(2 * i).norm();
    const double normal_impulse = gamma_normal(i);
    EXPECT_LE(friction_impulse, problem.mu() * normal_impulse);
    EXPECT_NEAR(friction_impulse, friction_impulse_expected,
                relative_tolerance * friction_impulse_expected);
    EXPECT_LE(slip, max_slip_expected);
  }
}

// Solve a stiction problem and verify the solution for a nominal set of solver
// parameters.
GTEST_TEST(PizzaSaver, Stiction) {
  SapSolverParameters params;
  params.abs_tolerance = 1.0e-14;
  params.rel_tolerance = 1.0e-6;
  params.ls_max_iterations = 40;
  // For the tolerances specified, we expect the solver to reach the optimality
  // condition for all time steps.
  const bool cost_criterion_reached = false;
  // At these tolerances, the solution error scales with the optimality
  // condition specified tolerance.
  const double relative_tolerance = params.rel_tolerance;
  VerifyStictionSolution(params, relative_tolerance, cost_criterion_reached);
}

// We set a very tight optimality tolerance. The solver won't be able to reach
// these tolerances. However, it will reach the optimal solution within
// round-off errors. This is the best the solver could do. It makes sense that
// we return this as the valid solution. To detect this condition the solver
// monitors the decrease of the cost each iteration. If the decrease of the cost
// is below the relative tolerance specified by
// SapSolverParameters::cost_rel_tolerance, the solver stops the iteration and
// returns a success code with the last computed solution.
// In this test we verify the computed solution is accurate within a reasonable
// tolerance that takes into account the temporal transient.
GTEST_TEST(PizzaSaver, StictionAtTightOptimalityTolerance) {
  SapSolverParameters params;
  // Extremely tight tolerances to trigger the cost stopping criterion.
  params.abs_tolerance = 1.0e-20;
  params.rel_tolerance = 1.0e-20;
  params.ls_max_iterations = 40;
  // Inform the unit test we want to verify this condition.
  const bool cost_criterion_reached = true;
  // Use a reasonable tolerance to verify the accuracy of the solution.
  const double relative_tolerance = 1.0e-6;
  VerifyStictionSolution(params, relative_tolerance, cost_criterion_reached);
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
  params.ls_max_iterations = 40;
  const double beta = kEps;  // No near-rigid regime.

  const double fx = 1.0;
  const Vector4d tau(fx, 0.0, -problem.mass() * problem.g(), 0.0);
  const SapSolverResults<double> result =
      AdvanceNumSteps(problem, tau, num_steps, params, beta);

  // Expected generalized impulse.
  const Vector4d j_expected(0.0, 0.0, dt * problem.mass() * problem.g(), 0.0);

  const double vx_expected = num_steps * dt * fx / problem.mass();

  EXPECT_TRUE(CompareMatrices(result.v, Vector4d(vx_expected, 0.0, 0.0, 0.0),
                              2.0 * params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(result.j, j_expected, 2.0 * params.rel_tolerance,
                              MatrixCompareType::relative));

  const Vector3d per_contact_impulse =
      dt * problem.mass() * problem.g() * Vector3d::UnitZ() / 3.0;
  const VectorXd gamma_expected =
      per_contact_impulse.replicate(problem.kNumContacts, 1);
  EXPECT_TRUE(CompareMatrices(result.gamma, gamma_expected,
                              2.0 * params.rel_tolerance,
                              MatrixCompareType::relative));
  const Vector3d per_contact_velocity = Vector3d::UnitX() * vx_expected;
  const VectorXd vc_expected =
      per_contact_velocity.replicate(problem.kNumContacts, 1);
  EXPECT_TRUE(CompareMatrices(result.vc, vc_expected,
                              2.0 * params.rel_tolerance,
                              MatrixCompareType::relative));
}

// This tests the solver when we apply a moment Mz about COM to the pizza
// saver. If Mz > mu * m * g * R, the saver will slide.
// occurs at M_transition = mu * m * g * R = 30.0
GTEST_TEST(PizzaSaver, Sliding) {
  const double dt = 0.01;
  const double mu = 2. / 3.;
  const double k = 1.0e4;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.ls_max_iterations = 40;
  const double beta = kEps;  // No near-rigid regime.

  const double Mz = 40.0;
  const double weight = problem.mass() * problem.g();
  const Vector4d tau(0.0, 0.0, -weight, Mz);

  const SapSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 10, params, beta);

  EXPECT_GT(result.v(3), 0.0);  // It's accelerating.
  for (int i = 0; i < 3; ++i) {
    const double friction_impulse = result.gamma.segment<2>(3 * i).norm();
    const double normal_impulse = result.gamma(3 * i + 2);
    EXPECT_NEAR(friction_impulse, mu * normal_impulse,
                std::numeric_limits<double>::epsilon() * normal_impulse);
  }

  // To verify the line search throws when it doesn't converge, we set a low
  // maximum number of iterations and verify the solver fails for the right
  // reasons.
  params.ls_max_iterations = 1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      AdvanceNumSteps(problem, tau, 1, params),
      "Line search reached the maximum number of iterations.*");
}

// Verify we can also get a solution in the near-rigid regime. To trigger this
// regime we set a very large contact stiffness. In particular, for this test we
// apply a moment Mz such that Mz < mu * m * g * R, and the saver is in
// stiction. For this setup the transition occurs at
// M_transition = mu * m * g * R = 30.
GTEST_TEST(PizzaSaver, NearRigidStiction) {
  const double dt = 0.01;
  const double mu = 2. / 3.;
  // We use a very large value of stiffness that we know makes the solver fail
  // if the near-rigid transition strategy is not used.
  const double k = 1.0e20;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  const double Mz = 20.0;
  const Vector4d tau(0.0, 0.0, -problem.mass() * problem.g(), Mz);

  // We first verify that if we don't use the near-rigid regime strategy the
  // solver fails to converge.
  double beta = kEps;  // No near-rigid regime.
  EXPECT_THROW(AdvanceNumSteps(problem, tau, 30, params, beta), std::exception);

  // We try again the same problem but with with near-rigid transition enabled.
  // N.B. AdvanceNumSteps() creates a new SAP solver every time. Therefore it is
  // impossible that results from the previous computation affect this new
  // computation.
  beta = 1.0;  // Enable near-rigid regime with default value of beta.
  const SapSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 30, params, beta);

  // Expected generalized force.
  const Vector4d j_expected(0.0, 0.0, dt * problem.mass() * problem.g(),
                            -dt * Mz);

  // Maximum expected slip. See Castro et al. 2021 for details (Eq. 22).
  const double max_slip_expected = kDefaultSigma * dt * problem.g();

  EXPECT_TRUE(CompareMatrices(result.v.head<3>(), Vector3d::Zero(), 1.0e-9));
  EXPECT_TRUE((result.j - j_expected).norm() / j_expected.norm() <
              params.rel_tolerance);

  VectorXd gamma_normal(problem.kNumContacts);
  ExtractNormal(result.gamma, &gamma_normal);
  EXPECT_TRUE(CompareMatrices(
      gamma_normal,
      VectorXd::Constant(problem.kNumContacts, j_expected(2) / 3.0),
      params.rel_tolerance, MatrixCompareType::relative));

  VectorXd vc_normal(problem.kNumContacts);
  ExtractNormal(result.vc, &vc_normal);
  EXPECT_TRUE(
      CompareMatrices(vc_normal, VectorXd::Zero(problem.kNumContacts), 1.0e-9));
  const double friction_impulse_expected = dt * Mz / problem.radius() / 3.0;
  for (int i = 0; i < 3; ++i) {
    const double slip = result.vc.segment<2>(3 * i).norm();
    const double friction_impulse = result.gamma.segment<2>(3 * i).norm();
    const double normal_impulse = result.gamma(3 * i + 2);
    EXPECT_LE(friction_impulse, mu * normal_impulse);
    EXPECT_NEAR(friction_impulse, friction_impulse_expected,
                params.rel_tolerance * friction_impulse_expected);
    EXPECT_LE(slip, max_slip_expected);
  }
}

GTEST_TEST(PizzaSaver, NoConstraints) {
  const double dt = 0.01;
  const double mu = NAN;    // not used in this problem.
  const double k = NAN;     // not used in this problem.
  const double taud = NAN;  // not used in this problem.
  const PizzaSaverProblem problem(dt, mu, k, taud);

  const double weight = problem.mass() * problem.g();
  const Vector4d tau(0.0, 0.0, -weight, 0.0);

  const double theta = M_PI / 5;  // Arbitrary orientation.
  VectorXd q = Vector4d(0.0, 0.0, 0.0, theta);
  VectorXd v = VectorXd::Zero(problem.kNumVelocities);

  const auto contact_problem =
      problem.MakeContactProblemWithoutConstraints(q, v, tau);
  SapSolver<double> sap;
  SapSolverResults<double> result;
  const SapSolverStatus status =
      sap.SolveWithGuess(*contact_problem, v, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);

  // Verify no iterations were performed.
  const SapSolver<double>::SolverStats& stats = sap.get_statistics();
  EXPECT_EQ(stats.num_iters, 0);

  // The solution is trivial since constraint impulses are zero and v = v*.
  EXPECT_EQ(result.v, contact_problem->v_star());
  EXPECT_EQ(result.j, VectorXd::Zero(contact_problem->num_velocities()));
  EXPECT_EQ(result.gamma.size(), 0);
  EXPECT_EQ(result.vc.size(), 0);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
