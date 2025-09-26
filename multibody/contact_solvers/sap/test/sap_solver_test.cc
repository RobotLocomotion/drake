#include "drake/multibody/contact_solvers/sap/sap_solver.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/systems/framework/context.h"

using drake::systems::Context;
using Eigen::Matrix2d;
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
// friction, see [Castro et al. 2022].
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
    // Since contact constraints are involved, there must be at least one object
    // with a valid index, equal to zero.
    problem->set_num_objects(1);

    // Add contact constraints.
    const double phi0 = q0(2);
    const SapFrictionConeConstraint<double>::Parameters parameters{
        mu_, stiffness_, taud_, beta, 1.0e-3};

    // For these tests, only the signed distance phi is relevant.
    // Object indices must be valid, even though not used in these tests. Every
    // other configuration will be left uninitialized.
    const ContactConfiguration<double> configuration{
        .objectA = 0 /* valid, though not used */,
        .objectB = 0 /* valid, though not used */,
        .phi = phi0};

    MatrixXd J;  // Full system Jacobian for the three contacts.
    CalcContactJacobian(q0(3), &J);
    problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
        configuration, SapConstraintJacobian<double>{0, J.middleRows(0, 3)},
        parameters));
    problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
        configuration, SapConstraintJacobian<double>{0, J.middleRows(3, 3)},
        parameters));
    problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
        configuration, SapConstraintJacobian<double>{0, J.middleRows(6, 3)},
        parameters));

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

class PizzaSaverTest
    : public testing::TestWithParam<SapSolverParameters::LineSearchType> {
 public:
  static SapSolverResults<double> AdvanceNumSteps(
      const PizzaSaverProblem& problem, const VectorXd& tau, int num_steps,
      const SapSolverParameters& params, double beta = 1.0,
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
      const SapStatistics& stats = sap.get_statistics();

      if (cost_criterion_reached) {
        EXPECT_TRUE(stats.cost_criterion_reached);
      } else {
        EXPECT_TRUE(stats.optimality_criterion_reached);
      }
    }

    return result;
  }

  // Makes a problem for which we know the solution will be in stiction. If Mz <
  // mu * m * g * R, the saver should be in stiction (that is, the sliding
  // velocity should be smaller than the regularization parameter). Otherwise
  // the saver will start sliding. For this setup, the transition occurs at
  // M_transition = mu * m * g * R = 30.
  static PizzaSaverProblem MakeStictionProblem() {
    const double dt = 0.01;
    const double mu = 2. / 3.;
    const double k = 1.0e4;
    const double taud = 2.0 * dt;
    return PizzaSaverProblem(dt, mu, k, taud);
  }

  // Make a stiction problem with MakeStictionProblem() and verify the solution
  // is accurate to within `relative_tolerance`. `params` specifies SAP's
  // parameters. `cost_criterion_reached` specifies if we expect the cost
  // criterion to be reached for all time steps.
  static void VerifyStictionSolution(const SapSolverParameters& params,
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

    // Maximum expected slip. See Castro et al. 2022 for details (Eq. 31).
    const double max_slip_expected =
        kDefaultSigma * problem.mu() * problem.time_step() * problem.g();

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
};

// Solve a problem with no applied torque. In this case contact forces should
// balance weight.
TEST_P(PizzaSaverTest, NoAppliedTorque) {
  const double dt = 0.01;
  const double mu = 1.0;
  const double k = 1.0e4;
  const double taud = dt;
  const PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;  // Default set of parameters.
  params.line_search_type = GetParam();
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
TEST_P(PizzaSaverTest, ConvergenceWithExactGuess) {
  const double dt = 0.01;
  const double mu = 1.0;
  const double k = 1.0e4;
  const double taud = dt;
  const PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;  // Default set of parameters.
  params.line_search_type = GetParam();
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
  const SapStatistics& stats = sap.get_statistics();
  EXPECT_EQ(stats.num_iters, 0);

  // The guess was not even touched but directly copied into the results.
  EXPECT_EQ(result.v, v);
}

// Solve a stiction problem and verify the solution for a nominal set of solver
// parameters.
TEST_P(PizzaSaverTest, Stiction) {
  SapSolverParameters params;  // Default set of parameters.
  params.line_search_type = GetParam();

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
TEST_P(PizzaSaverTest, StictionAtTightOptimalityTolerance) {
  SapSolverParameters params;  // Default set of parameters.
  // Extremely tight tolerances to trigger the cost stopping criterion.
  params.abs_tolerance = 0;
  params.rel_tolerance = 0;
  params.line_search_type = GetParam();

  // Inform the unit test we want to verify the solver reaches the cost
  // convergence criterion first. This is true regardless of the line search
  // method since we specified zero tolerances above.
  const bool cost_criterion_reached = true;
  // Use a reasonable tolerance to verify the accuracy of the solution.
  const double relative_tolerance = 1.0e-6;
  VerifyStictionSolution(params, relative_tolerance, cost_criterion_reached);
}

// Test case with zero friction coefficient.
TEST_P(PizzaSaverTest, NoFriction) {
  const double dt = 0.01;
  const double mu = 0.0;
  const double k = 1.0e4;
  const double taud = dt;
  const int num_steps = 30;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;  // Default set of parameters.
  params.line_search_type = GetParam();
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
TEST_P(PizzaSaverTest, Sliding) {
  const double dt = 0.01;
  const double mu = 2. / 3.;
  const double k = 1.0e4;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;  // Default set of parameters.
  params.line_search_type = GetParam();
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

  // ls_max_iterations only pertains to backtracking line search.
  if (params.line_search_type != SapSolverParameters::LineSearchType::kExact) {
    // To verify the line search throws when it doesn't converge, we set a low
    // maximum number of iterations and verify the solver fails for the right
    // reasons.
    params.backtracking_line_search.max_iterations = 1;
    DRAKE_EXPECT_THROWS_MESSAGE(
        AdvanceNumSteps(problem, tau, 1, params),
        "Line search reached the maximum number of iterations.*");
  }
}

// Verify we can also get a solution in the near-rigid regime. To trigger this
// regime we set a very large contact stiffness. In particular, for this test we
// apply a moment Mz such that Mz < mu * m * g * R, and the saver is in
// stiction. For this set up the transition occurs at
// M_transition = mu * m * g * R = 30.
TEST_P(PizzaSaverTest, NearRigidStiction) {
  const double dt = 0.01;
  const double mu = 2. / 3.;
  // We use a very large value of stiffness that we know makes the solver fail
  // if the near-rigid transition strategy is not used.
  const double k = 1.0e20;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;  // Default set of parameters.
  params.line_search_type = GetParam();
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

  // Maximum expected slip. See Castro et al. 2022 for details (Eq. 31).
  const double max_slip_expected =
      kDefaultSigma * problem.mu() * dt * problem.g();

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

TEST_P(PizzaSaverTest, NoConstraints) {
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
  SapSolverParameters params;  // Default set of parameters.
  params.line_search_type = GetParam();
  sap.set_parameters(params);
  SapSolverResults<double> result;
  const SapSolverStatus status =
      sap.SolveWithGuess(*contact_problem, v, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);

  // Verify no iterations were performed.
  const SapStatistics& stats = sap.get_statistics();
  EXPECT_EQ(stats.num_iters, 0);

  // The solution is trivial since constraint impulses are zero and v = v*.
  EXPECT_EQ(result.v, contact_problem->v_star());
  EXPECT_EQ(result.j, VectorXd::Zero(contact_problem->num_velocities()));
  EXPECT_EQ(result.gamma.size(), 0);
  EXPECT_EQ(result.vc.size(), 0);
}

INSTANTIATE_TEST_SUITE_P(
    TestLineSearchMethods, PizzaSaverTest,
    testing::Values(SapSolverParameters::LineSearchType::kBackTracking,
                    SapSolverParameters::LineSearchType::kExact));

// Constraint to keep velocities within given bounds vl (lower) and vu (upper).
// Recall that SAP constraints are compliant and therefore non-zero impulses
// result when velocities are slightly outside of these bounds. This constraint
// applies bounds to the velocities that belong to a given clique. Both vl and
// vu have the same size, equal to the number of generalized velocities nc for
// the specified clique.
// This constraint models impulses as: γ = max(0, −R⁻¹⋅(vc−v̂)), where
// regularization R is given and bias v̂ = [vl, -vu], of size 2⋅nc.
template <typename T>
class LimitConstraint final : public SapConstraint<T> {
 public:
  // Constructs a limit constraint on `clique` with lower limit vl, upper
  // limit vu and regularization R.
  LimitConstraint(int clique, const VectorX<T>& vl, const VectorX<T>& vu,
                  VectorX<T> R)
      : SapConstraint<T>({clique, CalcConstraintJacobian(vl.size())}, {}),
        R_(std::move(R)),
        vhat_(ConcatenateVectors(vl, -vu)) {
    DRAKE_DEMAND(vl.size() == vu.size());
    DRAKE_DEMAND(R_.size() == 2 * vl.size());
    DRAKE_DEMAND((vl.array() <= vu.array()).all());
  }

  // For this constraint the projection is γ = P(y) = max(0, y), componentwise.
  void Project(const Eigen::Ref<const VectorX<double>>& y,
               EigenPtr<VectorX<double>> gamma,
               MatrixX<double>* dPdy = nullptr) const {
    // For this constraint the number of equations equals the number of
    // velocities in the constrained clique.
    const int nv = this->num_constraint_equations();
    if (dPdy != nullptr) {
      dPdy->resize(nv, nv);
      dPdy->setZero(nv, nv);
    }
    for (int i = 0; i < nv; ++i) {
      if (y(i) > 0) {
        (*gamma)(i) = y(i);
        if (dPdy != nullptr) {
          (*dPdy)(i, i) = 1.0;
        }
      } else {
        (*gamma)(i) = 0.0;
      }
    }
  }

 private:
  static VectorX<T> ConcatenateVectors(const VectorX<T>& v1,
                                       const VectorX<T>& v2) {
    VectorX<T> v(v1.size() + v2.size());
    v << v1, v2;
    return v;
  }

  static VectorX<T> CalcConstraintFunction(const VectorX<T>& vl,
                                           const VectorX<T>& vu,
                                           const VectorX<T>& v) {
    DRAKE_DEMAND(vl.size() == v.size());
    DRAKE_DEMAND(vu.size() == v.size());
    const int nv = v.size();
    const VectorX<T> g = (VectorX<T>(2 * nv) << v - vl, vu - v).finished();
    return g;
  }

  static MatrixX<T> CalcConstraintJacobian(int nv) {
    MatrixX<T> J = MatrixX<T>::Identity(2 * nv, nv);
    J.topRows(nv) = MatrixX<T>::Identity(nv, nv);
    J.bottomRows(nv) = -MatrixX<T>::Identity(nv, nv);
    return J;
  }

  LimitConstraint(const LimitConstraint&) = default;

  std::unique_ptr<AbstractValue> DoMakeData(
      const T&, const Eigen::Ref<const VectorX<T>>&) const final {
    // We'll store the constraint velocity in our data.
    VectorX<T> vc(this->num_constraint_equations());
    return SapConstraint<T>::MoveAndMakeAbstractValue(std::move(vc));
  }
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* abstract_data) const final {
    abstract_data->get_mutable_value<VectorX<T>>() = vc;
  }
  T DoCalcCost(const AbstractValue& abstract_data) const final {
    // SAP regularizer cost, the R-norm of the impulse.
    VectorX<T> gamma(this->num_constraint_equations());
    this->CalcImpulse(abstract_data, &gamma);
    return 0.5 * gamma.dot(R_.asDiagonal() * gamma);
  }
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const final {
    const auto& vc = abstract_data.get_value<VectorX<T>>();
    const VectorX<T> y = R_.asDiagonal() * (vhat_ - vc);
    Project(y, gamma);
  }
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const final {
    const auto& vc = abstract_data.get_value<VectorX<T>>();
    const VectorX<T> y = R_.asDiagonal() * (vhat_ - vc);
    VectorX<T> gamma(vc.size());
    Project(y, &gamma, G);
    (*G) *= R_.cwiseSqrt().asDiagonal();
  }
  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<LimitConstraint<T>>(new LimitConstraint<T>(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    throw std::runtime_error("DoToDouble() not used in these unit tests.");
  }

  VectorX<T> R_;     // Regularization.
  VectorX<T> vhat_;  // Bias.
};

// This fixture is used to test the number of iterations performed by SAP's
// Newton solver. We set up a contact problem for which the solution is v = v*,
// the "free-motion" velocities. The problem has three cliques and a single
// LimitConstraint on the first clique. We can test how the Newton iterations
// perform by changing the initial guess. For instance, when the initial guess
// is within the contraint's bounds, the cost is quadratic and we expect SAP to
// converge in a single Newton iteration.
class SapNewtonIterationTest
    : public testing::TestWithParam<SapSolverParameters::LineSearchType> {
 public:
  void SetUp() override {
    const double time_step = 0.01;

    // Arbitrary non-identity SPD matrices to build the dynamics matrix A.
    // clang-format off
    constexpr int num_velocities{9};  // Total number of velocities.
    constexpr int clique1_nv{3};  // Number of velocities for clique 1.
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

    // Velocity limits on clique 1.
    vl_ = Vector3d(-3., -2, -1.);
    vu_ = Vector3d(1., 2, 3.);

    // We make a v* that is inside the "box" specified by vl and vu.
    VectorXd v_star =
        VectorXd::LinSpaced(num_velocities, 1., 1. * num_velocities);
    // The limit constraint applies to the second clique (clique=1) only. This
    // clique has clique1_nv = 3 degrees of freedom starting at entry 2 in the
    // full vector of generalized velocities. Thus the .segment<clique1_nv>(2)
    // below:
    auto v_star_clique1 =
        v_star.segment<clique1_nv>(2);  // Velocities for clique 1.
    v_star_clique1 = 0.5 * (vl_ + vu_);
    v_star_.resize(num_velocities);
    v_star_ = v_star;

    std::vector<MatrixXd> A = {S22, S33, S44};
    sap_problem_ = std::make_unique<SapContactProblem<double>>(
        time_step, std::move(A), std::move(v_star));

    constexpr int num_limit_constraint_equations = 2 * clique1_nv;
    VectorXd R = VectorXd::Constant(num_limit_constraint_equations, 1.0e-3);
    sap_problem_->AddConstraint(
        std::make_unique<LimitConstraint<double>>(1, vl_, vu_, std::move(R)));
    // Model and context to test solver internals.
    model_ = std::make_unique<SapModel<double>>(
        sap_problem_.get(), SapHessianFactorizationType::kBlockSparseCholesky);
    context_ = model_->MakeContext();
  }

  // We verify our supernodal algebra by comparing the Hessian reconstructed
  // with the supernodal solver against our already tested dense algebra.
  void VerifySupernodalHessian(const SapSolver<double>& sap,
                               const VectorXd& v_guess) const {
    // Verify Hessian obtained with sparse supernodal algebra.
    auto v = model_->GetMutableVelocities(context_.get());
    model_->velocities_permutation().Apply(v_guess, &v);

    // Sanity check the default is not dense, since we'll use a dense
    // factorization to construct an expected value.
    ASSERT_NE(model_->hessian_type(), SapHessianFactorizationType::kDense);

    // To reconstruct a dense Hessian from its factorization we do:
    //  1. Apply factorization to the identity matrix to get its inverse, Hinv.
    //  2. Explicitly compute the inverse of Hinv to get H.
    const HessianFactorizationCache& factorization =
        model_->EvalHessianFactorizationCache(*context_);
    MatrixXd Hinv = MatrixXd::Identity(v.size(), v.size());
    factorization.SolveInPlace(&Hinv);
    const MatrixXd H = Hinv.inverse();

    // Compute Hessian using dense algebra.
    HessianFactorizationCache dense_factorization(
        SapHessianFactorizationType::kDense, &model_->dynamics_matrix(),
        &model_->constraints_bundle().J());
    dense_factorization.UpdateWeightMatrixAndFactor(
        model_->EvalConstraintsHessian(*context_));
    MatrixXd Hinv_expected = MatrixXd::Identity(v.size(), v.size());
    dense_factorization.SolveInPlace(&Hinv_expected);
    const MatrixXd H_expected = Hinv.inverse();

    // Verify dense and sparse hessians match.
    EXPECT_TRUE(CompareMatrices(H, H_expected, 3.0 * kEps,
                                MatrixCompareType::relative));
  }

  // Compute next step velocity using the provide set of `params` and the given
  // guess `v_guess`.
  VectorXd SolveWithGuess(const SapSolverParameters& params,
                          const VectorXd& v_guess) const {
    SapSolver<double> sap;
    sap.set_parameters(params);
    SapSolverResults<double> result;
    const SapSolverStatus status =
        sap.SolveWithGuess(*sap_problem_, v_guess, &result);
    EXPECT_EQ(status, SapSolverStatus::kSuccess);
    return result.v;
  }

  // Compare solutions obtained with dense and supernodal algebra.
  void CompareDenseAgainstSupernodal(const VectorXd& v_guess) const {
    const double relative_tolerance = kEps;

    // Perform computation with supernodal algebra.
    SapSolverParameters params_supernodal;  // Default set of parameters.
    params_supernodal.abs_tolerance = 0;
    params_supernodal.rel_tolerance = relative_tolerance;
    params_supernodal.line_search_type = GetParam();
    const VectorXd v_supernodal = SolveWithGuess(params_supernodal, v_guess);

    // Perform computation with dense algebra.
    SapSolverParameters params_dense;  // Default set of parameters.
    params_dense.linear_solver_type = SapHessianFactorizationType::kDense;
    params_dense.abs_tolerance = 0;
    params_dense.rel_tolerance = relative_tolerance;
    params_dense.line_search_type = GetParam();
    const VectorXd v_dense = SolveWithGuess(params_dense, v_guess);

    // We expected results computed with dense and supernodal algebra to match
    // close to machine epsilon for this small problem.
    EXPECT_TRUE(CompareMatrices(v_supernodal, v_dense, 5.0 * relative_tolerance,
                                MatrixCompareType::relative));
  }

 protected:
  VectorXd vl_;
  VectorXd vu_;
  VectorXd v_star_;
  std::unique_ptr<SapContactProblem<double>> sap_problem_;
  std::unique_ptr<SapModel<double>> model_;
  std::unique_ptr<Context<double>> context_;
};

// Unit test that SAP performs no computation when provided with an initial
// guess that satisfies optimality condition.
TEST_P(SapNewtonIterationTest, GuessIsTheSolution) {
  SapSolver<double> sap;
  SapSolverParameters parameters;
  parameters.line_search_type = GetParam();
  sap.set_parameters(parameters);
  const VectorXd v_guess = v_star_;
  SapSolverResults<double> result;
  const SapSolverStatus status =
      sap.SolveWithGuess(*sap_problem_, v_guess, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);

  // Verify optimality is satisfied but no iterations were performed.
  const SapStatistics& stats = sap.get_statistics();
  EXPECT_EQ(stats.num_iters, 0);
  EXPECT_TRUE(stats.optimality_criterion_reached);

  // Since the initial guess is the solution to this problem, we expect the
  // result ot be an exact copy of it. Constraints are not active and therefore
  // impulses are zero.
  EXPECT_EQ(result.v, v_guess);
  EXPECT_EQ(result.j, VectorXd::Zero(sap_problem_->num_velocities()));
  EXPECT_EQ(result.gamma,
            VectorXd::Zero(sap_problem_->num_constraint_equations()));
}

// For this particular problem, the cost is quadratic for velocities within the
// bounds of the limits imposed by the constraint. Therefore we expect the
// Newton solver to achieve convergence in just a single iteration, to machine
// precision when a full step (alpha=1) is taken by the line search.
TEST_P(SapNewtonIterationTest, GuessWithinLimits) {
  SapSolver<double> sap;
  SapSolverParameters params;
  params.line_search_type = GetParam();
  if (params.line_search_type ==
      SapSolverParameters::LineSearchType::kBackTracking) {
    // For the backtracking line search, we set the maximum step size so that
    // the method tries alpha = 1.0. Since alpha_m = alpha_max * rho^m
    // (m=0..max_iterations), we set alpha_max = 1.0/ls_rho so that alpha_1 =
    // 1.0 on the second line search iteration. Since in this case the guess is
    // within the constraint limits, where the cost is exactly quadratic, one
    // Newton iteration with alpha = 1 will achieve convergence within machine
    // precision.
    params.backtracking_line_search.alpha_max =
        1.0 / params.backtracking_line_search.rho;
  }
  sap.set_parameters(params);

  // Arbitrary initial guess within the velocity limits but different from the
  // solution v_star_.
  VectorXd v_guess = v_star_;
  v_guess.segment<3>(2) = 0.9 * vl_ + 0.1 * vu_;
  SapSolverResults<double> result;
  const SapSolverStatus status =
      sap.SolveWithGuess(*sap_problem_, v_guess, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);

  // Since we provide the guess to be within the limits, and we know the
  // solution is v = v*, we expect the solver achieve convergence in a single
  // Newton iteration.
  const SapStatistics& stats = sap.get_statistics();
  EXPECT_EQ(stats.num_iters, 1);
  // We expect two backtracking line search iterations given alpha_max != 1.
  // Since the problem is quadratic, we expect the exact line search to
  // take only one iteration.
  const int expected_line_search_iterations =
      params.line_search_type == SapSolverParameters::kBackTracking ? 2 : 1;
  EXPECT_EQ(stats.num_line_search_iters, expected_line_search_iterations);
  // This problem is very well conditioned, we expect convergence on the
  // optimality condition.
  EXPECT_TRUE(stats.optimality_criterion_reached);

  // The solution exactly equals the initial guess v* within machine epsilon
  // given in this case the problem is linear.
  EXPECT_TRUE(CompareMatrices(result.v, v_star_, 10.0 * kEps,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.j,
                              VectorXd::Zero(sap_problem_->num_velocities()),
                              3.0 * kEps, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      result.gamma, VectorXd::Zero(sap_problem_->num_constraint_equations()),
      3.0 * kEps, MatrixCompareType::absolute));

  VerifySupernodalHessian(sap, v_guess);

  // Verify solution computed with dense and supernodal algebra match.
  CompareDenseAgainstSupernodal(v_guess);
}

// For this problem when the initial guess is outside the constraint's limits
// the cost is non-linear and we need several Newton iterations to achieve
// convergence.
TEST_P(SapNewtonIterationTest, GuessOutsideLimits) {
  SapSolver<double> sap;
  SapSolverParameters params;
  params.line_search_type = GetParam();
  if (params.line_search_type ==
      SapSolverParameters::LineSearchType::kBackTracking) {
    // For the backtracking line search, we use the same parameters as in
    // the unit test SapNewtonIterationTest__GuessWithinLimits so that the only
    // difference between the two tests is the initial guess given to the
    // solver.
    params.backtracking_line_search.alpha_max =
        1.0 / params.backtracking_line_search.rho;
  }
  sap.set_parameters(params);

  // Arbitrary initial guess outside the constraint bounds to force several
  // Newton iterations.
  VectorXd v_guess = v_star_;
  v_guess.segment<3>(2) = Vector3d(1.2 * vl_(0) /* below lower limit */,
                                   v_star_(1) /* within limits */,
                                   1.1 * vu_(2) /* above upper limit */);
  SapSolverResults<double> result;
  const SapSolverStatus status =
      sap.SolveWithGuess(*sap_problem_, v_guess, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);

  // Since the initial guess is outside the constraint's bounds, and we know the
  // solution is v = v* inside the bounds, we expect several Newton iterations
  // to achieve convergence.
  const SapStatistics& stats = sap.get_statistics();
  EXPECT_GT(stats.num_iters, 1);
  EXPECT_GT(stats.num_line_search_iters, 1);
  // This problem is very well conditioned, we expect convergence on the
  // optimality condition.
  EXPECT_TRUE(stats.optimality_criterion_reached);

  // Even though the initial guess is outside the constraint limits, once a SAP
  // iteration reaches a solution within the constraint's bounds, it will
  // converge within machine epsilon in the next iteration.
  EXPECT_TRUE(CompareMatrices(result.v, v_star_, 3.0 * kEps,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.j,
                              VectorXd::Zero(sap_problem_->num_velocities()),
                              3.0 * kEps, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      result.gamma, VectorXd::Zero(sap_problem_->num_constraint_equations()),
      3.0 * kEps, MatrixCompareType::absolute));

  // Verify Hessian obtained with sparse supernodal algebra.
  VerifySupernodalHessian(sap, v_guess);

  // Verify solution computed with dense and supernodal algebra match.
  CompareDenseAgainstSupernodal(v_guess);
}

INSTANTIATE_TEST_SUITE_P(
    TestLineSearchMethods, SapNewtonIterationTest,
    testing::Values(SapSolverParameters::LineSearchType::kBackTracking,
                    SapSolverParameters::LineSearchType::kExact));

// A SAP constraint to model a constant impulse g.
// The cost is defined as ℓ(vc) = -g⋅vc such that γ = −∂ℓ/∂vc = g is constant.
class ConstantForceConstraint final : public SapConstraint<double> {
 public:
  // Constructs a constraint that produces a constant impulse g.
  ConstantForceConstraint(SapConstraintJacobian<double> J, VectorXd g)
      : SapConstraint<double>(std::move(J), {}), g_(std::move(g)) {}

 private:
  // Copy constructor for cloning.
  ConstantForceConstraint(const ConstantForceConstraint&) = default;

  std::unique_ptr<AbstractValue> DoMakeData(
      const double&, const Eigen::Ref<const VectorXd>&) const final {
    // We'll store the constraint velocity in our data.
    VectorXd vc(this->num_constraint_equations());
    return SapConstraint<double>::MoveAndMakeAbstractValue(std::move(vc));
  }
  void DoCalcData(const Eigen::Ref<const VectorXd>& vc,
                  AbstractValue* abstract_data) const final {
    abstract_data->get_mutable_value<VectorXd>() = vc;
  }
  double DoCalcCost(const AbstractValue& abstract_data) const final {
    const auto& vc = abstract_data.get_value<VectorXd>();
    return -g_.dot(vc);
  }
  void DoCalcImpulse(const AbstractValue&,
                     EigenPtr<VectorXd> gamma) const final {
    *gamma = g_;
  }
  void DoCalcCostHessian(const AbstractValue&, MatrixXd* G) const final {
    // The cost is linear, therefore its Hessian is Zero.
    const int ne = num_constraint_equations();
    G->setZero(ne, ne);
  }
  std::unique_ptr<SapConstraint<double>> DoClone() const final {
    return std::unique_ptr<ConstantForceConstraint>(
        new ConstantForceConstraint(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    throw std::runtime_error("DoToDouble() not used in these unit tests.");
  }

  const VectorXd g_;  // Constant impulse.
};

// For as long as they are convex, SAP admits constraints with a generic
// functional form. Thus costs can become negative and cannot be assumed to be
// positive. This test verifies that the SAP solver can handle constraints with
// costs that can be negative. For this test we setup a contact problem with an
// arbitrary dynamics matrix A, zero v*, and a ConstantForceConstraint with
// known impulse g with identity Jacobian, so that the optimality conditions
// simply reads: A⋅v = g.
GTEST_TEST(SapSolver, ConstraintWithNegativeCost) {
  // Problem data: time step, matrix A and constant impulse g.
  const double time_step = 0.1;
  const Vector2d g(2.0, 5.0);
  const Matrix2d S22 = 0.1 * (Matrix2d() << 2, 1, 1, 2).finished();

  // Make contact problem.
  std::vector<MatrixX<double>> A = {S22};
  SapContactProblem<double> problem(time_step, std::move(A), Vector2d::Zero());
  SapConstraintJacobian<double> J(0 /* the one clique */, Matrix2d::Identity());
  problem.AddConstraint(
      std::make_unique<ConstantForceConstraint>(std::move(J), g));

  // Initial guess that produces a negative cost ℓ = −‖g‖².
  VectorXd v_guess = g;

  // Solve problem.
  SapSolver<double> solver;
  SapSolverResults<double> result;
  const SapSolverStatus status =
      solver.SolveWithGuess(problem, v_guess, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);

  // Since the cost is a combination of a quadratic term (from A) and a linear
  // term (from the constant impulse constraint), we expect the solver to
  // achieve convergence in a single Newton iteration.
  const SapStatistics& stats = solver.get_statistics();
  EXPECT_EQ(stats.num_iters, 1);

  // The whole purpose of this test is to verify the behavior of SAP when the
  // cost becomes negative. Here we verify this indeed happened.
  EXPECT_EQ(stats.cost.size(), 2);  // Initial cost and final cost.
  const double ell0 = stats.cost[0];
  EXPECT_LT(ell0, 0.0);

  // Since the problem is quadratic, we expect the exact line search to
  // take only one iteration.
  EXPECT_EQ(stats.num_line_search_iters, 1);
  // This problem is very well conditioned, we expect convergence on the
  // optimality condition.
  EXPECT_TRUE(stats.optimality_criterion_reached);

  // The momentum equation (the optimality condition) for this case is simply
  // A⋅v = g, from where we know the solution is:
  const VectorXd v_expected = S22.llt().solve(g);
  EXPECT_TRUE(CompareMatrices(result.v, v_expected, 4 * kEps,
                              MatrixCompareType::relative));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
