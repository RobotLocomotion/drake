#include "drake/systems/analysis/region_of_attraction.h"

#include <algorithm>

#include "drake/math/continuous_lyapunov_equation.h"
#include "drake/math/quadratic_form.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace systems {
namespace analysis {

using solvers::MathematicalProgram;
using solvers::Solve;

using symbolic::Environment;
using symbolic::Expression;
using symbolic::Polynomial;
using symbolic::Substitution;
using symbolic::Variable;
using symbolic::Variables;

namespace {

// Assumes V positive semi-definite at the origin.
// If the Hessian of Vdot is negative definite at the origin, then we use
// Vdot = 0 => V >= rho (or x=0) via
//   maximize   rho
//   subject to (V-rho)*x'*x - Lambda*Vdot is SOS.
// If we cannot confirm negative definiteness, then we must ask instead for
// Vdot >=0 => V >= rho (or x=0).
Expression FixedLyapunovConvex(const solvers::VectorXIndeterminate& x,
                               const Expression& V, const Expression& Vdot) {
  // Check if the Hessian of Vdot is negative definite.
  Environment env;
  for (int i = 0; i < x.size(); i++) {
    env.insert(x(i), 0.0);
  }
  const Eigen::MatrixXd S =
      symbolic::Evaluate(symbolic::Jacobian(V.Jacobian(x), x), env);
  const Eigen::MatrixXd P =
      symbolic::Evaluate(symbolic::Jacobian(Vdot.Jacobian(x), x), env);
  // Eigen's recommended way of getting the Eigenvalues.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(P);
  DRAKE_THROW_UNLESS(eigensolver.info() == Eigen::Success);

  // A positive max eigenvalue indicates the system is locally unstable.
  const double max_eigenvalue = eigensolver.eigenvalues().maxCoeff();
  // According to the Lapack manual, the absolute accuracy of eigenvalues is
  // eps*max(|eigenvalues|), so I will write my thresholds in those units.
  // Anderson et al., Lapack User's Guide, 3rd ed. section 4.7, 1999.
  const double tolerance = 1e-8;
  const double max_abs_eigenvalue =
      eigensolver.eigenvalues().cwiseAbs().maxCoeff();
  DRAKE_THROW_UNLESS(max_eigenvalue <=
                     tolerance * std::max(1., max_abs_eigenvalue));

  bool Vdot_is_locally_negative_definite =
      (max_eigenvalue <= -tolerance * std::max(1., max_abs_eigenvalue));

  Polynomial V_balanced, Vdot_balanced;
  if (Vdot_is_locally_negative_definite) {
    // Then "balance" V and Vdot.
    const Eigen::MatrixXd T = math::BalanceQuadraticForms(S, -P);
    const VectorX<Expression> Tx = T*x;
    Substitution subs;
    for (int i=0; i<static_cast<int>(x.size()); i++) {
      subs.emplace(x(i), Tx(i));
    }
    V_balanced = Polynomial(V.Substitute(subs));
    Vdot_balanced = Polynomial(Vdot.Substitute(subs));
  } else {
    V_balanced = Polynomial(V);
    Vdot_balanced = Polynomial(Vdot);
  }

  MathematicalProgram prog;
  prog.AddIndeterminates(x);

  const int V_degree = V_balanced.TotalDegree();
  const int Vdot_degree = Vdot_balanced.TotalDegree();

  // TODO(russt): Add this as an argument once I have an example that needs it.
  // This is a reasonable guess.
  const int lambda_degree = Vdot_degree;
  const auto lambda = prog.NewFreePolynomial(Variables(x), lambda_degree);

  const auto rho = prog.NewContinuousVariables<1>("rho")[0];

  // Want (V-rho)(x'x)^d and Lambda*Vdot to be the same degree.
  const int d = std::floor((lambda_degree + Vdot_degree - V_degree) / 2);
  prog.AddSosConstraint(
      ((V_balanced - rho) * Polynomial(pow((x.transpose() * x)[0], d)) -
       lambda * Vdot_balanced));

  // If Vdot is indefinite, then the linearization does not inform us about the
  // local stability.  Add "lambda(x) is SOS" to confirm this local stability.
  if (!Vdot_is_locally_negative_definite) {
    prog.AddSosConstraint(lambda);
  }

  prog.AddCost(-rho);
  const auto result = Solve(prog);

  DRAKE_THROW_UNLESS(result.is_success());

  DRAKE_THROW_UNLESS(result.GetSolution(rho) > 0.0);
  return V / result.GetSolution(rho);
}

}  // namespace

Expression RegionOfAttraction(const System<double>& system,
                              const Context<double>& context,
                              const RegionOfAttractionOptions& options) {
  system.ValidateContext(context);
  DRAKE_THROW_UNLESS(context.has_only_continuous_state());

  const int num_states = context.num_continuous_states();
  VectorX<double> x0 = context.get_continuous_state_vector().CopyToVector();

  // Check that x0 is a fixed point.
  VectorX<double> xdot0 =
      system.EvalTimeDerivatives(context).get_vector().CopyToVector();
  DRAKE_THROW_UNLESS(xdot0.template lpNorm<Eigen::Infinity>() <= 1e-14);

  const auto symbolic_system = system.ToSymbolic();
  const auto symbolic_context = symbolic_system->CreateDefaultContext();
  symbolic_context->SetTimeStateAndParametersFrom(context);
  symbolic_system->FixInputPortsFrom(system, context, symbolic_context.get());

  // Subroutines should create their own programs to avoid incidental
  // sharing of costs or constraints.  However, we pass x and expect that
  // sub-programs will use AddIndeterminates(x).
  MathematicalProgram prog;
  // Define the relative coordinates: x_bar = x - x0
  const auto x_bar = prog.NewIndeterminates(num_states, "x");

  Environment x0env;
  for (int i = 0; i < num_states; i++) {
    x0env.insert(x_bar(i), 0.0);
  }

  // Evaluate the dynamics (in relative coordinates).
  symbolic_context->SetContinuousState(x0 + x_bar);
  const VectorX<Expression> f =
      symbolic_system->EvalTimeDerivatives(*symbolic_context)
          .get_vector()
          .CopyToVector();

  Expression V;
  bool user_provided_lyapunov_candidate =
      !options.lyapunov_candidate.EqualTo(Expression::Zero());

  if (user_provided_lyapunov_candidate) {
    DRAKE_THROW_UNLESS(options.lyapunov_candidate.is_polynomial());
    V = options.lyapunov_candidate;

    Substitution subs;
    subs.reserve(num_states);
    // If necessary, replace the state variables.
    if (options.state_variables.rows() > 0) {
      for (int i = 0; i < num_states; i++) {
        subs.emplace(options.state_variables(i), x0(i) + x_bar(i));
      }
    } else {  // just change to relative coordinates.
      for (int i = 0; i < num_states; i++) {
        subs.emplace(x_bar(i), x0(i) + x_bar(i));
      }
    }
    V = V.Substitute(subs);

    // Check that V has the right Variables.
    DRAKE_THROW_UNLESS(V.GetVariables().IsSubsetOf(Variables(x_bar)));

    // Check that V is positive definite.
    prog.AddSosConstraint(V);
    const auto result = Solve(prog);
    DRAKE_THROW_UNLESS(result.is_success());
  } else {
    // Solve a Lyapunov equation to find a candidate.
    const Eigen::MatrixXd A = Evaluate(Jacobian(f, x_bar), x0env);
    const Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(num_states, num_states);
    const Eigen::MatrixXd P = math::RealContinuousLyapunovEquation(A, Q);
    V = x_bar.dot(P * x_bar);
  }

  const Expression Vdot = V.Jacobian(x_bar).dot(f);

  V = FixedLyapunovConvex(x_bar, V, Vdot);

  // Put V back into global coordinates.
  Substitution subs;
  subs.reserve(num_states);
  if (options.state_variables.rows() > 0) {
    for (int i = 0; i < num_states; i++) {
      subs.emplace(x_bar(i), options.state_variables(i) - x0(i));
    }
  } else {
    for (int i = 0; i < num_states; i++) {
      subs.emplace(x_bar(i), x_bar(i) - x0(i));
    }
  }
  V = V.Substitute(subs);

  return V;
}

}  // namespace analysis
}  // namespace systems
}  // namespace drake
