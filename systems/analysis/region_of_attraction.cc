#include "drake/systems/analysis/region_of_attraction.h"

#include <algorithm>

#include "drake/math/continuous_lyapunov_equation.h"
#include "drake/math/matrix_util.h"
#include "drake/math/quadratic_form.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace analysis {

using Eigen::MatrixXd;

using math::IsPositiveDefinite;

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
//   subject to (V-rho)*(x'*x)^d - Lambda*Vdot is SOS.
// If we cannot confirm negative definiteness, then we must ask instead for
// Vdot >=0 => V >= rho (or x=0).
Expression FixedLyapunovConvex(
    const solvers::VectorXIndeterminate& x, const Expression& V,
    const Expression& Vdot, const std::optional<solvers::SolverId>& solver_id,
    const std::optional<solvers::SolverOptions>& solver_options) {
  // Check if the Hessian of Vdot is negative definite.
  Environment env;
  for (int i = 0; i < x.size(); i++) {
    env.insert(x(i), 0.0);
  }
  const Eigen::MatrixXd P =
      symbolic::Evaluate(symbolic::Jacobian(Vdot.Jacobian(x), x), env);
  const double tolerance = 1e-8;
  bool Vdot_is_locally_negative_definite = IsPositiveDefinite(-P, tolerance);

  Polynomial V_balanced, Vdot_balanced;
  if (Vdot_is_locally_negative_definite) {
    // Then "balance" V and Vdot.
    const Eigen::MatrixXd S =
        symbolic::Evaluate(symbolic::Jacobian(V.Jacobian(x), x), env);
    const Eigen::MatrixXd T = math::BalanceQuadraticForms(S, -P);
    const VectorX<Expression> Tx = T * x;
    Substitution subs;
    for (int i = 0; i < static_cast<int>(x.size()); i++) {
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

  // TODO(russt): Add this as an option once I have an example that needs it.
  // This is a reasonable guess: we want the multiplier to be able to compete
  // with terms in Vdot, and to be even (since it may be SOS below).
  const int lambda_degree = std::ceil(Vdot_degree / 2.0) * 2;
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
  solvers::MathematicalProgramResult result;
  if (solver_id.has_value()) {
    const auto solver = solvers::MakeSolver(solver_id.value());
    solver->Solve(prog, std::nullopt, solver_options, &result);
  } else {
    result = Solve(prog, std::nullopt, solver_options);
  }

  DRAKE_THROW_UNLESS(result.is_success());

  DRAKE_THROW_UNLESS(result.GetSolution(rho) > 0.0);
  return V / result.GetSolution(rho);
}

// Variant of FixedLyapunovConvex which takes Vdot(x,xdot), and certifies the
// condition on the variety defined by the residuals g(x,xdot)=0.
// Vdot(x,z) = 0, g(x,z)=0 => V(x) >= rho (or x=0) via
//   maximize   rho
//   subject to (V-rho)*(x'*x)^d - Lambda*Vdot + Lambda_g*g is SOS.
// If we cannot confirm negative definiteness, then we must ask instead for
// Vdot(x,z) >=0, g(x,z)=0 => V >= rho (or x=0).
Expression FixedLyapunovConvexImplicit(
    const solvers::VectorXIndeterminate& x,
    const solvers::VectorXIndeterminate& xdot, const Expression& V,
    const Expression& Vdot, const VectorX<Expression>& g) {
  // Check if the Hessian of Vdot is negative definite on the tangent space.
  // Given Vdot(x,z) and g(x,z)=0, we wish to test whether yᵀQy ≤ 0 for all
  // y where Gy=0, where y=[x,z], P = Hessian(Vdot,y), and G=dgdy.  To do this,
  // we find N as an orthonormal basis for the nullspace of G, and confirm that
  // NᵀPN is negative definite.
  Environment env;
  for (int i = 0; i < x.size(); i++) {
    env.insert(x(i), 0.0);
  }
  for (int i = 0; i < xdot.size(); i++) {
    env.insert(xdot(i), 0.0);
  }
  solvers::VectorXIndeterminate y(x.size() + xdot.size());
  y << x, xdot;
  const Eigen::MatrixXd P =
      symbolic::Evaluate(symbolic::Jacobian(Vdot.Jacobian(y), y), env);
  const Eigen::MatrixXd G = symbolic::Evaluate(symbolic::Jacobian(g, y), env);
  Eigen::FullPivLU<MatrixXd> lu(G);
  MatrixXd N = lu.kernel();
  const double tolerance = 1e-8;
  bool Vdot_is_locally_negative_definite =
      IsPositiveDefinite(-N.transpose() * P * N, tolerance);

  Polynomial V_poly(V);
  Polynomial Vdot_poly(Vdot);
  // TODO(russt): implement balancing.

  MathematicalProgram prog;
  prog.AddIndeterminates(x);
  prog.AddIndeterminates(xdot);

  const int V_degree = V_poly.TotalDegree();
  const int Vdot_degree = Vdot_poly.TotalDegree();

  // TODO(russt): Add this as an option once I have an example that needs it.
  // This is a reasonable guess: we want the multiplier to be able to compete
  // with terms in Vdot, and to be even (since it may be SOS below).
  const int lambda_degree = std::ceil(Vdot_degree / 2.0) * 2;
  const Polynomial lambda = prog.NewFreePolynomial(Variables(y), lambda_degree);
  VectorX<Polynomial> lambda_g(g.size());
  VectorX<Polynomial> g_poly(g.size());
  for (int i = 0; i < g.size(); ++i) {
    // Want λ_g[i] * g[i] to have the same degree as λ * Vdot.
    const int lambda_gi_degree = std::max(
        lambda_degree + Vdot_degree - Polynomial(g[0]).TotalDegree(), 0);
    lambda_g[i] = prog.NewFreePolynomial(Variables(y), lambda_gi_degree);
    g_poly[i] = Polynomial(g[i]);
  }

  const auto rho = prog.NewContinuousVariables<1>("rho")[0];

  // Want (V-rho)(x'x)^d and Lambda*Vdot to be the same degree.
  const int d = std::floor((lambda_degree + Vdot_degree - V_degree) / 2);
  prog.AddSosConstraint(
      ((V_poly - rho) * Polynomial(pow((x.transpose() * x)[0], d)) -
       lambda * Vdot_poly + lambda_g.dot(g_poly)));

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
    solvers::MathematicalProgramResult result;
    if (options.solver_id.has_value()) {
      auto solver = solvers::MakeSolver(options.solver_id.value());
      solver->Solve(prog, std::nullopt, options.solver_options, &result);
    } else {
      result = Solve(prog, std::nullopt, options.solver_options);
    }
    DRAKE_THROW_UNLESS(result.is_success());
  } else {
    // Solve a Lyapunov equation to find a candidate.
    const auto linearized_system =
        Linearize(system, context, InputPortSelection::kNoInput,
                  OutputPortSelection::kNoOutput);
    const Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(num_states, num_states);
    const Eigen::MatrixXd P =
        math::RealContinuousLyapunovEquation(linearized_system->A(), Q);
    V = x_bar.dot(P * x_bar);
  }

  // Evaluate the dynamics (in relative coordinates).
  symbolic_context->SetContinuousState(x0 + x_bar);

  if (options.use_implicit_dynamics) {
    const auto derivatives = symbolic_system->AllocateTimeDerivatives();
    const solvers::VectorXIndeterminate xdot =
        prog.NewIndeterminates(derivatives->size(), "xdot");
    const Expression Vdot = V.Jacobian(x_bar).dot(xdot);
    derivatives->SetFromVector(xdot.cast<Expression>());
    VectorX<Expression> g(
        symbolic_system->implicit_time_derivatives_residual_size());
    symbolic_system->CalcImplicitTimeDerivativesResidual(*symbolic_context,
                                                         *derivatives, &g);
    V = FixedLyapunovConvexImplicit(x_bar, xdot, V, Vdot, g);
  } else {
    const VectorX<Expression> f =
        symbolic_system->EvalTimeDerivatives(*symbolic_context)
            .get_vector()
            .CopyToVector();
    const Expression Vdot = V.Jacobian(x_bar).dot(f);
    V = FixedLyapunovConvex(x_bar, V, Vdot, options.solver_id,
                            options.solver_options);
  }

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
