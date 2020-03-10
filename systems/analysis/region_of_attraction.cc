#include "drake/systems/analysis/region_of_attraction.h"

#include "drake/math/continuous_lyapunov_equation.h"
#include "drake/math/matrix_util.h"
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

// Assumes V positive semi-definite and the Hessian of Vdot is negative
// definite at the origin (negative definiteness of Vdot is required to use
// Vdot = 0 instead of Vdot >=0 in the form below).
// Use Vdot = 0 => V >= rho (or x=0) via
//   maximize   rho
//   subject to (V-rho)*x'*x - Lambda*Vdot is SOS.
Expression FixedLyapunovConvex(const solvers::VectorXIndeterminate& x,
                               const Polynomial& V, const Polynomial& Vdot) {
  MathematicalProgram prog;
  prog.AddIndeterminates(x);

  // TODO(russt): Add this as an argument once I have an example that needs it.
  // This is a reasonable guess.
  const int V_degree = V.TotalDegree();
  const int Vdot_degree = Vdot.TotalDegree();
  const int lambda_degree = Vdot_degree;
  const auto lambda = prog.NewFreePolynomial(Variables(x), lambda_degree);

  const auto rho = prog.NewContinuousVariables<1>("rho")[0];

  // Want (V-rho)(x'x)^d and Lambda*Vdot to be the same degree.
  const int d = std::floor((lambda_degree + Vdot_degree - V_degree) / 2);
  // TODO(russt): Remove ToExpression below pending resolution of #12833.
  prog.AddSosConstraint(
      ((V - rho) * Polynomial(pow((x.transpose() * x)[0], d)) - lambda * Vdot)
          .ToExpression());

  prog.AddCost(-rho);
  const auto result = Solve(prog);

  DRAKE_THROW_UNLESS(result.is_success());

  DRAKE_THROW_UNLESS(result.GetSolution(rho) > 0.0);
  return V.ToExpression() / result.GetSolution(rho);
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
  // Time and parameters should just be doubles (not Variables).
  symbolic_context->SetTime(0.0);
  symbolic_context->get_mutable_parameters().SetFrom(context.get_parameters());

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

  if (user_provided_lyapunov_candidate) {
    // Check that the Hessian of Vdot is negative definite.
    const Eigen::MatrixXd H = symbolic::Evaluate(
        symbolic::Jacobian(Vdot.Jacobian(x_bar), x_bar), x0env);
    DRAKE_THROW_UNLESS(math::IsPositiveDefinite(-H, 1e-8));
  }

  // TODO(russt): implement numerical "balancing" from matlab version.
  // https://github.com/RobotLocomotion/drake/blob
  //        /last_sha_with_original_matlab/drake/matlab/systems
  //        /%40PolynomialSystem/regionOfAttraction.m

  V = FixedLyapunovConvex(x_bar, Polynomial(V), Polynomial(Vdot));

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
