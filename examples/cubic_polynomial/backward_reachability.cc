// A simple "hello world" example of computing a finite-time backward reachable
// set for a polynomial system using convex optimization.
//
// This implements the occupation measure-based SDP relaxation described in
// "Convex computation of the region of attraction of polynomial control
// systems" by Didier Henrion and Milan Korda, IEEE Transactions on Automatic
// Control, vol. 59, no. 2, pp. 297-312, Feb. 2014.
//
// TODO(jadecastro) Transcribe this example into Python.
#include <cmath>
#include <ostream>

#include "drake/common/proto/call_python.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace {

using std::cout;
using std::endl;
using std::pow;

using common::CallPython;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;

using MapType = Polynomial::MapType;
using Bound = std::map<Variable, double>;
using BoundingBox = std::pair<Bound, Bound>;

/// Defines the autonomous cubic polynomial system:
///   ẋ = 100 x(x - 0.5)(x + 0.5) = 100 x³ - 25 x
template <typename T>
class CubicSystem : public systems::VectorSystem<T> {
 public:
  CubicSystem() : systems::VectorSystem<T>(0, 0) {
    // 0 inputs, 0 outputs, 1 continuous state.
    this->DeclareContinuousState(1);
  }

 private:
  virtual void DoCalcVectorTimeDerivatives(
      const systems::Context<T>&, const Eigen::VectorBlock<const VectorX<T>>&,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const {
    (*derivatives)(0) = 100. * pow(state(0), 3.0) - 25. * state(0);
  }
};

/// Makes a cost equal to the volume of polynomial @p p over with respect to the
/// indeterminates over the domain defined by @p b.  For polynomials, this is
/// equivalent to the product of `coeff(p) * l`, where `coeff(p)` is the vector
/// of coefficients of `p` and `l` is the corresponding vector of moments of the
/// Lebesgue measure over the domain.
Expression MakeVolumetricCost(const Polynomial& p, const BoundingBox& b) {
  DRAKE_DEMAND(b.first.size() == b.second.size());
  DRAKE_DEMAND(p.indeterminates().size() == b.first.size());

  // Build a moment vector.
  const MapType& map = p.monomial_to_coefficient_map();
  Expression result;
  const Variables& indeterminates = p.indeterminates();
  for (const auto& monomial : map) {
    const std::map<Variable, int>& powers = monomial.first.get_powers();
    Expression moment{1.};
    for (const auto& var : indeterminates) {
      if (powers.find(var) == powers.end()) {
        moment *= b.second.at(var) - b.first.at(var);  // zero exponent case.
        continue;
      }
      moment *= (pow(b.second.at(var), powers.at(var) + 1) -
                 pow(b.first.at(var), powers.at(var) + 1)) /
                (powers.at(var) + 1);
    }
    result += moment * monomial.second;
  }
  return result;
}

/// Solves the backward reachable set over T seconds on the above univariate
/// example, given a terminal set.  Specifically, we solve the dual of the
/// polynomial relaxation to an infinite-dimensional LP over the space of
/// occupation measures capturing the reachable set.  By restricting the space
/// of functions to polynomials and representing measures by their moments, we
/// solve the dual relaxation over the space of nonnegative functions in order
/// to find a tight over-approximation to the reachable set.
///
/// Given the target set {x | ‖x(1)‖ ≤ 0.1}, the cubic polynomial system
/// ẋ = 100x(x - 0.5)(x + 0.5) is known to have a backward reachable set
/// B = {x | ‖x(0)‖ < 0.5}.
void ComputeBackwardReachableSet() {
  // Construct the system.
  CubicSystem<Expression> system;
  auto context = system.CreateDefaultContext();
  auto derivatives = system.AllocateTimeDerivatives();

  // Set up the optimization problem and declare the indeterminates.
  solvers::MathematicalProgram prog;
  const VectorX<Variable> tvec{prog.NewIndeterminates<1>("t")};
  const VectorX<Variable> xvec{prog.NewIndeterminates<1>("x")};
  const Variable& t = tvec(0);
  const Variable& x = xvec(0);

  // The desired order of the polynomial function approximations (noting that
  // the SOS multipliers, constructed below, have order d-2).
  const int d = 8;

  // Extract the polynomial dynamics.
  context->get_mutable_continuous_state_vector().SetAtIndex(0, x);
  system.CalcTimeDerivatives(*context, derivatives.get());

  // Define the domain bounds on t: 0 ≤ t ≤ T.
  const double T = 1.;
  const Polynomial gt = Polynomial{t * (T - t)};

  // Define the (symmetric) domain bounds on x: ‖x‖ ≤ x_bound
  // (X = {x | gx ≥ 0}).
  const double x_bound = 1.;
  const Polynomial gx = Polynomial{-(x - x_bound) * (x + x_bound)};

  // Define the terminal set: ‖x(T)‖ ≤ 0.1 (X_T = {x | gxT ≥ 0}).
  const Polynomial gxT{-(x - 0.1) * (x + 0.1)};

  // Define the ground-truth backward reachable set (for plotting only):
  // ‖x(0)‖ < 0.5 (a.k.a. X₀).
  const Polynomial gx0{-(x - 0.5) * (x + 0.5)};

  // Set up and solve the following optimization problem:
  // Inf  w'l  over v, w, qₓ, qt, qT, q₀, sₓ
  // s.t. ℒv ≤ 0 on [0, T] × X  (see below and Henrion and Korda 2014)
  //      v ≥ 0 on {T} × X_T
  //      w ≥ v + 1 on {0} × X
  //      w ≥ 0 on X
  //
  // The result is the T-horizon outer-approximated backward reachable set,
  //    B = X₀ = {x ∈ X | w ≥ 1}.
  const Polynomial v{prog.NewFreePolynomial({t, x}, d)};
  const Polynomial w{prog.NewFreePolynomial({x}, d)};

  // ℒ operator, defined as v ↦ ℒv := ∂v/∂t + Σ_{i=1,…,n} ∂v/∂x fᵢ(t,x).
  const Polynomial Lv{v.Jacobian(tvec).coeff(0) +
                      v.Jacobian(xvec).coeff(0) *
                          Polynomial((*derivatives)[0])};

  // v decreases along trajectories over t ∈ [0, T], x ∈ X.
  const Polynomial qx{prog.NewSosPolynomial({t, x}, d - 2).first};
  const Polynomial qt{prog.NewSosPolynomial({t, x}, d - 2).first};
  prog.AddSosConstraint(-Lv - qx * gx - qt * gt);

  // v(t=T, x) is SOS for all x ∈ X_T.  Note that this constraint, coupled with
  // the one above, implies that v(t=0, x) ≥ 0 for all x ∈ B.
  // Notice that, by replacing v_at_T with v, we may solve the "free final-time"
  // problem.
  const Polynomial v_at_T = v.EvaluatePartial({{t, T}});
  const Polynomial qT{prog.NewSosPolynomial({x}, d - 2).first};
  prog.AddSosConstraint(v_at_T - qT * gxT);

  // w is an outer-approximation to B over x ∈ X.
  const Polynomial v_at_0 = v.EvaluatePartial({{t, 0.}});
  const Polynomial q0{prog.NewSosPolynomial({x}, d - 2).first};
  prog.AddSosConstraint(w - v_at_0 - 1. - q0 * gx);

  // w is SOS for all x ∈ X.
  const Polynomial sx{prog.NewSosPolynomial({x}, d - 2).first};
  prog.AddSosConstraint(w - sx * gx);

  // w represents a *tight* outer-approximation.
  prog.AddCost(MakeVolumetricCost(
      w, std::make_pair(Bound{{x, -x_bound}}, Bound{{x, x_bound}})));

  cout << " Program attributes: " << endl;
  cout << "    number of decision variables: " << prog.num_vars() << endl;
  cout << "    number of PSD constraints: "
       << prog.positive_semidefinite_constraints().size() << endl;

  const solvers::MathematicalProgramResult result = Solve(prog);
  DRAKE_DEMAND(result.get_solver_id() == solvers::MosekSolver::id());
  DRAKE_DEMAND(result.is_success());

  // Print the solution (if one is found).
  cout << " Solution found with optimal cost: " << result.get_optimal_cost()
       << endl;

  Environment w_env;
  for (const auto& var : w.decision_variables()) {
    w_env.insert(var, result.GetSolution(var));
  }
  const Polynomial w_sol{w.EvaluatePartial(w_env)};

  // Serialize the result and generate Python plots.  In particular, plot the
  // true indicator function of B and w (its polynomial outer-approximation).
  // Execute:
  //    > bazel run //examples/cubic_polynomial:backward_reachability -c dbg
  //         --config mosek
  //    > bazel run //common/proto:call_python_client_cli
  const int N{1000};
  Eigen::VectorXd x_val(N), w_val(N), v_val(N), ground_val(N);
  for (int i = 0; i < N; i++) {
    x_val[i] = x_bound * (2. * i / N - 1.);
    w_val[i] = w_sol.Evaluate({{x, x_val[i]}});
    ground_val[i] = (gx0.Evaluate({{x, x_val[i]}}) >= 0.) ? 1. : 0.;
  }
  CallPython("figure", 1);
  CallPython("clf");
  CallPython("plot", x_val, w_val);
  CallPython("setvars", "x_val", x_val, "w_val", w_val);
  CallPython("plot", x_val, ground_val);
  CallPython("setvars", "x_val", x_val, "ground_val", ground_val);
  CallPython("plt.xlabel", "x");
  CallPython("plt.ylabel", "w, I_B");
}

}  // namespace
}  // namespace drake

int main() {
  drake::ComputeBackwardReachableSet();
  return 0;
}
