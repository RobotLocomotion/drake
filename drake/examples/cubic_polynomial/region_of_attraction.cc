// A simple example of computing a region of attraction for a (polynomial)
// dynamical system. Note that this is a C++ version of
// CubicPolynomialExample.m which is available at the following link:
//
// https://github.com/RobotLocomotion/drake/blob/00ec5a9836871d5a22579963d45376b5979e41d5/drake/examples/CubicPolynomialExample.m.
//
// TODO(russt): Provide an additional python-only implementation of this
// example.

#include <cmath>
#include <ostream>

#include "drake/common/symbolic.h"
#include "drake/common/unused.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {

using std::cout;
using std::endl;

using symbolic::Expression;
using symbolic::Polynomial;
using symbolic::Variable;

/// Cubic Polynomial System:
///   ẋ = -x + x³
///   y = x
template <typename T>
class CubicPolynomialSystem : public systems::VectorSystem<T> {
 public:
  CubicPolynomialSystem()
      : systems::VectorSystem<T>(0, 0) {  // Zero inputs, zero outputs.
    this->DeclareContinuousState(1);      // One state variable.
  }

 private:
  // ẋ = -x + x³
  virtual void DoCalcVectorTimeDerivatives(
      const systems::Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const {
    unused(context, input);
    using std::pow;
    (*derivatives)(0) = -state(0) + pow(state(0), 3.0);
  }
};

void ComputeRegionOfAttraction() {
  // Create the simple system.
  CubicPolynomialSystem<Expression> system;
  auto context = system.CreateDefaultContext();
  auto derivatives = system.AllocateTimeDerivatives();

  // Setup the optimization problem.
  solvers::MathematicalProgram prog;
  const VectorX<Variable> xvec{prog.NewIndeterminates<1>("x")};
  const Variable& x = xvec(0);

  // Extract the polynomial dynamics.
  context->get_mutable_continuous_state_vector().SetAtIndex(0, x);
  system.CalcTimeDerivatives(*context, derivatives.get());

  // Define the Lyapunov function.
  const Polynomial V{x * x};

  // TODO(soonho): Remove the explicit cast to Polynomial below.
  const Polynomial Vdot{V.Jacobian(xvec).coeff(0) *
                        Polynomial((*derivatives)[0])};

  // Maximize ρ s.t. V̇ ≥ 0 ∧ x ≠ 0 ⇒ V ≥ ρ,
  // implemented as (V(x) - ρ)x² - λ(x)V̇(x) is SOS;
  //                 λ(x) is SOS.
  const Variable rho{prog.NewContinuousVariables<1>("rho").coeff(0)};
  const Polynomial rho_poly{rho, {} /* no indeterminate */};

  const Polynomial lambda{prog.NewSosPolynomial({x}, 4).first};

  // TODO(soonho): Remove the explicit cast to Polynomial below.
  prog.AddSosConstraint((V - rho_poly) * Polynomial(x * x) - lambda * Vdot);
  prog.AddCost(-rho);
  const solvers::SolutionResult result{prog.Solve()};
  DRAKE_DEMAND(result == solvers::SolutionResult::kSolutionFound);

  cout << "Verified that " << V << " < " << prog.GetSolution(rho)
       << " is in the region of attraction." << endl;

  // Check that ρ ≃ 1.0.
  DRAKE_DEMAND(std::abs(prog.GetSolution(rho) - 1.0) < 1e-6);
}
}  // namespace drake

int main() {
  drake::ComputeRegionOfAttraction();
  return 0;
}
