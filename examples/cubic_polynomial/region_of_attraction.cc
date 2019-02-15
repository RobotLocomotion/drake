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
#include "drake/solvers/solve.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {

using std::cout;
using std::endl;

using symbolic::Expression;
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;

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
  const VectorX<Variable> xvar{prog.NewIndeterminates<1>("x")};
  const VectorX<Expression> x = xvar.cast<Expression>();

  // Extract the polynomial dynamics.
  context->get_mutable_continuous_state_vector().SetFromVector(x);
  system.CalcTimeDerivatives(*context, derivatives.get());

  // Define the Lyapunov function.
  const Expression V = x.dot(x);

  const Expression Vdot = (Jacobian(Vector1<Expression>(V), xvar) *
                         derivatives->CopyToVector())[0];

  // Maximize ρ s.t. V̇ ≥ 0 ∧ x ≠ 0 ⇒ V ≥ ρ,
  // implemented as (V(x) - ρ)x² - λ(x)V̇(x) is SOS;
  //                 λ(x) is SOS.
  const Variable rho{prog.NewContinuousVariables<1>("rho").coeff(0)};
  const Expression lambda{
      prog.NewSosPolynomial(Variables(xvar), 4).first.ToExpression()};

  prog.AddSosConstraint((V - rho) * x.dot(x) - lambda * Vdot);
  prog.AddLinearCost(-rho);
  const solvers::MathematicalProgramResult result = Solve(prog);
  DRAKE_DEMAND(result.is_success());

  cout << "Verified that " << V << " < " << result.GetSolution(rho)
       << " is in the region of attraction." << endl;

  // Check that ρ ≃ 1.0.
  DRAKE_DEMAND(std::abs(result.GetSolution(rho) - 1.0) < 1e-6);
}
}  // namespace drake

int main() {
  drake::ComputeRegionOfAttraction();
  return 0;
}
