
// A simple example of computing a region of attraction for a (polynomial) dynamical system.

// TODO(russt): Provide an additional python-only implementation of this example.

#include "drake/common/unused.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {

// Cubic Polynomial System
//   xdot = -x + x^3
//   y = x
template<typename T>
class CubicPolynomialSystem : public systems::VectorSystem<T> {
public:
  CubicPolynomialSystem()
      : systems::VectorSystem<T>(0,0) { // Zero inputs, zero outputs.
    this->DeclareContinuousState(1);               // One state variable.
  }

private:
  // xdot = -x + x^3
  virtual void DoCalcVectorTimeDerivatives(
      const systems::Context<T> &context,
      const Eigen::VectorBlock<const VectorX<T>> &input,
      const Eigen::VectorBlock<const VectorX<T>> &state,
      Eigen::VectorBlock<VectorX<T>> *derivatives) const {
    unused(context, input);
    using std::pow;
    (*derivatives)(0) = -state(0) + pow(state(0), 3.0);
  }
};

void ComputeRegionOfAttraction() {
  // Create the simple system.
  CubicPolynomialSystem<symbolic::Expression> system;
  auto context = system.CreateDefaultContext();
  auto derivatives = system.AllocateTimeDerivatives();

  // Setup the optimization problem.
  solvers::MathematicalProgram prog;
  const VectorX<symbolic::Variable> xvec = prog.NewIndeterminates<1>("x");
  const symbolic::Variable& x = xvec(0);

  // Extract the polynomial dynamics.
  context->get_mutable_continuous_state_vector()->SetAtIndex(0, x);
  system.CalcTimeDerivatives(*context, derivatives.get());

  // Define the Lyapunov function.
  const symbolic::Polynomial V(x*x);
  // TODO(soonho): Remove the explicit cast to Polynomial below.
  const symbolic::Polynomial Vdot = V.Jacobian(xvec).coeff(0) * symbolic::Polynomial((*derivatives)[0]);

  // Maximize rho s.t. Vdot > 0 && x!=0 => V >= rho,
  // implemented as (V(x) - rho)*x^2 - lambda(x)*Vdot(x) is SOS; lambda(x) is SOS.
  const auto& rho = prog.NewContinuousVariables<1>("rho").coeff(0);
  // TODO(soonho): Replace the below with
  //   const auto lambda = prog.NewContinuousVariables<4>("c").dot(symbolic::MonomialBasis({"x"},4))
  const VectorX<symbolic::Monomial> m(symbolic::MonomialBasis({x}, 3));
  const auto lambda = prog.NewContinuousVariables<4>("c").cast<symbolic::Monomial>().dot(m.cast<symbolic::Polynomial>());
  // TODO(soonho): Remove the explicit cast to Polynomial below.
  prog.AddSosConstraint((V-symbolic::Polynomial(rho))*symbolic::Polynomial(x*x) + lambda*Vdot);
  prog.AddSosConstraint(lambda);
  prog.AddCost(-rho);
  DRAKE_DEMAND(prog.Solve() == solvers::SolutionResult::kSolutionFound);

  std::cout << "Verified that " << V << " < " << prog.GetSolution(rho) << " is in the region of attraction." << std::endl;

  // Check that rho ~= 1.0.
  DRAKE_DEMAND(std::abs(prog.GetSolution(rho) - 1.0) < 1e-6);
}

}


int main() {
  drake::ComputeRegionOfAttraction();
  return 0;
}
