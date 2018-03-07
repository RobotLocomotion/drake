#include "drake/systems/analysis/lyapunov.h"

#include <string>

#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace systems {
namespace analysis {

using Eigen::VectorXd;
using symbolic::Expression;

namespace {

// Helper because AddLinearConstraint throws if I pass in something trivially
// true.
void AddLinearConstraintIfNonTrivial(const symbolic::Formula &f,
                                     solvers::MathematicalProgram *prog) {
  if (!symbolic::is_true(f)) {
    prog->AddLinearConstraint(f);
  }
}

}  // end namespace

Eigen::VectorXd SampleBasedLyapunovAnalysis(
    const System<double>& system, const Context<double>& context,
    const std::function<VectorX<AutoDiffXd>(const VectorX<AutoDiffXd>& state)>&
        basis_functions,
    const Eigen::Ref<const Eigen::MatrixXd>& state_samples,
    const Eigen::Ref<const Eigen::VectorXd>& V_zero_state) {
  const int state_size = state_samples.rows();
  const int num_samples = state_samples.cols();
  DRAKE_DEMAND(state_size > 0);
  DRAKE_DEMAND(num_samples > 0);
  DRAKE_DEMAND(V_zero_state.rows() == state_size);

  // TODO(russt): handle discrete state.
  DRAKE_DEMAND(context.has_only_continuous_state());
  DRAKE_DEMAND(context.get_continuous_state().size() == state_size);

  // TODO(russt): check that the system is time-invariant.

  solvers::MathematicalProgram prog;

  const VectorXd phi0 =
      math::autoDiffToValueMatrix(basis_functions(V_zero_state));
  const int num_parameters = phi0.size();
  DRAKE_DEMAND(num_parameters > 0);

  const solvers::VectorXDecisionVariable params =
      prog.NewContinuousVariables(num_parameters, "a");

  // Add an objective that drives Vdot ~= -1.
  // Note(russt): Tried the L2 norm version of this (which doesn't require
  // the slack variables), but it was significantly slower.
  // Note(russt): Also had a version of this that accepted an optional
  // MatrixXd of V(x₁) = 1 as a different way to set the boundary conditions.
  // But having this objective is much more generally useful, I think.
  //
  // Add slack variables s >= |Vdot + 1|.
  const solvers::VectorXDecisionVariable slack = prog.NewContinuousVariables
      (num_samples, "s");
  // Minimize ∑ sᵢ
  prog.AddLinearCost(VectorXd::Ones(num_samples), 0, slack);

  drake::log()->info("Building mathematical program.");

  // V(x₀) = 0.
  AddLinearConstraintIfNonTrivial(params.dot(phi0) == 0, &prog);

  Eigen::VectorXd state(state_size);
  VectorX<AutoDiffXd> autodiff_state(state_size);
  auto my_context = context.Clone();
  auto& context_state = my_context->get_mutable_continuous_state_vector();
  auto derivatives = system.AllocateTimeDerivatives();

  for (int si = 0; si < num_samples; si++) {
    state = state_samples.col(si);

    math::initializeAutoDiff(state, autodiff_state, state_size);
    const VectorX<AutoDiffXd> phi = basis_functions(autodiff_state);
    const Expression V = params.dot(math::autoDiffToValueMatrix(phi));

    context_state.SetFromVector(state);
    system.CalcTimeDerivatives(*my_context, derivatives.get());

    const Eigen::VectorXd phidot =
        math::autoDiffToGradientMatrix(phi) * derivatives->CopyToVector();
    const Expression Vdot = params.dot(phidot);

    // ∀xᵢ, V(xᵢ) ≥ 0
    AddLinearConstraintIfNonTrivial(V >= 0., &prog);

    // ∀xᵢ, V̇(xᵢ) = ∂V/∂x f(xᵢ) ≤ 0.
    AddLinearConstraintIfNonTrivial(Vdot <= 0., &prog);

    // ∀i, sᵢ ≥ |V̇(xᵢ) + 1|.
    prog.AddLinearConstraint(slack(si) >= Vdot + 1);
    prog.AddLinearConstraint(slack(si) >= -(Vdot + 1));
  }

  drake::log()->info("Solving program.");
  const solvers::SolutionResult result = prog.Solve();
  if (result != solvers::SolutionResult::kSolutionFound) {
    drake::log()->error("No solution found.  SolutionResult = " +
                        std::to_string(result));
  }
  drake::log()->info("Done solving program.");

  return prog.GetSolution(params);
}

}  // namespace analysis
}  // namespace systems
}  // namespace drake
