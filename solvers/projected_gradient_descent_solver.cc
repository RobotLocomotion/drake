#include "drake/solvers/projected_gradient_descent_solver.h"

namespace drake {
namespace solvers {

using Eigen::VectorXd;

namespace {
constexpr const char kCustomGradientFunctionOptionsName[] =
    "CustomGradientFunction";
constexpr const char kCustomProjectionFunctionOptionsName[] =
    "CustomProjectionFunction";
constexpr const char kProjectionSolverInterfaceOptionName[] =
    "ProjectionSolverInterface";

struct KnownOptions {
  std::optional<std::function<VectorXd(const VectorXd&)>>
      custom_gradient_function{std::nullopt};
  std::optional<std::function<VectorXd(const VectorXd&)>>
      custom_projection_function{std::nullopt};
  SolverInterface* projection_solver_interface{nullptr};
};

// None of the current options can be serialized.
void Serialize(internal::SpecificOptions*, KnownOptions&) {}

KnownOptions ParseOptions(internal::SpecificOptions* options) {
  KnownOptions result;
  options->CopyToSerializableStruct(&result);
  return result;
}
}  // namespace

ProjectedGradientDescentSolver::ProjectedGradientDescentSolver()
    : SolverBase(id(), &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}
ProjectedGradientDescentSolver::~ProjectedGradientDescentSolver() = default;

void ProjectedGradientDescentSolver::DoSolve2(
    const MathematicalProgram& prog, const Eigen::VectorXd& initial_guess,
    internal::SpecificOptions* options,
    MathematicalProgramResult* result) const {
  const KnownOptions parsed_options = ParseOptions(options);

  // TODO(cohnt): implement this

  unused(prog);
  unused(initial_guess);
  unused(result);
}

std::string ProjectedGradientDescentSolver::CustomGradientFunctionOptionName() {
  return kCustomGradientFunctionOptionsName;
}

std::string
ProjectedGradientDescentSolver::CustomProjectionFunctionOptionName() {
  return kCustomProjectionFunctionOptionsName;
}

std::string
ProjectedGradientDescentSolver::ProjectionSolverInterfaceOptionName() {
  return kProjectionSolverInterfaceOptionName;
}

SolverId ProjectedGradientDescentSolver::id() {
  static const never_destroyed<SolverId> singleton{"ProjectedGradientDescent"};
  return singleton.access();
}

bool ProjectedGradientDescentSolver::is_available() {
  return true;
}

bool ProjectedGradientDescentSolver::is_enabled() {
  return true;
}

bool ProjectedGradientDescentSolver::ProgramAttributesSatisfied(
    const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kGenericConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kQuadraticConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kGenericCost, ProgramAttribute::kLinearCost,
          ProgramAttribute::kL2NormCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kCallback});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}

}  // namespace solvers
}  // namespace drake
