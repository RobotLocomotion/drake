#pragma once

namespace drake {
namespace solvers {
class MathematicalProgram;
class MathematicalProgramResult;
}  // namespace solvers
namespace systems {
namespace analysis {
namespace internal {

// Throws if the solve failed, returned non-finite decision variables, or
// violated a constraint by more than the absolute certificate tolerance (1e-6).
// This is a numerical feasibility check, not a rigorous proof of nonnegativity.
void CheckRegionOfAttractionCertificate(
    const solvers::MathematicalProgram& prog,
    const solvers::MathematicalProgramResult& result);

}  // namespace internal
}  // namespace analysis
}  // namespace systems
}  // namespace drake
