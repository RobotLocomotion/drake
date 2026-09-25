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
// violated a constraint by more than the given absolute certificate tolerance.
// This is a numerical feasibility check, not a rigorous proof of nonnegativity.
void CheckRegionOfAttractionCertificate(
    const solvers::MathematicalProgram& prog,
    const solvers::MathematicalProgramResult& result, double tolerance);

}  // namespace internal
}  // namespace analysis
}  // namespace systems
}  // namespace drake
