#include <functional>
#include <memory>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

namespace {

// Replace all the psd constraints of prog using one of
// prog->TightenPSDConstraintToDDConstraint,
// prog->TightenPSDConstraintToSDDConstraint,
// prog->RelaxPSDConstraintToDDDualConeConstraint,
// prog->RelaxPSDConstraintToSDDDualConeConstraint.
// Expect that prog and the implicit prog in psd_constraint_replacing_function are the same.
template <typename T>
void ApproximateProgram(
    const std::function<T(const Binding<PositiveSemidefiniteConstraint>&)>&
        psd_constraint_replacing_function,
    MathematicalProgram* prog) {
  for (const auto& psd_constraint : prog->positive_semidefinite_constraints()) {
    psd_constraint_replacing_function(psd_constraint);
  }
}
}  // namespace

std::unique_ptr<MathematicalProgram> MakeDiagonallyDominantInnerApproximation(
    std::unique_ptr<MathematicalProgram> prog){
  ApproximateProgram<MatrixX<symbolic::Expression>>(prog.get()->TightenPSDConstraintToDDConstraint, prog.get());
}

//std::unique_ptr<MathematicalProgram>
//MakeScaledDiagonallyDominantInnerApproximation(std::unique_ptr<MathematicalProgram> prog){
//  ApproximateProgram(prog.get()->TightenPSDConstraintToSDDConstraint, prog.get());
//};
//
//std::unique_ptr<MathematicalProgram>
//MakeDiagonallyDominantDualConeOuterApproximation(
//    std::unique_ptr<MathematicalProgram> prog){
//  ApproximateProgram(prog.get()->RelaxPSDConstraintToDDDualConeConstraint, prog.get());
//}
//
//std::unique_ptr<MathematicalProgram>
//MakeScaledDiagonallyDominantDualConeOuterApproximation(
//    std::unique_ptr<MathematicalProgram> prog){
//  ApproximateProgram(prog.get()->RelaxPSDConstraintToSDDDualConeConstraint, prog.get());
//}

}  // namespace solvers
}  // namespace drake
