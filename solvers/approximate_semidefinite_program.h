#pragma once

#include <memory>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

/** Provides an inner approximation of the MathematicalProgram @p prog by
 * replacing all positive semidefinite constraints with positive diagonally
 * dominant constraints. Note that we do NOT tighten SOCP and QP constraints to
 * linear constraints with this method, even though they are semidefinite
 * representable.
 *
 * @param[in/out] prog We mutate the mathematical program in place, replacing
 * all the positive semidefinite constraints with positive diagonally dominant
 * constraints. If you would like to avoid mutating the original program,
 * consider cloning it using prog.Clone().
 */
void MakeDiagonallyDominantInnerApproximation(std::unique_ptr<MathematicalProgram> prog);

/** Provides an inner approximation of the MathematicalProgram @p prog by
 * replacing all positive semidefinite constraints with scaled positive
 * diagonally dominant constraints.
 *
 * @param[in/out] prog We mutate the mathematical program in place, replacing
 * all the positive semidefinite constraints with scaled diagonally dominant
 * constraints. If you would like to avoid mutating the original program,
 * consider cloning it using prog.Clone().
 */
void MakeScaledDiagonallyDominantInnerApproximation(std::unique_ptr<MathematicalProgram> prog);

/** Provides an outer approximation of the MathematicalProgram @p prog by
 * replacing all positive semidefinite constraints with positive diagonally
 * dominant dual cone constraints. Note that we do NOT relax SOCP and QP
 * constraints to linear constraints with this method, even though they are
 * semidefinite representable.
 *
 * @param[in/out] prog We mutate the mathematical program in place, replacing
 * all the positive semidefinite constraints with positive diagonally dominant
 * dual cone constraints. If you would like to avoid mutating the original
 * program, consider cloning it using prog.Clone().
 */
void MakeDiagonallyDominantDualConeOuterApproximation(
    std::unique_ptr<MathematicalProgram> prog);

/** Provides an outer approximation of the MathematicalProgram @p prog by
 * replacing all positive semidefinite constraints with scaled positive
 * diagonally dominant dual cone constraints.
 *
 * @param[in/out] prog We mutate the mathematical program in place, replacing
 * all the positive semidefinite constraints with scaled diagonally dominant
 * dual cone constraints. If you would like to avoid mutating the original
 * program, consider cloning it using prog.Clone().
 */
void MakeScaledDiagonallyDominantDualConeOuterApproximation(
    std::unique_ptr<MathematicalProgram> prog);

}  // namespace solvers
}  // namespace drake
