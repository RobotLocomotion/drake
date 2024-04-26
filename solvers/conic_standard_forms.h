#pragma once

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

/** Given a convex program (specficially an LP, QP, SOCP, or SDP), construct an
 * equivalent program in standard primal conic form:
 * min        〈 c, x 〉
 * subject to  Ax + b ∈ K
 *             x ∈ K
 * Where K is the product of cones.
 * @param prog
 * @return
 */
std::unique_ptr<MathematicalProgram> ParseToConicStandardForm(
    const MathematicalProgram& prog);

std::unordered_map<Binding<L2NormCost>, symbolic::Variable>
ParseL2NormCostsToEpigraphForm(MathematicalProgram* prog);

std::unordered_map<Binding<L1NormCost>, symbolic::Variable>
ParseL1NormCostsToEpigraphForm(MathematicalProgram* prog);

std::unordered_map<Binding<QuadraticCost>, symbolic::Variable>
ParseQuadraticCostsToEpigraphForm(MathematicalProgram* prog);

/* Most convex solvers require only support linear and quadratic costs when
operating with nonlinear constraints. This removes costs and adds variables and
constraints as needed by the solvers. */
void ParseNonlinearCostsToEpigraphForm(MathematicalProgram* prog);

}  // namespace solvers
}  // namespace drake
