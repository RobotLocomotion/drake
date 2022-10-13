#pragma once

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/** Returns the type of the optimization program (LP, QP, etc), based on the
 * properties of its cost/constraints/variables.
 * Each mathematical program should be characterized by a unique type. If a
 * program can be characterized as either type A or type B (for example, a
 * program with linear constraint and linear costs can be characterized as
 * either an LP or an SDP), then we choose the type corresponding to a smaller
 * set of programs (LP in this case).
 */
[[nodiscard]] ProgramType GetProgramType(const MathematicalProgram& prog);
/**
 * Returns true if this program will invoke a convex solver.
 * The program must have all its constraints being convex, and every
 * individual cost being convex. Note that this is stronger than the program
 * being convex, which only requires convex constraints with the total cost
 * being convex. For example, if this program cost is in the form of cost1(x)
 * + cost2(x), where cost1(x) is non-convex while cost2(x) is convex, then
 * AccepctConvexSolver() will return false, even if the added cost cost1(x) +
 * cost2(x) is convex.
 */
[[nodiscard]] bool AcceptConvexSolver(const MathematicalProgram& prog);

}  // namespace solvers
}  // namespace drake
