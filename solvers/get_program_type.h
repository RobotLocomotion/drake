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
ProgramType GetProgramType(const MathematicalProgram& prog);
}  // namespace solvers
}  // namespace drake
