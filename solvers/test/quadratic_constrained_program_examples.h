#pragma once

#include <memory>
#include <optional>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace solvers {
namespace test {
// min x0 + x1
// s.t 4x0²+9x1² ≤ 1
void TestEllipsoid1(const SolverInterface& solver,
                    const std::optional<SolverOptions>& solver_options,
                    double tol);

// min x0 + x1
// s.t x0²+ x0*x1 + x1² ≤ 1
//     x0²+ 2*x0*x1 + 4x1² ≤ 9
//     x0 +x1 <= 0
void TestEllipsoid2(const SolverInterface& solver,
                    const std::optional<SolverOptions>& solver_options,
                    double tol);

}  // namespace test
}  // namespace solvers
}  // namespace drake
