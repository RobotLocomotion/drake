#pragma once

// For external users, please do not include this header file. It only exists so
// that we can expose the internals to csdp_solver_internal_test.cc

#include <vector>

namespace csdp {
extern "C" {
#include <declarations.h>
}  // extern C
}  // namespace csdp

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {
}  // namespace internal
}  // namespace solvers
}  // namespace drake
