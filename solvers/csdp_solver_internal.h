#pragma once

// For external users, please do not include this header file. It only exists so
// that we can expose the internals to csdp_solver_internal_test.cc

namespace csdp {
extern "C" {
// TODO(Jeremy.Nimmer): include this header as <csdp/declarations.h>
#include <declarations.h>
}  // extern C
}  // namespace csdp

