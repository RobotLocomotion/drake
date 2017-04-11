#pragma once

#include "drake/common/drake_compat.h"

namespace drake {

/// Documents the argument(s) as unused, thus suppressing GCC's -Wunused-param
/// warning.  This can be called within function bodies to mark that certain
/// parameters are unused.
///
/// When possible, removing the unused parameter is better than suppressing the
/// warning.  However, in some cases the dead parameter serves as documentation
/// (e.g., when the parameter is in a header file and forms part of a virtual
/// base API declaration), so we can't remove it.  In those cases, this
/// function is an appropriate work-around.
template <typename ... Args>
void unused(const Args& ...) {}

}  // namespace drake
