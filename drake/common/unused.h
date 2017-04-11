#pragma once

namespace drake {

/// Documents the argument as unused.  This can be added within function bodies
/// to mark that certain parameters are unused.  (However, note that removing
/// the unused parameter is a superior solution, when possible.)
template <typename T>
void unused(const T&) {}

}
