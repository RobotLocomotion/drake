#pragma once

#include <experimental/optional>

#include "drake/common/drake_compat.h"

/// @file
/// Provides drake::optional as an alias for the appropriate implementation of
/// std::optional or std::experimental::optional on the given platform.

namespace drake {

template <typename T>
using optional = std::experimental::optional<T>;

}  // namespace drake
