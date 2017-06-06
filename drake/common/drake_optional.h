#pragma once

#include "drake/common/drake_compat.h"
#include "drake/thirdParty/boost/tcbrindle_cpp17_headers/stx/optional.hpp"

/// @file
/// Provides drake::optional as an alias for the appropriate implementation of
/// std::optional or std::experimental::optional on the given platform.

namespace drake {

template <typename T>
using optional = stx::optional<T>;

constexpr auto nullopt = stx::nullopt;

}  // namespace drake
