#pragma once

// NOLINTNEXTLINE(whitespace/line_length)
#warning This header is deprecated; use drake/common/value.h instead, and change all uses from drake::systems::Value to drake::Value and drake::systems::AbstractValue to drake::AbstractValue.

// TODO(jwnimmer-tri) Remove this file (and its build rules) on 2019-06-01.

#include "drake/common/drake_deprecated.h"
#include "drake/common/value.h"

namespace drake {
namespace systems {

using AbstractValue
    DRAKE_DEPRECATED("2019-06-01",
        "Spell as drake::AbstractValue instead.")
    = drake::AbstractValue;

template <typename T>
using Value
    DRAKE_DEPRECATED("2019-06-01",
        "Spell as drake::Value instead.")
    = drake::Value<T>;

}  // namespace systems
}  // namespace drake
