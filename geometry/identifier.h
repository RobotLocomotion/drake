#pragma once

// NOLINTNEXTLINE(whitespace/line_length)
#warning The drake/geometry/identifier.h header is deprecated and will be removed on 2020-05-01.  Use drake/common/identifier.h instead.

#include "drake/common/drake_deprecated.h"
#include "drake/common/identifier.h"

namespace drake {
namespace geometry {

template <class Tag>
using Identifier
    DRAKE_DEPRECATED("2020-05-01", "Use drake::common::Identifier instead.")
    = drake::Identifier<Tag>;

}  // namespace geometry
}  // namespace drake
