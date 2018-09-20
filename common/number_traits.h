#pragma once

#include "drake/common/drake_deprecated.h"

namespace drake {

/// This traits type is deprecated, and will be removed.
template <typename T>
struct is_numeric {
  // TODO(jwnimmer-tri) Delete number_traits.h on or about 2018-12-01.
  DRAKE_DEPRECATED("This trait will be removed")
  static constexpr bool value = true;
};

}  // namespace drake
