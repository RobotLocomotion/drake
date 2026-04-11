#pragma once

#include <fmt/ostream.h>

#include "drake/common/fmt.h"

namespace drake {

template <typename T>
[[deprecated(
    "\nDRAKE DEPRECATED: Use fmt::streamed instead.\n"
    "The deprecated code will be removed from Drake on or after 2026-07-01.")]]
auto fmt_streamed(const T& ref) {
  return fmt::streamed(ref);
}

using ostream_formatter
    [[deprecated("\nDRAKE DEPRECATED: Use fmt::ostream_formatter instead.\n"
                 "The deprecated code will be removed from Drake on or after "
                 "2026-07-01.")]]  // BR
    = fmt::ostream_formatter;

}  // namespace drake
