#pragma once

#include <sstream>
#include <string_view>

#include <fmt/ostream.h>

#include "drake/common/fmt.h"

namespace drake {

// Compatibility shim for fmt::streamed.
#if FMT_VERSION >= 90000 || defined(DRAKE_DOXYGEN_CXX)
/** When using fmt >= 9, this is an alias for fmt::streamed.
When using fmt < 9, this uses a polyfill instead. */
template <typename T>
auto fmt_streamed(const T& ref) { return fmt::streamed(ref); }
#else  // FMT_VERSION
namespace internal {
template <typename T> struct streamed_ref { const T& ref; };
}  // namespace internal
template <typename T>
auto fmt_streamed(const T& ref) { return internal::streamed_ref(ref); }
#endif  // FMT_VERSION

// Compatibility shim for fmt::ostream_formatter.
#if FMT_VERSION >= 90000
/** When using fmt >= 9, this is an alias for fmt::ostream_formatter.
When using fmt < 9, this uses a polyfill instead. */
using fmt::ostream_formatter;
#else  // FMT_VERSION
struct ostream_formatter : fmt::formatter<std::string_view> {
  template <typename T, typename FormatContext>
  auto format(const T& value,
              // NOLINTNEXTLINE(runtime/references) To match fmt API.
              FormatContext& ctx) DRAKE_FMT8_CONST {
    std::ostringstream output;
    output << value;
    output.exceptions(std::ios_base::failbit | std::ios_base::badbit);
    return fmt::formatter<std::string_view>::format(output.str(), ctx);
  }
};
#endif  // FMT_VERSION

}  // namespace drake

// Formatter specialization for drake::fmt_streamed.
#if FMT_VERSION < 90000
namespace fmt {
template <typename T>
struct formatter<drake::internal::streamed_ref<T>> : drake::ostream_formatter {
  template <typename FormatContext>
  auto format(drake::internal::streamed_ref<T> tag,
              // NOLINTNEXTLINE(runtime/references) To match fmt API.
              FormatContext& ctx) DRAKE_FMT8_CONST {
    return ostream_formatter::format(tag.ref, ctx);
  }
};
}  // namespace fmt
#endif  // FMT_VERSION
