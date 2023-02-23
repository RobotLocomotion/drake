#pragma once

#include <type_traits>

#include <fmt/format.h>

namespace drake {
namespace internal {

/* Defines a googletest printer for values that provide fmt::format.
We hack this option into our tools/workspace/gtest/package.BUILD.bazel file. */
struct FmtFormatPrinter {
  // This is true iff the type provides fmt::formatter<T> directly, without
  // any conversions or fallback to operator<< streaming.
  template <typename T>
  static constexpr bool is_directly_formattable =
      std::is_constructible_v<fmt::formatter<T, char>>;

  // SFINAE on whether the type directly supports formatting.
  // If not, gtest has a suite of other kinds of printers is will fall back on.
  template <typename T, typename = std::enable_if_t<is_directly_formattable<T>>>
  static void PrintValue(const T& value, std::ostream* os) {
    *os << fmt::to_string(value);
  }
};

}  // namespace internal
}  // namespace drake
