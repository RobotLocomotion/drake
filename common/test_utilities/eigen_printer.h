#pragma once

#include <string>
#include <type_traits>

#include "drake/common/fmt_eigen.h"

namespace drake {
namespace internal {

/* Defines a googletest printer for Eigen matrices.
We hack this option into our tools/workspace/gtest/package.BUILD.bazel file. */
struct EigenPrinter {
  template <typename Derived>
  static constexpr bool is_eigen_matrix =
      std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>;

  // SFINAE on whether the type is an Eigen::Matrix of some kind.
  // If not, gtest has a suite of other kinds of printers is will fall back on.
  template <typename T, typename = std::enable_if_t<is_eigen_matrix<T>>>
  static void PrintValue(const T& value, std::ostream* os) {
    // If the printed representation has newlines, start it on a fresh line.
    std::string printed = fmt::to_string(fmt_eigen(value));
    for (const char ch : printed) {
      if (ch == '\n') {
        *os << '\n';
        break;
      }
    }
    *os << printed;
  }
};

}  // namespace internal
}  // namespace drake
