#pragma once

#include <limits>
#include <sstream>
#include <string_view>

#include <Eigen/Core>

#include "drake/common/fmt.h"

namespace drake {
namespace internal {

/* A tag type to be used in fmt::format("{}", fmt_eigen(...)) calls.
Below we'll add a fmt::formatter<> specialization for this tag. */
template <typename Derived>
struct fmt_eigen_ref {
  const Eigen::MatrixBase<Derived>& matrix;
};

}  // namespace internal

/** When passing an Eigen::Matrix to fmt, use this wrapper function to instruct
fmt to use Drake's custom formatter for Eigen types.

@warning The return value of this function should only ever be used as a
temporary object, i.e., in a fmt argument list or a logging statement argument
list. Never store it as a local variable, member field, etc. */
template <typename Derived>
internal::fmt_eigen_ref<Derived> fmt_eigen(
    const Eigen::MatrixBase<Derived>& matrix) {
  return {matrix};
}

}  // namespace drake

// Formatter specialization for drake::fmt_eigen.
// TODO(jwnimmer-tri) Write our own formatting logic instead of using Eigen IO,
// and add customization flags for how to display the matrix data.
namespace fmt {
template <typename Derived>
struct formatter<drake::internal::fmt_eigen_ref<Derived>>
    : formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const drake::internal::fmt_eigen_ref<Derived>& ref,
              // NOLINTNEXTLINE(runtime/references) To match fmt API.
              FormatContext& ctx) DRAKE_FMT8_CONST -> decltype(ctx.out()) {
    const auto& matrix = ref.matrix;
    std::stringstream stream;
    // We'll print our matrix data using as much precision as we can, so that
    // console log output and/or error messages paint the full picture. Sadly,
    // the ostream family of floating-point formatters doesn't know how to do
    // "shortest round-trip precision". If we set the precision to max_digits,
    // then simple numbers like "1.1" print as "1.1000000000000001"; instead,
    // well use max_digits - 1 to avoid that problem, with the risk of losing
    // the last ulps in the printout it case it does matter. This will all be
    // fixed once we stop using Eigen IO.
    stream.precision(std::numeric_limits<double>::max_digits10 - 1);
    stream << matrix;
    return formatter<std::string_view>{}.format(stream.str(), ctx);
  }
};
}  // namespace fmt
