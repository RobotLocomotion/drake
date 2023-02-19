#pragma once

#include <string>
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

/* Returns the string formatting of the given matrix.
@tparam T must be either double, float, or string */
template <typename T>
std::string FormatEigenMatrix(
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>&
        matrix);

}  // namespace internal

/** When passing an Eigen::Matrix to fmt, use this wrapper function to instruct
fmt to use Drake's custom formatter for Eigen types.

@warning This function should only ever be used as a temporary object, i.e., in
a fmt argument list or a logging statement argument list. Never store it as a
local variable, member field, etc. */
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
    using Scalar = typename Derived::Scalar;
    const auto& matrix = ref.matrix;
    if constexpr (std::is_same_v<Scalar, double> ||
                  std::is_same_v<Scalar, float>) {
      return formatter<std::string_view>{}.format(
          drake::internal::FormatEigenMatrix<Scalar>(matrix), ctx);
    } else {
      return formatter<std::string_view>{}.format(
          drake::internal::FormatEigenMatrix<std::string>(
              matrix.unaryExpr([](const auto& element) -> std::string {
                return fmt::to_string(element);
              })),
          ctx);
    }
  }
};
}  // namespace fmt
