#pragma once

#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>

#include "drake/common/fmt.h"

namespace drake {
namespace internal {

/* A tag type to be used in fmt::format("{}", fmt_eigen(...)) calls.
Below we'll add a fmt::formatter<> specialization for this tag. */
template <typename Scalar>
struct fmt_eigen_ref {
  Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>
      matrix;
};

/* Returns the string formatting of the given matrix elements (i.e., padded by
whitespace and/or newlines into a tabular layout). The elements are provided
in row-major order.
@pre rows * cols == ssize(elements) */
std::string FormatMatrix(Eigen::Index rows, Eigen::Index cols,
                         std::span<std::string_view> elements);

/* Type trait for supporting the use of fmt_eigen(x) in DRAKE_THROW_UNLESS. This
is the set of scalar types that are supported (have been explicitly instantiated
in fmt_eigen.cc). The set is arbitrarily chosen as the scalar types tested in
fmt_eigen.cc. */
template <typename T>
using is_fmt_eigen_drake_throw_scalar =
    std::disjunction<std::is_same<std::remove_cvref_t<T>, double>,
                     std::is_same<std::remove_cvref_t<T>, float>,
                     std::is_same<std::remove_cvref_t<T>, int>,
                     std::is_same<std::remove_cvref_t<T>, std::string>>;

/* Provides _limited_ support for `fmt_eigen(x)` in
`DRAKE_THROW_UNLESS(condition, fmt_eigen(x))`. This is an overload of
`StringifyErrorDetailValue()` (as declared in drake_assert.h) for fmt_eigen_ref.

The support is considered limited in the supported scalar types (see
is_fmt_eigen_drake_throw_scalar). */
template <typename Scalar>
std::string StringifyErrorDetailValue(const fmt_eigen_ref<Scalar>& value)
  requires is_fmt_eigen_drake_throw_scalar<Scalar>::value;

}  // namespace internal

/** When passing an Eigen::Matrix to fmt, use this wrapper function to instruct
fmt to use Drake's custom formatter for Eigen types.

Within Drake, when formatting an Eigen matrix into a string you must wrap the
Eigen object as `fmt_eigen(M)`. This holds true whether it be for logging, error
messages, debugging, or etc.

For example:
@code
if (!CheckValid(M)) {
  throw std::logic_error(fmt::format("Invalid M = {}", fmt_eigen(M)));
}
@endcode

@warning The return value of this function should only ever be used as a
temporary object, i.e., in a fmt argument list or a logging statement argument
list. Never store it as a local variable, member field, etc.

@note To ensure floating-point data is formatted without losing any digits,
Drake's code is compiled using -DEIGEN_NO_IO, which enforces that nothing within
Drake is allowed to use Eigen's `operator<<`. Downstream code that calls into
Drake is not required to use that option; it is only enforced by Drake's build
system, not by Drake's headers. */
template <typename Derived>
internal::fmt_eigen_ref<typename Derived::Scalar> fmt_eigen(
    const Eigen::MatrixBase<Derived>& matrix) {
  return {matrix};
}

}  // namespace drake

#ifndef DRAKE_DOXYGEN_CXX
// Formatter specialization for drake::fmt_eigen.
namespace fmt {
template <typename Scalar>
struct formatter<drake::internal::fmt_eigen_ref<Scalar>>
    : formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const drake::internal::fmt_eigen_ref<Scalar>& ref,
              // NOLINTNEXTLINE(runtime/references) To match fmt API.
              FormatContext& ctx) const -> decltype(ctx.out()) {
    const auto& matrix = ref.matrix;
    // Format every matrix element in turn. We'll format them back-to-back into
    // the same buffer (while keeping track of where each one started), and then
    // in a second pass we'll slice up the buffer into string_views.
    formatter<Scalar> element_formatter;
    std::vector<char> element_buffer;            // Slab for formatted elements.
    std::vector<size_t> element_starts;          // Indices into element_buffer.
    element_buffer.reserve(matrix.size() * 20);  // An estimate (not precise).
    element_starts.reserve(matrix.size() + 1);   // Exact allocation (precise).
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
        element_starts.push_back(element_buffer.size());
        using OutputIt = std::back_insert_iterator<std::vector<char>>;
        using OutputContext = fmt::basic_format_context<OutputIt, char>;
        OutputContext element_ctx{OutputIt(element_buffer), {}};
        element_formatter.format(matrix(row, col), element_ctx);
      }
    }
    element_starts.push_back(element_buffer.size());
    std::vector<std::string_view> elements;
    elements.reserve(matrix.size());
    for (Eigen::Index i = 0; i < matrix.size(); ++i) {
      const size_t begin = element_starts[i];
      const size_t end = element_starts[i + 1];
      elements.push_back(std::string_view{&element_buffer[begin], end - begin});
    }
    // Combine the element strings with whitespace.
    const std::string content =
        drake::internal::FormatMatrix(matrix.rows(), matrix.cols(), elements);
    // Copy the content into the output buffer.
    return formatter<std::string_view>::format(content, ctx);
  }
};
}  // namespace fmt
#endif
