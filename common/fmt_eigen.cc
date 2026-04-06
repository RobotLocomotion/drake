#include "drake/common/fmt_eigen.h"

#include <algorithm>

#include <fmt/ranges.h>

namespace drake {
namespace internal {

std::string FormatMatrix(const Eigen::Index rows, const Eigen::Index cols,
                         const std::span<std::string_view> elements,
                         const FmtEigenSpecs& fmt_eigen_specs) {
  const bool plain = fmt_eigen_specs.plain;

  // Special case empty matrices.
  if (rows <= 0 || cols <= 0) {
    return plain ? "" : "[]";
  }

  // When not in plain mode, we need to special case 1-d vectors so that they
  // only have one pair of brackets (not two).  For compactness, column vectors
  // are formated as a row vector with a transpose symbol.
  if (!plain && (rows <= 1 || cols <= 1)) {
    return fmt::format("[{}]{}", fmt::join(elements, ", "),
                       (rows > 1) ? "ᵀ" : "");
  }

  // No more special cases needed, we can use the base case logic now.
  // The matrix is either 2-d, or 1-d in plain format.
  std::string result;
  auto maybe_add_decoration = [plain, &result](char ch) {
    if (!plain) {
      result.push_back(ch);
    }
  };
  const size_t max_element_size =
      std::ranges::max(elements, {}, &std::string_view::size).size();
  result.reserve(rows * cols * (max_element_size + 5));
  maybe_add_decoration('[');
  for (Eigen::Index row = 0; row < rows; ++row) {
    if (row > 0) {
      maybe_add_decoration(',');
      result.push_back('\n');
      maybe_add_decoration(' ');
    }
    maybe_add_decoration('[');
    const std::span<std::string_view> row_elements =
        elements.subspan(row * cols, cols);
    fmt::format_to(std::back_inserter(result), "{0:>{1}}",
                   fmt::join(row_elements, plain ? " " : ", "),
                   max_element_size);
    maybe_add_decoration(']');
  }
  maybe_add_decoration(']');
  return result;
}

template <typename Scalar>
std::string StringifyErrorDetailValue(const fmt_eigen_ref<Scalar>& value)
  requires is_fmt_eigen_drake_throw_scalar<Scalar>::value
{
  return fmt::to_string(value);
}

// Explicitly instantiate for the allowed scalar types in our header.
template std::string StringifyErrorDetailValue<double>(
    const fmt_eigen_ref<double>& value);
template std::string StringifyErrorDetailValue<float>(
    const fmt_eigen_ref<float>& value);
template std::string StringifyErrorDetailValue<int>(
    const fmt_eigen_ref<int>& value);
template std::string StringifyErrorDetailValue<std::string>(
    const fmt_eigen_ref<std::string>& value);

}  // namespace internal
}  // namespace drake
