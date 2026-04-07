#include "drake/common/fmt_eigen.h"

#include <algorithm>

#include <fmt/ranges.h>

namespace drake {
namespace internal {

std::string FormatMatrix(const Eigen::Index rows, const Eigen::Index cols,
                         const std::span<std::string_view> elements) {
  if (rows <= 0 || cols <= 0) {
    return std::string{};
  }

  const size_t max_element_size =
      std::ranges::max(elements, {}, &std::string_view::size).size();
  std::string result;
  result.reserve(rows * cols * (max_element_size + 1));
  for (Eigen::Index row = 0; row < rows; ++row) {
    if (row > 0) {
      result.push_back('\n');
    }
    const std::span<std::string_view> row_elements =
        elements.subspan(row * cols, cols);
    fmt::format_to(std::back_inserter(result), "{0:>{1}}",
                   fmt::join(row_elements, " "), max_element_size);
  }
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
