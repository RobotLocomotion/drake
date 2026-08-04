#include "drake/common/fmt_eigen.h"

#include <algorithm>

#include <fmt/ranges.h>

namespace drake {
namespace internal {
namespace {

// We need to know how much storage to reserve when formatting a fmt_eigen.
// Consider the following Matrix3d sample output:
//
// [[1.1, 2.2, 3.3],\n
//  [4.4, 5.5, 6.6],\n
//  [7.7, 8.8, 9.9]]
//
// Each formatted matrix element consumes 3 characters ("1.1"). If the element
// sizes are heterogeneous, we'll take the maximum size and left-pad the shorter
// elements with whitespace. Therefore, we can compute an exact tally for how
// much space individual elements need by multiplying the max size by the number
// of elements, to cover this portion of the output:
//
// [[███, ███, ███],\n
//  [███, ███, ███],\n
//  [███, ███, ███]]
//
// How do we account for the size needed for the decorations (brackets, commas,
// spaces, newlines)? We could try to compute it exactly, but the complexity
// isn't worth it. Instead, we'll use a worst-case per-element tax as an
// overestimate. The worst-case padding for any single element is the last
// element in a row: " ███],\n", so 1 character before and 4 after.
constexpr size_t kMaxDecorationSizePerElement = 5;

}  // namespace

std::string FormatMatrix(const Eigen::Index rows, const Eigen::Index cols,
                         const std::span<std::string_view> elements,
                         const FmtEigenSpecs& fmt_eigen_specs) {
  const bool brackets = fmt_eigen_specs.brackets;
  const bool commas = fmt_eigen_specs.commas;

  // Special case empty matrices.
  if (rows <= 0 || cols <= 0) {
    return brackets ? "[]" : "";
  }

  // When printing brackets, we need to special case 1-d vectors so that they
  // only have one pair of brackets (not two). For compactness, column vectors
  // are formatted as a row vector with a transpose symbol.
  if (brackets && (rows <= 1 || cols <= 1)) {
    return fmt::format("[{}]{}", fmt::join(elements, ", "),
                       (rows > 1) ? "ᵀ" : "");
  }

  // No more special cases needed, we can use the base case logic now.
  // The matrix is either 2-d, or 1-d in plain format.
  std::string result;
  auto add_if_brackets = [brackets, &result](char ch) {
    if (brackets) {
      result.push_back(ch);
    }
  };
  const size_t max_element_size =
      std::ranges::max(elements, {}, &std::string_view::size).size();
  result.reserve(rows * cols *
                 (max_element_size + kMaxDecorationSizePerElement));
  add_if_brackets('[');
  for (Eigen::Index row = 0; row < rows; ++row) {
    if (row > 0) {
      if (commas) {
        result.push_back(',');
      }
      result.push_back('\n');
      add_if_brackets(' ');
    }
    add_if_brackets('[');
    const std::span<std::string_view> row_elements =
        elements.subspan(row * cols, cols);
    fmt::format_to(std::back_inserter(result), "{0:>{1}}",
                   fmt::join(row_elements, commas ? ", " : " "),
                   max_element_size);
    add_if_brackets(']');
  }
  add_if_brackets(']');
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
