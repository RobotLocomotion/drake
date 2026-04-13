#pragma once

#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>

#include "drake/common/fmt.h"

namespace drake {

#ifndef DRAKE_DOXYGEN_CXX
// In our definition of fmt_eigen_ref below, we must use *exactly* this alias
// due to a Clang >= 16 bug (see drake#22061). Ideally, we would obtain the
// alias via `#include "drake/common/eigen_types.h"` where it's canonically
// defined, but we can't include that file due to a build dependency cycle,
// so we'll repeat the alias here.
template <typename Scalar>
using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
#endif

namespace internal {

/* A tag type to be used in fmt::format("{}", fmt_eigen(...)) calls.
Below we'll add a fmt::formatter<> specialization for this tag. */
template <typename Scalar>
struct fmt_eigen_ref {
  Eigen::Ref<const MatrixX<Scalar>> matrix;
};

/* Parsed format string specifiers for fmt_eigen. */
struct FmtEigenSpecs {
  bool brackets{true};  // Disabled via "n" or "#".
  bool commas{true};    // Disabled via "#".
  bool string{false};   // Enabled via "s" or "?s".
  bool debug{false};    // Enabled via "?s".
};

/* Returns the string formatting of the given matrix elements (i.e., padded by
whitespace and/or newlines into a tabular layout with brackets, commas, etc.).
The elements are provided in row-major order.
@pre rows * cols == ssize(elements) */
std::string FormatMatrix(Eigen::Index rows, Eigen::Index cols,
                         std::span<std::string_view> elements,
                         const FmtEigenSpecs& fmt_eigen_specs);

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
system, not by Drake's headers.

### Format string syntax

The format string specification syntax for fmt_eigen is based on fmtlib's
[range format](https://fmt.dev/dev/syntax/#range-format-specifications)
specification, recognizable by the distinctive double colon. We offer one
additional option `"#"` that uses the "alternate form", which in this case means
without any extra characters except whitespace (i.e., no brackets, no commas, no
transpose).

```txt
eigen_format_spec ::= ["s" | "?s" | [["n" | "#"][":" range_underlying_spec]]]
```

- "s" is only available when the `Scalar` is `char`; the Matrix contents are
  formatted as a quoted string, with the characters taken in row-major order.

- "?s" is only available when the `Scalar` is `char`; the Matrix contents are
  formatted as a debug string (i.e., quoted and escaped), with the characters
  taken in row-major order.

- "n" turns off brackets (but leaves commas enabled);

- "#" turns off both brackets and commas, similar to Eigen's default IOFormat.

The "range_underlying_spec" format string depends on the particular Scalar type
captured in the fmt_eigen instance.

Examples:

```
Eigen::Vector3d x{M_PI, M_SQRT2, M_E};
fmt::format("{}", fmt_eigen(x));
// [3.141592653589793, 1.4142135623730951, 2.718281828459045]ᵀ
// (By default, a column-vector is printed as a transposed row-vector.)

fmt::format("{::.2f}", fmt_eigen(x));
// "[3.14, 1.41, 2.72]ᵀ"

fmt::format("{:n:.2f}", fmt_eigen(x.transpose()));
// "3.14, 1.41, 2.72"
// (We can use transpose() to avoid the column-vector newlines.)

fmt::format("{:#:.3f}", fmt_eigen(x));
// "3.142\n1.414\n2.718"
// (Eigen's default IOFormat uses newlines for a column-vector.)

fmt::format("{x::e}", fmt::arg("x", fmt_eigen(x)));
// "[3.141593e+00, 1.414214e+00, 2.718282e+00]ᵀ"
```

Refer to https://fmt.dev/ for syntax details, but in short:

- The `arg_id` appears before the first colon, and specifies which argument
  should be formatted. This syntax is part of fmt, not specific to Drake.
  In the above examples we mostly leave it blank, but in the last one we give
  the argument the name `"x"` using `fmt::arg` and then use that name in the
  format string.

- The fmt_eigen format spec appears between the first and second colon. When
  empty, the normal presentation is used (with brackets and commas). When
  `'#'` (like in the final two examples), the brackets and commas are omitted.

- The floating-point format spec appears after the second colon. This syntax is
  part of fmt, not specific to Drake. As seen in the examples, it can be used to
  change the precision or use scientific notation, etc.

@remark To format a 2-d Eigen::Matrix as a 1-d Eigen::Vector, use
`fmt_eigen(M.reshaped(1, M.size()))`. */
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
struct formatter<drake::internal::fmt_eigen_ref<Scalar>> : formatter<Scalar> {
  constexpr auto parse(fmt::format_parse_context& ctx)
      -> decltype(ctx.begin()) {
    auto iter = ctx.begin();
    auto end = ctx.end();
    // The options "n", "#", "s", "?s" are mutually exclusive.
    // The options "s" and "?"s are only valid when Scalar is char.
    if (iter != end && *iter == 'n') {
      ++iter;
      fmt_eigen_specs.brackets = false;
    } else if (iter != end && *iter == '#') {
      ++iter;
      fmt_eigen_specs.brackets = false;
      fmt_eigen_specs.commas = false;
    } else if (iter != end && *iter == 's') {
      if constexpr (std::is_same_v<Scalar, char>) {
        ++iter;
        fmt_eigen_specs.string = true;
      }
      return iter;
    } else if (iter != end && *iter == '?') {
      ++iter;
      if (iter != end && *iter == 's') {
        if constexpr (std::is_same_v<Scalar, char>) {
          ++iter;
          fmt_eigen_specs.string = true;
          fmt_eigen_specs.debug = true;
        }
      } else {
        throw fmt::format_error("Malformed fmt_eigen format specification");
      }
      return iter;
    }
    if (iter == end || *iter == '}') {
      return iter;
    }
    if (*iter == ':') {
      ++iter;
      ctx.advance_to(iter);
      return formatter<Scalar>::parse(ctx);
    }
    throw fmt::format_error("Malformed fmt_eigen format specification");
  }

  template <typename FormatContext>
  auto format(const drake::internal::fmt_eigen_ref<Scalar>& ref,
              // NOLINTNEXTLINE(runtime/references) To match fmt API.
              FormatContext& ctx) const -> decltype(ctx.out()) {
    const auto& matrix = ref.matrix;
    // Special handling for strings.
    if constexpr (std::is_same_v<Scalar, char>) {
      if (fmt_eigen_specs.string) {
        const bool add_quotes = !fmt_eigen_specs.debug;
        std::string elements;
        elements.reserve(matrix.size() + (add_quotes ? 2 : 0));
        if (add_quotes) {
          elements.push_back('"');
        }
        for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
          for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
            elements.push_back(matrix(row, col));
          }
        }
        if (add_quotes) {
          elements.push_back('"');
        }
        formatter<std::string_view> string_formatter;
        if (fmt_eigen_specs.debug) {
          string_formatter.set_debug_format();
        }
        return string_formatter.format(elements, ctx);
      }
    }
    // Format every matrix element in turn. We'll format them back-to-back into
    // the same buffer (while keeping track of where each one started), and then
    // in a second pass we'll slice up the buffer into string_views.
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
        // Use our base class Scalar formatter so its format_spec will be used.
        formatter<Scalar>::format(matrix(row, col), element_ctx);
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
    // Decorate the element strings with brackets, commas, whitespace, etc.
    const std::string content = drake::internal::FormatMatrix(
        matrix.rows(), matrix.cols(), elements, fmt_eigen_specs);
    // Copy the content into the output buffer.
    return formatter<std::string_view>{}.format(content, ctx);
  }

 private:
  drake::internal::FmtEigenSpecs fmt_eigen_specs;
};
}  // namespace fmt
#endif
