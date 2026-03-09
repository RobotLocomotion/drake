// TODO(jwnimmer-tri) Write our own formatting logic instead of using Eigen IO,
// and add customization flags for how to display the matrix data.
#undef EIGEN_NO_IO
#include "drake/common/fmt_eigen.h"

#include <limits>
#include <sstream>

namespace drake {
namespace internal {

template <typename T>
using FormatterEigenRef =
    Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>;

template <typename Scalar>
std::string FormatEigenMatrix(const FormatterEigenRef<Scalar>& matrix)
  requires std::is_same_v<Scalar, double> || std::is_same_v<Scalar, float> ||
           std::is_same_v<Scalar, std::string>
{  // NOLINT(whitespace/braces)
  std::stringstream stream;
  // We'll print our matrix data using as much precision as we can, so that
  // console log output and/or error messages paint the full picture. Sadly,
  // the ostream family of floating-point formatters doesn't know how to do
  // "shortest round-trip precision". If we set the precision to max_digits,
  // then simple numbers like "1.1" print as "1.1000000000000001"; instead,
  // we'll use max_digits - 1 to avoid that problem, with the risk of losing
  // the last ulps in the printout in case we needed that last digit. This
  // will all be fixed once we stop using Eigen IO.
  stream.precision(std::numeric_limits<double>::max_digits10 - 1);
  stream << matrix;
  return stream.str();
}

template <typename Scalar>
std::string StringifyErrorDetailValue(const fmt_eigen_ref<Scalar>& value)
  requires is_fmt_eigen_drake_throw_scalar<Scalar>::value
{
  return fmt::to_string(value);
}

// Explicitly instantiate for the allowed scalar types in our header.
template std::string FormatEigenMatrix<double>(
    const FormatterEigenRef<double>& matrix);
template std::string FormatEigenMatrix<float>(
    const FormatterEigenRef<float>& matrix);
template std::string FormatEigenMatrix<std::string>(
    const FormatterEigenRef<std::string>& matrix);
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
