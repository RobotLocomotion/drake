#include "drake/math/rotation_matrix.h"

#include <string>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/unused.h"

namespace drake {
namespace math {

template <typename T>
void RotationMatrix<T>::ThrowIfUnableToMakeUnitVectorDueToZeroVector(
    const Vector3<T>& u, const char* function_name) {
  if (u == Vector3<T>::Zero()) {
    std::string message = fmt::format(
        "RotationMatrix::{}():"
        " Unable to make a unit vector from a zero vector.", function_name);
    throw std::runtime_error(message);
  }
}

template <typename T>
void RotationMatrix<T>::ThrowIfUnableToMakeUnitVectorDueToNanVector(
    const Vector3<T>& u, const char* function_name) {
  if (!u.allFinite()) {
    std::string message = fmt::format(
        "RotationMatrix::{}():"
        " Unable to make a unit vector."
        " Vector contains an element that is infinity or Nan.",
        function_name);
    throw std::runtime_error(message);
  }
}

template <typename T>
void RotationMatrix<T>::ThrowIfNotValidUnitVector(const Vector3<T>& u,
    double tolerance, const char* function_name) {
  ThrowIfUnableToMakeUnitVectorDueToNanOrZeroVector(u, function_name);

  // Skip symbolic expressions.
  // TODO(Mitiguy) This is a generally-useful method.  Consider moving it to a
  //  into more general view in an appropriate file and also deal with symbolic
  //  expressions that can be easily evaluated to a number, e.g., consider:
  //  ThrowIfNotValidUnitVector(Vector3<symbolic::Expression> u_sym(3, 2, 1));
  if constexpr (scalar_predicate<T>::is_bool) {
    // Give a detailed message if |u| is not within tolerance of 1.
    using std::abs;
    const T u_norm_as_T = u.norm();
    const double u_norm = ExtractDoubleOrThrow(u_norm_as_T);
    const double abs_deviation = abs(1 - u_norm);
    if (abs_deviation > tolerance) {
      const double ux = ExtractDoubleOrThrow(u(0));
      const double uy = ExtractDoubleOrThrow(u(1));
      const double uz = ExtractDoubleOrThrow(u(2));
      std::string message = fmt::format(
          "RotationMatrix::{}(). Vector is not a unit vector."
          " The magnitude of vector [{:E} {:E} {:E}] deviates from 1."
          " The vector's actual magnitude is {:.16f}."
          " Its deviation from 1 is {:E}."
          " The allowable tolerance (deviation) is {:E}."
          " To normalize a vector u, consider using u.normalized().",
          function_name, ux, uy, uz, u_norm, abs_deviation, tolerance);
      throw std::logic_error(message);
    }
  } else {
    drake::unused(tolerance);
  }
}

}  // namespace math
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RotationMatrix)
