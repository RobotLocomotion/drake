#include "drake/multibody/tree/rotational_inertia.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace multibody {

template <typename T>
void RotationalInertia<T>::ThrowNotPhysicallyValid(const char* func_name)
    const {
  std::string error_message = fmt::format(
      "{}(): The rotational inertia\n"
      "{}did not pass the test CouldBePhysicallyValid().",
      func_name, *this);
  // Provide additional information if a moment of inertia is non-negative
  // or if moments of inertia do not satisfy the triangle inequality.
  if constexpr (scalar_predicate<T>::is_bool) {
    if (!IsNaN()) {
      const Vector3<double> p = CalcPrincipalMomentsOfInertia();
      if (!AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality(
              p(0), p(1), p(2), /* epsilon = */ 0.0)) {
        error_message += fmt::format(
            "\nThe associated principal moments of inertia:"
            "\n{}  {}  {}", p(0), p(1), p(2));
        if (p(0) < 0 || p(1) < 0 || p(2) < 0) {
          error_message += "\nare invalid since at least one is negative.";
        } else {
          error_message += "\ndo not satisify the triangle inequality.";
        }
      }
    }
  }
  throw std::logic_error(error_message);
}

// TODO(Mitiguy) Consider using this code (or code similar to this) to write
//  most/all Drake matrices and consolidate other usages to use this.
// TODO(jwnimmer-tri) Obeying the formatting choices from `out` (via `copyfmt`
//  is a defect; we should always display full round-trip precision.  We should
//  not re-use this pattern elsewhere.
template <typename T>
std::ostream& operator<<(std::ostream& out, const RotationalInertia<T>& I) {
  int width = 0;
  // Computes largest width so that we can align columns for a prettier format.
  // Idea taken from: Eigen::internal::print_matrix() in Eigen/src/Core/IO.h
  for (int j = 0; j < I.cols(); ++j) {
    for (int i = 0; i < I.rows(); ++i) {
      std::stringstream sstr;
      sstr.copyfmt(out);
      sstr << I(i, j);
      width = std::max<int>(width, static_cast<int>(sstr.str().length()));
    }
  }

  // Outputs to stream.
  for (int i = 0; i < I.rows(); ++i) {
    out << "[";
    if (width) out.width(width);
    out << I(i, 0);
    for (int j = 1; j < I.cols(); ++j) {
      out << "  ";
      if (width) out.width(width);
      out << I(i, j);
    }
    out << "]\n";
  }
  return out;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<std::ostream&(*)(std::ostream&, const RotationalInertia<T>&)>(
        &operator<< )
))

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::RotationalInertia)
