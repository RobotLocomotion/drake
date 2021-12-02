#include "drake/multibody/tree/rotational_inertia.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace multibody {

// TODO(Mitiguy) Consider using this code (or code similar to this) to write
//  most/all Drake matrices and consolidate other usages to use this.
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
      out << ", ";
      if (width) out.width(width);
      out << I(i, j);
    }
    out << "]" << std::endl;
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
