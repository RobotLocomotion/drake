#include "drake/multibody/fixed_fem/dev/test/test_utilities.h"

#include <algorithm>
#include <limits>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
namespace test {

template <typename T>
double CalcConditionNumber(const Eigen::Ref<const MatrixX<T>>& A) {
  Eigen::JacobiSVD<MatrixX<T>> svd(A);
  const VectorX<T>& sigma = svd.singularValues();
  /* Prevents division by zero for singular matrix. */
  const T epsilon = std::numeric_limits<T>::epsilon();
  const T cond = sigma(0) / std::max(sigma(sigma.size() - 1), epsilon);
  return ExtractDoubleOrThrow(cond);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcConditionNumber<T>))

}  // namespace test
}  // namespace fem
}  // namespace multibody
}  // namespace drake
