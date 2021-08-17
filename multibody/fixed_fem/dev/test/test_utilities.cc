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
  /* Eigen::JacobiSVD::singularValues() returns sigma as positive, monotonically
   decreasing values.
   See https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html. */
  const VectorX<T>& sigma = svd.singularValues();
  /* Prevents division by zero for singular matrix. */
  const T& sigma_min = sigma(sigma.size() - 1);
  DRAKE_DEMAND(sigma_min > 0);
  const T& sigma_max = sigma(0);
  const T cond = sigma_max / sigma_min;
  return ExtractDoubleOrThrow(cond);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcConditionNumber<T>))

}  // namespace test
}  // namespace fem
}  // namespace multibody
}  // namespace drake
