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
  VectorX<T> sigma = svd.singularValues();
  /* Sort the singular values by its absolute value in descending order. */
  std::sort(sigma.data(), sigma.data() + sigma.size(),
            [](const T& a, const T& b) { return a * a > b * b; });
  /* Prevents division by zero for singular matrix. */
  const T epsilon = std::numeric_limits<T>::epsilon();
  const T cond = sigma(0) / std::max(sigma(sigma.size() - 1), epsilon);
  if constexpr (std::is_same_v<T, double>) {
    return std::abs(cond);
  } else {
    return std::abs(cond.value());
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcConditionNumber<T>))

}  // namespace test
}  // namespace fem
}  // namespace multibody
}  // namespace drake
