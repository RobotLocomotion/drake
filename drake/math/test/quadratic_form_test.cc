#include "drake/math/quadratic_form.h"

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace math {
namespace {
GTEST_TEST(TestDecomposePositiveQuadraticForm, Test0) {
  // Decomposes a positive quadratic form without cross terms.
  Eigen::Matrix2d Q = 4 * Eigen::Matrix2d::Identity();
  Eigen::Vector2d b = Eigen::Vector2d::Zero();
  double c = 0;
  auto result = DecomposePositiveQuadraticForm(Q, b, c);
  std::cout << NiceTypeName::Get<decltype(std::get<0>(result))>() << std::endl;
  std::cout << NiceTypeName::Demangle(typeid(std::get<0>(result)).name()) << std::endl;
  std::cout << Eigen::ColMajor << std::endl;
  typedef decltype(std::get<0>(result)) returntype;
  static_assert(std::is_same<std::decay_t<returntype>,
                             Eigen::Matrix<double, -1, 2, 0, 2, 2>>::value,
                "return type is not correct");
}
}  // namespace
}  // namespace math
}  // namespace drake
