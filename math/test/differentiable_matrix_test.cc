#include "drake/math/differentiable_matrix.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::AutoDiffScalar;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

namespace drake {
namespace math {
namespace {

GTEST_TEST(Matrix3dWithDerivatives, Misc) {
  // using Matrix34d = Eigen::Matrix<double, 3, 4>;
  // Matrix34d m34d;
  using Matrix34d = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  Matrix34d m34d(3,4);
  m34d <<  1,  2,  3,  4,
           5,  6,  7,  8,
           9, 10, 11, 12;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> grad;
  grad.resize(12, 2);
  for (int i=0; i < 12; ++i) {
    grad.row(i) = Eigen::Vector2d(m34d(i) * 10, m34d(i) * 10 + 1);
  }
  // Eigen::Matrix<AutoDiffXd, 3, 4> m34ad;
  Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, Eigen::Dynamic> m34ad;
  InitializeAutoDiff(m34d, grad, &m34ad);
  m34ad(1, 1).derivatives().resize(0);
  m34ad(2, 2).derivatives().resize(0);

  DifferentiableMatrix<Matrix34d, true> m34wd(m34ad);

  EXPECT_TRUE(CompareMatrices(ExtractValue(m34ad),
                              ExtractValue(m34wd.ToAutoDiffXd()), kEpsilon));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(m34ad),
                              ExtractGradient(m34wd.ToAutoDiffXd()), kEpsilon));

  DifferentiableMatrix<Matrix34d, true>::TransposeType m34wdt =
      m34wd.transpose();
  auto m34adt = m34ad.transpose();

  EXPECT_TRUE(CompareMatrices(ExtractValue(m34adt),
                              ExtractValue(m34wdt.ToAutoDiffXd()), kEpsilon));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(m34adt),
                              ExtractGradient(m34wdt.ToAutoDiffXd()), kEpsilon));
}

}  // namespace
}  // namespace math
}  // namespace drake

