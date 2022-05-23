#include "drake/math/differentiable_matrix.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::AutoDiffScalar;

namespace drake {
namespace math {
namespace {

GTEST_TEST(DifferentiableMatrix, Construction) {

}

}  // namespace
}  // namespace math
}  // namespace drake

