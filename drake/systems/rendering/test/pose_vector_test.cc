#include "drake/systems/rendering/pose_vector.h"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/test/symbolic_test_util.h"

namespace drake {

using symbolic::test::ExprEqual;

namespace systems {
namespace rendering {
namespace {

/// Tests that the PoseVector is initialized to identity.
GTEST_TEST(PoseVector, InitiallyIdentity) {
  const PoseVector<double> vec;
  EXPECT_TRUE(CompareMatrices(Isometry3<double>::Identity().matrix(),
                              vec.get_isometry().matrix()));

  // Check that the underlying storage has the values we'd expect.
  for (int i = 0; i < 3; ++i) {
    // p_WA is entirely zero.
    EXPECT_EQ(0.0, vec[i]);
  }

  // The real part of R_WA is cos(0) = 1.
  EXPECT_EQ(1.0, vec[3]);

  for (int i = 4; i < vec.kSize; ++i) {
    // The imaginary parts of R_WA are sin(0) = 0.
    EXPECT_EQ(0.0, vec[i]);
  }
}

GTEST_TEST(PoseVector, Rotation) {
  PoseVector<double> vec;
  // Rotate by 2π/3 around the axis {1, 1, 1}.
  vec.set_rotation(Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5));
  // Check that the data was copied.
  for (int i = 3; i < vec.kSize; ++i) {
    EXPECT_EQ(0.5, vec[i]);
  }
  // Check that the isometry transforms the x-axis to the y-axis.
  const Isometry3<double> X_WA = vec.get_isometry();
  const Eigen::Vector3d p_in = {1.0, 0.0, 0.0};
  const Eigen::Vector3d p_out = X_WA * p_in;
  EXPECT_EQ((Eigen::Vector3d{0.0, 1.0, 0.0}), p_out);
}

GTEST_TEST(PoseVector, Translation) {
  PoseVector<double> vec;
  vec.set_translation(Eigen::Translation<double, 3>(1.0, 2.0, 3.0));
  // Check that the data was copied.
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(1.0 * (i + 1), vec[i]);
  }
  // Check that the isometry is a pure translation.
  const Isometry3<double> X_WA = vec.get_isometry();
  const Eigen::Vector3d p_in = {1.0, 2.0, 3.0};
  const Eigen::Vector3d p_out = X_WA * p_in;
  EXPECT_EQ((Eigen::Vector3d{2.0, 4.0, 6.0}), p_out);
}

// Tests that PoseVector<T>::Clone has PoseVector type.
GTEST_TEST(PoseVector, Clone) {
  const PoseVector<double> vec;
  auto clone = vec.Clone();
  EXPECT_NE(nullptr, dynamic_cast<PoseVector<double>*>(clone.get()));
}

GTEST_TEST(PoseVector, Symbolic) {
  PoseVector<symbolic::Expression> vec;
  vec[0] = symbolic::Variable("x");
  vec[1] = symbolic::Variable("y");
  vec[2] = symbolic::Variable("z");
  vec[3] = symbolic::Variable("w");
  vec[4] = symbolic::Variable("a");
  vec[5] = symbolic::Variable("b");
  vec[6] = symbolic::Variable("c");

  // Spot-check some terms from the output.
  const Isometry3<symbolic::Expression> X_WA = vec.get_isometry();
  const auto matrix = X_WA.matrix();
  // Rotation - first element on the diagonal
  EXPECT_PRED2(ExprEqual,
               1 - 2 * vec[5] * vec[5] - 2 * vec[6] * vec[6], matrix(0, 0));
  // Rotation - second element in the third row
  EXPECT_PRED2(ExprEqual,
               2 * vec[5] * vec[6] + 2 * vec[3] * vec[4], matrix(2, 1));
  // Translation - fourth element in the second row
  EXPECT_PRED2(ExprEqual, vec[1], matrix(1, 3));
}

GTEST_TEST(PoseVector, Autodiff) {
  PoseVector<AutoDiffXd> vec;
  for (int i = 0; i < vec.size(); ++i) {
    // Initialize the position to {0.5, 0.5, 0.5}, and the rotation to
    // 2π/3 around the axis {1, 1, 1}.
    vec[i].value() = 0.5;

    // Initialize the Jacobian to the identity matrix.
    vec[i].derivatives().resize(vec.kSize);
    for (int j = 0; j < vec.kSize; ++j) {
      vec[i].derivatives()[j] = j == i ? 1.0 : 0.0;
    }
  }

  // Spot-check some terms from the output.
  const Isometry3<AutoDiffXd> X_WA = vec.get_isometry();
  auto matrix = X_WA.matrix();

  // The partial of the first element of the rotation matrix with respect to
  // "b", the j coefficient in the quaternion, is -4 * b.  With respect to the
  // real part of the quaternion, it's zero.
  EXPECT_DOUBLE_EQ(-4.0 * vec[5].value(), matrix(0, 0).derivatives()[5]);
  EXPECT_DOUBLE_EQ(0, matrix(0, 0).derivatives()[3]);

  // The partial of the third element of the transformation vector with respect
  // to "z" is 1. With respect to anything else, it's zero.
  const auto& dz = matrix(2, 3).derivatives();
  for (int i = 0; i < dz.size(); ++i) {
    EXPECT_EQ(i == 2 ? 1 : 0, dz[i]);
  }

  // The bottom row of the rotation matrix is constant.
  for (int i = 0; i < 4; ++i) {
    const auto& partials = matrix(3, i).derivatives();
    for (int j = 0; j < partials.size(); ++j) {
      EXPECT_EQ(0.0, partials[j]);
    }
  }
}

};  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
