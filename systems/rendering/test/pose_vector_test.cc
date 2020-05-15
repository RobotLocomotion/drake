#include "drake/systems/rendering/pose_vector.h"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using symbolic::test::ExprEqual;

namespace systems {
namespace rendering {
namespace {

// Tests that the default PoseVector is initialized to identity.
GTEST_TEST(PoseVector, InitiallyIdentity) {
  const PoseVector<double> vec;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_TRUE(CompareMatrices(Isometry3<double>::Identity().matrix(),
                              vec.get_isometry().matrix()));
#pragma GCC diagnostic pop
EXPECT_TRUE(CompareMatrices(RigidTransformd().matrix(),
                              vec.transform().matrix()));
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

// Tests the fully-parameterized PoseVector.
GTEST_TEST(PoseVector, FullyParameterizedCtor) {
  const Eigen::Quaternion<double> rotation(0.5, 0.5, 0.5, 0.5);
  const Eigen::Translation3d translation(1.0, 2.0, 3.0);
  const PoseVector<double> vec(rotation, translation);

  Eigen::Isometry3d isometry_expected(translation);
  isometry_expected.rotate(rotation);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_TRUE(
      CompareMatrices(isometry_expected.matrix(), vec.get_isometry().matrix()));
#pragma GCC diagnostic pop
  RigidTransformd transform_expected =
      RigidTransformd(translation) *
      RigidTransformd(rotation, Vector3d::Zero());
  EXPECT_TRUE(
      CompareMatrices(transform_expected.matrix(), vec.transform().matrix()));
  EXPECT_TRUE(CompareMatrices(
      Eigen::Isometry3d(translation).matrix(),
      Eigen::Isometry3d(vec.get_translation()).matrix()));
  EXPECT_TRUE(CompareMatrices(rotation.matrix(), vec.get_rotation().matrix()));
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
  const RigidTransformd X_WA = vec.transform();
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
  const RigidTransformd X_WA = vec.transform();
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
  const RigidTransform<symbolic::Expression> X_WA = vec.transform();
  const auto matrix = X_WA.matrix();
  // Note: RigidTransform normalizes the quaternion in an order of operations
  // that leads to a *very* specific set of symbolic expressions. That is what
  // is represented below.

  // The squared L2 norm of the quaternion; if it were already unit length,
  // this factor would be one and the expressions down below would be much
  // simpler.
  const symbolic::Expression q_norm2 =
      vec[3] * vec[3] + vec[4] * vec[4] + vec[5] * vec[5] + vec[6] * vec[6];
  
  // Rotation - first element on the diagonal
  EXPECT_PRED2(
      ExprEqual,
      1 - vec[5] * ((2 * vec[5]) / q_norm2) - vec[6] * ((2 * vec[6]) / q_norm2),
      matrix(0, 0));
  // Rotation - second element in the third row
  EXPECT_PRED2(
      ExprEqual,
      vec[5] * ((2 * vec[6]) / q_norm2) + vec[3] * ((2 * vec[4]) / q_norm2),
      matrix(2, 1));
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
  const RigidTransform<AutoDiffXd> X_WA = vec.transform();
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
