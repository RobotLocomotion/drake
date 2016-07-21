#include "drake/systems/framework/primitives/vector_constant3.h"

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {
namespace {

// Verify that we can create a VectorConstant<double> system using an explicit
// VectorInterface as the constant, and extract the constant value from its
// output port.
GTEST_TEST(VectorConstantTest, DoubleVectorInterface) {
  Eigen::Vector3d expected;
  expected << 5, 7, 9;

  auto basic = std::make_unique<BasicVector<double>>(3);
  basic->get_mutable_value() = expected;
  VectorConstant3<double> constant("double", std::move(basic));
  auto context = constant.CreateDefaultContext();
  const VectorInterface<double>& result =
      constant.EvalVectorOutputPort(*context, 0);

  EXPECT_EQ(result.get_value(), expected);
}

// Try the same thing instantiating with an AutoDiffScalar.
GTEST_TEST(VectorConstantTest, AutoDiff) {
  using T = Eigen::AutoDiffScalar<Eigen::Vector2d>;
  Vector3<T> expected;
  expected << 5, 7, 9;

  auto basic = std::make_unique<BasicVector<T>>(3);
  basic->get_mutable_value() = expected;
  VectorConstant3<T> constant("autodiff", std::move(basic));
  auto context = constant.CreateDefaultContext();
  const VectorInterface<T>& result =
      constant.EvalVectorOutputPort(*context, 0);

  EXPECT_EQ(result.get_value(), expected);
}

// Check the convenience constructor that takes an Eigen vector directly.
GTEST_TEST(VectorConstantTest, DoubleEigenVector) {
  Eigen::Vector3d expected;
  expected << 5, 7, 9;

  // This is the simple version using an Eigen vector.
  VectorConstant3<double> constant("convenient", expected);
  auto context = constant.CreateDefaultContext();
  EXPECT_EQ(constant.EvalVectorOutputPort(*context, 0).get_value(), expected);

  // This takes an Eigen expression.
  VectorConstant3<double> constant2("convenient2", 2 * expected);
  auto context2 = constant2.CreateDefaultContext();
  EXPECT_EQ(constant2.EvalVectorOutputPort(*context2, 0).get_value(),
            2 * expected);
}


// Check the convenience constructor that takes a scalar.
GTEST_TEST(VectorConstantTest, DoubleScalar) {
  const double expected = 1.25;

  VectorConstant3<double> constant("double", expected);
  auto context = constant.CreateDefaultContext();
  const VectorInterface<double>& result =
      constant.EvalVectorOutputPort(*context, 0);

  EXPECT_EQ(result.get_value()[0], expected);
}

// Try the same thing instantiating with an AutoDiffScalar.
GTEST_TEST(VectorConstantTest, AutoDiffScalar) {
  using T = Eigen::AutoDiffScalar<Eigen::Vector2d>;

  const T expected = 3.5;

  VectorConstant3<T> constant("autodiff", expected);
  auto context = constant.CreateDefaultContext();
  const VectorInterface<T>& result =
      constant.EvalVectorOutputPort(*context, 0);

  EXPECT_EQ(result.get_value()[0], expected);
}

}  // namespace
}  // namespace systems
}  // namespace drake