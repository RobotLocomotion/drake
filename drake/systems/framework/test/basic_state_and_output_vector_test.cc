#include "drake/systems/framework/basic_state_and_output_vector.h"

#include <memory>

#include "gtest/gtest.h"

#include <Eigen/Dense>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace {

const int kLength = 2;

// The implementation of BasicStateAndOutputVector is largely inherited from
// BasicStateVector and BasicVector.  Here, we lightly touch most methods to
// ensure they are sane, relying on the delegated implementations' unit tests
// to ensure correctness.  Separate tests below cover the special cases.
GTEST_TEST(BasicStateAndOutputVectorTest, CoverDelegatedMethods) {
  // The device under test.
  BasicStateAndOutputVector<int> dut(kLength);
  EXPECT_EQ(kLength, dut.size());

  // Touch most VectorBase methods.
  dut.set_value((Eigen::VectorXi(kLength) << 10, 11).finished());
  EXPECT_EQ(11, dut.get_value()(1));
  dut.get_mutable_value()(1) = 5;
  EXPECT_EQ(5, dut.get_value()(1));

  // Touch most BasicStateVector methods.
  dut.SetFromVector((Eigen::VectorXi(kLength) << 20, 21).finished());
  EXPECT_EQ(21, dut.GetAtIndex(1));
  EXPECT_EQ(21, dut.CopyToVector()(1));
  dut.SetAtIndex(1, 6);
  EXPECT_EQ(6, dut.GetAtIndex(1));
  Eigen::VectorXi target = (Eigen::VectorXi(kLength) << 30, 31).finished();
  dut.ScaleAndAddToVector(3, target);
  EXPECT_EQ(6 * 3 + 31, target(1));
}

// Touch all constructors.
GTEST_TEST(BasicStateAndOutputVectorTest, Constructors) {
  BasicStateAndOutputVector<int> by_size(kLength);
  EXPECT_EQ(kLength, by_size.size());
  EXPECT_EQ(0, by_size.get_value()(1));

  BasicStateAndOutputVector<int> by_vector(std::vector<int>(kLength, 0));
  EXPECT_EQ(kLength, by_vector.size());
  EXPECT_EQ(0, by_vector.get_value()(1));

  BasicStateAndOutputVector<int> by_interface(
      std::make_unique<BasicVector<int>>(kLength));
  EXPECT_EQ(kLength, by_interface.size());
  EXPECT_EQ(0, by_interface.get_value()(1));
}

// Confirm that setting and getting are the same, no matter the API.
GTEST_TEST(BasicStateAndOutputVectorTest, MatchingSetAndGet) {
  // The device under test.
  BasicStateAndOutputVector<int> dut(kLength);

  // Set via VectorBase; get via BasicStateVector.
  dut.get_mutable_value()(1) = 5;
  EXPECT_EQ(5, dut.GetAtIndex(1));

  // Set via BasicStateVector; get via VectorBase.
  dut.SetAtIndex(0, 6);
  EXPECT_EQ(6, dut.get_value()(0));
}

// Confirm that cloning via the State API preserves our type.
GTEST_TEST(BasicStateAndOutputVectorTest, CloneState) {
  // The device under test.
  BasicStateAndOutputVector<int> dut(kLength);
  dut.SetFromVector((Eigen::VectorXi(kLength) << 10, 11).finished());

  std::unique_ptr<LeafStateVector<int>> clone = dut.Clone();
  BasicStateAndOutputVector<int>* typed_clone =
      dynamic_cast<BasicStateAndOutputVector<int>*>(clone.get());
  ASSERT_NE(nullptr, typed_clone);
  EXPECT_EQ(10, typed_clone->GetAtIndex(0));
  EXPECT_EQ(11, typed_clone->GetAtIndex(1));
}

// Confirm that cloning via the Vector API preserves our type.
GTEST_TEST(BasicStateAndOutputVectorTest, CloneVector) {
  // The device under test.
  BasicStateAndOutputVector<int> dut(kLength);
  dut.set_value((Eigen::VectorXi(kLength) << 10, 11).finished());

  std::unique_ptr<VectorBase<int>> clone = dut.CloneVector();
  BasicStateAndOutputVector<int>* typed_clone =
      dynamic_cast<BasicStateAndOutputVector<int>*>(clone.get());
  ASSERT_NE(nullptr, typed_clone);
  EXPECT_EQ(10, typed_clone->get_value()(0));
  EXPECT_EQ(11, typed_clone->get_value()(1));
}

}  // namespace
}  // namespace systems
}  // namespace drake
