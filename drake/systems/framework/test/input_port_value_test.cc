#include "drake/systems/framework/input_port_value.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

class FreestandingInputPortVectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
    vec->get_mutable_value() << 5, 6;
    port_value_.reset(new FreestandingInputPortValue(std::move(vec)));
    port_value_->set_invalidation_callback(
        std::bind(&FreestandingInputPortVectorTest::Invalidate, this));
  }

  std::unique_ptr<FreestandingInputPortValue> port_value_;
  int64_t latest_version_ = -1;

 private:
  void Invalidate() { latest_version_ = port_value_->get_version(); }
};

TEST_F(FreestandingInputPortVectorTest, Access) {
  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected,
            port_value_->template get_vector_data<int>()->get_value());
}

// Tests that changes to the vector data are propagated to the input port
// that wraps it.
TEST_F(FreestandingInputPortVectorTest, Mutation) {
  EXPECT_EQ(0, port_value_->get_version());
  port_value_->template GetMutableVectorData<int>()->get_mutable_value()
      << 7, 8;

  // Check that the version number was incremented.
  EXPECT_EQ(1, port_value_->get_version());

  // Check that the vector contents changed.
  VectorX<int> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected,
            port_value_->template get_vector_data<int>()->get_value());
}

class FreestandingInputPortAbstractValueTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto value = std::make_unique<Value<std::string>>("foo");
    port_value_.reset(new FreestandingInputPortValue(std::move(value)));
    port_value_->set_invalidation_callback(
        std::bind(&FreestandingInputPortAbstractValueTest::Invalidate, this));
  }

  std::unique_ptr<FreestandingInputPortValue> port_value_;
  int64_t latest_version_ = -1;

 private:
  void Invalidate() { latest_version_ = port_value_->get_version(); }
};

TEST_F(FreestandingInputPortAbstractValueTest, Access) {
  EXPECT_EQ("foo", port_value_->get_abstract_data()->GetValue<std::string>());
}

// Tests that changes to the vector data are propagated to the input port
// that wraps it.
TEST_F(FreestandingInputPortAbstractValueTest, Mutation) {
  EXPECT_EQ(0, port_value_->get_version());
  port_value_->GetMutableData()->GetMutableValue<std::string>() = "bar";

  // Check that the version number was incremented.
  EXPECT_EQ(1, port_value_->get_version());

  // Check that the contents changed.
  EXPECT_EQ("bar", port_value_->get_abstract_data()->GetValue<std::string>());
}

class DependentInputPortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
    vec->get_mutable_value() << 5, 6;
    output_port_value_.reset(new OutputPortValue(std::move(vec)));
    input_port_value_.reset(
        new DependentInputPortValue(output_port_value_.get()));
    input_port_value_->set_invalidation_callback(
        std::bind(&DependentInputPortTest::Invalidate, this));
  }

  std::unique_ptr<OutputPortValue> output_port_value_;
  std::unique_ptr<DependentInputPortValue> input_port_value_;
  int64_t latest_version_ = -1;

 private:
  void Invalidate() { latest_version_ = input_port_value_->get_version(); }
};

TEST_F(DependentInputPortTest, Access) {
  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected,
            input_port_value_->template get_vector_data<int>()->get_value());
}

// Tests that changes on the output port are propagated to the input port that
// is connected to it.
TEST_F(DependentInputPortTest, Mutation) {
  EXPECT_EQ(0, input_port_value_->get_version());
  output_port_value_->template GetMutableVectorData<int>()->get_mutable_value()
      << 7,
      8;

  // Check that the version number was incremented.
  EXPECT_EQ(1, input_port_value_->get_version());

  // Check that the invalidation callback was called.
  EXPECT_EQ(1, latest_version_);

  // Check that the vector contents changed.
  VectorX<int> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected,
            input_port_value_->template get_vector_data<int>()->get_value());
}

}  // namespace systems
}  // namespace drake
