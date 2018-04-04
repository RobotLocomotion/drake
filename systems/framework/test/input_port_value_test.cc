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
    std::unique_ptr<BasicVector<double>> vec(new BasicVector<double>(2));
    vec->get_mutable_value() << 5, 6;
    port_value_.reset(new FreestandingInputPortValue(
        std::make_unique<Value<BasicVector<double>>>(std::move(vec))));
  }

  std::unique_ptr<FreestandingInputPortValue> port_value_;
};

TEST_F(FreestandingInputPortVectorTest, Access) {
  VectorX<double> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected,
            port_value_->template get_vector_data<double>()->get_value());
}

// Tests that changes to the vector data are propagated to the input port
// that wraps it.
TEST_F(FreestandingInputPortVectorTest, Mutation) {
  port_value_->template GetMutableVectorData<double>()->get_mutable_value()
      << 7,
      8;

  // Check that the vector contents changed.
  VectorX<double> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected,
            port_value_->template get_vector_data<double>()->get_value());
}

class FreestandingInputPortAbstractValueTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto value = std::make_unique<Value<std::string>>("foo");
    port_value_.reset(new FreestandingInputPortValue(std::move(value)));
  }

  std::unique_ptr<FreestandingInputPortValue> port_value_;
};

TEST_F(FreestandingInputPortAbstractValueTest, Access) {
  EXPECT_EQ("foo", port_value_->get_abstract_data()->GetValue<std::string>());
}

// Tests that changes to the vector data are propagated to the input port
// that wraps it.
TEST_F(FreestandingInputPortAbstractValueTest, Mutation) {
  port_value_->GetMutableData()->GetMutableValue<std::string>() = "bar";

  // Check that the contents changed.
  EXPECT_EQ("bar", port_value_->get_abstract_data()->GetValue<std::string>());
}

class DependentInputPortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<BasicVector<double>> vec(new BasicVector<double>(2));
    vec->get_mutable_value() << 5, 6;
    output_port_value_.reset(new OutputPortValue(std::move(vec)));
    input_port_value_.reset(
        new DependentInputPortValue(output_port_value_.get()));
  }

  std::unique_ptr<OutputPortValue> output_port_value_;
  std::unique_ptr<DependentInputPortValue> input_port_value_;
};

TEST_F(DependentInputPortTest, Access) {
  VectorX<double> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected,
            input_port_value_->template get_vector_data<double>()->get_value());
}

// Tests that changes on the output port are propagated to the input port that
// is connected to it.
TEST_F(DependentInputPortTest, Mutation) {
  output_port_value_->template GetMutableVectorData<double>()
          ->get_mutable_value()
      << 7,
      8;

  // Check that the vector contents changed.
  VectorX<double> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected,
            input_port_value_->template get_vector_data<double>()->get_value());
}

}  // namespace systems
}  // namespace drake
