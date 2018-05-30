#include "drake/systems/framework/output_port_value.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class OutputPortVectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const BasicVector<double> vec{5, 6};
    output_port_value_.reset(new OutputPortValue(vec.Clone()));
  }

  std::unique_ptr<OutputPortValue> output_port_value_;
};

TEST_F(OutputPortVectorTest, Access) {
  VectorX<double> expected(2);
  expected << 5, 6;
  EXPECT_EQ(
      expected,
      output_port_value_->template get_vector_data<double>()->get_value());
}

TEST_F(OutputPortVectorTest, Mutation) {
  output_port_value_->template GetMutableVectorData<double>()
          ->get_mutable_value()
      << 7,
      8;

  // Check that the vector contents changed.
  VectorX<double> expected(2);
  expected << 7, 8;
  EXPECT_EQ(
      expected,
      output_port_value_->template get_vector_data<double>()->get_value());
}

// Tests that a clone of an OutputPortValue has a deep copy of the data.
TEST_F(OutputPortVectorTest, Clone) {
  auto clone = output_port_value_->Clone();
  // Changes to the original port should not affect the clone.
  output_port_value_->template GetMutableVectorData<double>()
          ->get_mutable_value()
      << 7,
      8;

  VectorX<double> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, clone->template get_vector_data<double>()->get_value());

  // The type should be preserved.
  EXPECT_NE(nullptr, clone->template get_vector_data<double>());
}

class OutputPortAbstractValueTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto value = std::make_unique<Value<std::string>>("foo");
    output_port_value_.reset(new OutputPortValue(std::move(value)));
  }

  std::unique_ptr<OutputPortValue> output_port_value_;
};

TEST_F(OutputPortAbstractValueTest, Access) {
  EXPECT_EQ("foo",
            output_port_value_->get_abstract_data()->GetValue<std::string>());
}

TEST_F(OutputPortAbstractValueTest, Mutation) {
  output_port_value_->GetMutableData()->GetMutableValue<std::string>() = "bar";

  // Check that the contents changed.
  EXPECT_EQ("bar",
            output_port_value_->get_abstract_data()->GetValue<std::string>());
}

// Tests that a clone of an OutputPortValue has a deep copy of the data.
TEST_F(OutputPortAbstractValueTest, Clone) {
  auto clone = output_port_value_->Clone();
  // Changes to the original port should not affect the clone.
  clone->GetMutableData()->GetMutableValue<std::string>() = "bar";
  EXPECT_EQ("bar", clone->get_abstract_data()->GetValue<std::string>());
  EXPECT_EQ("foo",
            output_port_value_->get_abstract_data()->GetValue<std::string>());
}

class LeafSystemOutputTest : public ::testing::Test {
 protected:
  void SetUp() override {
    {
      const BasicVector<double> vec{5, 25};
      output_.add_port(std::make_unique<OutputPortValue>(vec.Clone()));
    }
    {
      const BasicVector<double> vec{125, 625, 3125};
      output_.add_port(std::make_unique<OutputPortValue>(vec.Clone()));
    }
  }

  LeafSystemOutput<double> output_;
};

// Tests that a clone of the LeafSystemOutput has a deep copy of the data on
// each port.
TEST_F(LeafSystemOutputTest, Clone) {
  std::unique_ptr<SystemOutput<double>> clone = output_.Clone();

  {
    const OutputPortValue& port_value = clone->get_port_value(0);
    VectorX<double> expected(2);
    expected << 5, 25;
    EXPECT_EQ(expected,
              port_value.template get_vector_data<double>()->get_value());
    EXPECT_NE(nullptr, port_value.template get_vector_data<double>());
  }

  {
    const OutputPortValue& port_value = clone->get_port_value(1);
    VectorX<double> expected(3);
    expected << 125, 625, 3125;
    EXPECT_EQ(expected,
              port_value.template get_vector_data<double>()->get_value());
    EXPECT_NE(nullptr, port_value.template get_vector_data<double>());
  }
}

}  // namespace systems
}  // namespace drake
