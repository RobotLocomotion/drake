#include "drake/systems/framework/system_output.h"

#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/output_port_listener_interface.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class TestOutputPortListener : public detail::OutputPortListenerInterface {
 public:
  void Invalidate() override { invalidations_++; }
  void Disconnect() override { disconnections_++; }

  int get_invalidations() { return invalidations_; }
  int get_disconnections() { return disconnections_; }

 private:
  int invalidations_ = 0;
  int disconnections_ = 0;
};

class OutputPortVectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
    vec->get_mutable_value() << 5, 6;
    output_port_value_.reset(new OutputPortValue(std::move(vec)));
  }

  std::unique_ptr<OutputPortValue> output_port_value_;
};

TEST_F(OutputPortVectorTest, Access) {
  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected,
            output_port_value_->template get_vector_data<int>()->get_value());
}

TEST_F(OutputPortVectorTest, Mutation) {
  EXPECT_EQ(0, output_port_value_->get_version());
  output_port_value_->template GetMutableVectorData<int>()->get_mutable_value()
      << 7,
      8;

  // Check that the version number was incremented.
  EXPECT_EQ(1, output_port_value_->get_version());

  // Check that the vector contents changed.
  VectorX<int> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected,
            output_port_value_->template get_vector_data<int>()->get_value());
}

// Tests that a clone of an OutputPortValue has a deep copy of the data.
TEST_F(OutputPortVectorTest, Clone) {
  auto clone = output_port_value_->Clone();
  // Changes to the original port should not affect the clone.
  output_port_value_->template GetMutableVectorData<int>()->get_mutable_value()
      << 7,
      8;

  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, clone->template get_vector_data<int>()->get_value());

  // The type should be preserved.
  EXPECT_NE(nullptr, clone->template get_vector_data<int>());
}

// Tests that listeners are notified when GetMutableVectorData is called, and
// when the OutputPortValue is deleted.
TEST_F(OutputPortVectorTest, Listeners) {
  TestOutputPortListener a, b, c;
  output_port_value_->add_dependent(&a);
  output_port_value_->add_dependent(&b);
  output_port_value_->template GetMutableVectorData<int>();
  EXPECT_EQ(1, a.get_invalidations());
  EXPECT_EQ(1, b.get_invalidations());
  EXPECT_EQ(0, c.get_invalidations());

  output_port_value_->remove_dependent(&a);
  output_port_value_->add_dependent(&c);
  output_port_value_->template GetMutableVectorData<int>();
  EXPECT_EQ(1, a.get_invalidations());
  EXPECT_EQ(2, b.get_invalidations());
  EXPECT_EQ(1, c.get_invalidations());

  // Only the listeners that are connected at the time the OutputPortValue is
  // destroyed get a disconnect notification.
  output_port_value_.reset();
  EXPECT_EQ(0, a.get_disconnections());
  EXPECT_EQ(1, b.get_disconnections());
  EXPECT_EQ(1, c.get_disconnections());
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
  EXPECT_EQ(0, output_port_value_->get_version());
  output_port_value_->GetMutableData()->GetMutableValue<std::string>() = "bar";

  // Check that the version number was incremented.
  EXPECT_EQ(1, output_port_value_->get_version());

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
      std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
      vec->get_mutable_value() << 5, 25;
      output_.add_port(std::make_unique<OutputPortValue>(std::move(vec)));
    }
    {
      std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(3));
      vec->get_mutable_value() << 125, 625, 3125;
      output_.add_port(std::make_unique<OutputPortValue>(std::move(vec)));
    }
  }

  LeafSystemOutput<int> output_;
};

// Tests that a clone of the LeafSystemOutput has a deep copy of the data on
// each port.
TEST_F(LeafSystemOutputTest, Clone) {
  std::unique_ptr<SystemOutput<int>> clone = output_.Clone();

  {
    const OutputPortValue& port_value = clone->get_port_value(0);
    VectorX<int> expected(2);
    expected << 5, 25;
    EXPECT_EQ(expected,
              port_value.template get_vector_data<int>()->get_value());
    EXPECT_NE(nullptr, port_value.template get_vector_data<int>());
  }

  {
    const OutputPortValue& port_value = clone->get_port_value(1);
    VectorX<int> expected(3);
    expected << 125, 625, 3125;
    EXPECT_EQ(expected,
              port_value.template get_vector_data<int>()->get_value());
    EXPECT_NE(nullptr, port_value.template get_vector_data<int>());
  }
}

}  // namespace systems
}  // namespace drake
