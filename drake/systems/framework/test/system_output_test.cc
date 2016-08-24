#include "drake/systems/framework/system_output.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

class TestOutputPortListener : public OutputPortListenerInterface {
 public:
  void Invalidate() override {
    invalidations_++;
  };

  int get_invalidations() { return invalidations_; }

 private:
  int invalidations_ = 0;
};

class OutputPortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
    vec->get_mutable_value() << 5, 6;
    port_.reset(new OutputPort<int>(std::move(vec)));
  }

  std::unique_ptr<OutputPort<int>> port_;
};

TEST_F(OutputPortTest, Access) {
  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, port_->get_vector_data()->get_value());
}

TEST_F(OutputPortTest, Mutation) {
  EXPECT_EQ(0, port_->get_version());
  port_->GetMutableVectorData()->get_mutable_value() << 7, 8;

  // Check that the version number was incremented.
  EXPECT_EQ(1, port_->get_version());

  // Check that the vector contents changed.
  VectorX<int> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected, port_->get_vector_data()->get_value());
}

// Tests that a clone of an OutputPort has a deep copy of the data.
TEST_F(OutputPortTest, Clone) {
  auto clone = port_->Clone();
  // Changes to the original port should not affect the clone.
  port_->GetMutableVectorData()->get_mutable_value() << 7, 8;

  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, clone->get_vector_data()->get_value());

  // The type should be preserved.
  EXPECT_NE(nullptr,
            dynamic_cast<const BasicVector<int>*>(clone->get_vector_data()));
}

// Tests that listeners are notified when GetMutableVectorData is called.
TEST_F(OutputPortTest, Listeners) {
  TestOutputPortListener a, b, c;
  port_->add_dependent(&a);
  port_->add_dependent(&b);
  port_->GetMutableVectorData();
  EXPECT_EQ(1, a.get_invalidations());
  EXPECT_EQ(1, b.get_invalidations());
  EXPECT_EQ(0, c.get_invalidations());

  port_->remove_dependent(&a);
  port_->add_dependent(&c);
  port_->GetMutableVectorData();
  EXPECT_EQ(1, a.get_invalidations());
  EXPECT_EQ(2, b.get_invalidations());
  EXPECT_EQ(1, c.get_invalidations());
}

class LeafSystemOutputTest : public ::testing::Test {
 protected:
  void SetUp() override {
    {
      std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
      vec->get_mutable_value() << 5, 25;
      output_.get_mutable_ports()->emplace_back(
          new OutputPort<int>(std::move(vec)));
    }
    {
      std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(3));
      vec->get_mutable_value() << 125, 625, 3125;
      output_.get_mutable_ports()->emplace_back(
          new OutputPort<int>(std::move(vec)));
    }
  }

  LeafSystemOutput<int> output_;
};

// Tests that a clone of the LeafSystemOutput has a deep copy of the data on
// each port.
TEST_F(LeafSystemOutputTest, Clone) {
  std::unique_ptr<SystemOutput<int>> clone = output_.Clone();

  {
    const OutputPort<int>& port = clone->get_port(0);
    VectorX<int> expected(2);
    expected << 5, 25;
    EXPECT_EQ(expected, port.get_vector_data()->get_value());
    EXPECT_NE(nullptr,
              dynamic_cast<const BasicVector<int>*>(port.get_vector_data()));
  }

  {
    const OutputPort<int>& port = clone->get_port(1);
    VectorX<int> expected(3);
    expected << 125, 625, 3125;
    EXPECT_EQ(expected, port.get_vector_data()->get_value());
    EXPECT_NE(nullptr,
              dynamic_cast<const BasicVector<int>*>(port.get_vector_data()));
  }
}

}  // namespace systems
}  // namespace drake
