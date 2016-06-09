#include "drake/systems/framework/system_output.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

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

}  // namespace systems
}  // namespace drake
