#include "drake/systems/framework/system_input.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

class FreestandingInputPortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
    vec->get_mutable_value() << 5, 6;
    port_.reset(new FreestandingInputPort<int>(std::move(vec), 42.0));
    port_->set_invalidation_callback(
        std::bind(&FreestandingInputPortTest::Invalidate, this));
  }

  std::unique_ptr<FreestandingInputPort<int>> port_;
  int64_t latest_version_ = -1;

 private:
  void Invalidate() {
    latest_version_ = port_->get_version();
  }
};

TEST_F(FreestandingInputPortTest, Access) {
  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, port_->get_vector_data()->get_value());
  EXPECT_EQ(42.0, port_->get_sample_time_sec());
}

// Tests that changes to the vector data are propagated to the input port
// that wraps it.
TEST_F(FreestandingInputPortTest, Mutation) {
  EXPECT_EQ(0, port_->get_version());
  port_->GetMutableVectorData()->get_mutable_value() << 7, 8;

  // Check that the version number was incremented.
  EXPECT_EQ(1, port_->get_version());

  // Check that the vector contents changed.
  VectorX<int> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected, port_->get_vector_data()->get_value());
}

// Tests that the single-argument constructor for FreestandingInputPort creates
// a continuous port.
TEST_F(FreestandingInputPortTest, ContinouousPort) {
  std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
  port_.reset(new FreestandingInputPort<int>(std::move(vec)));
  EXPECT_EQ(0.0, port_->get_sample_time_sec());
}

class ConnectedInputPortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
    vec->get_mutable_value() << 5, 6;
    output_port_.reset(new OutputPort<int>(std::move(vec)));
    port_.reset(new ConnectedInputPort<int>(output_port_.get(), 42.0));
    port_->set_invalidation_callback(
        std::bind(&ConnectedInputPortTest::Invalidate, this));
  }

  std::unique_ptr<OutputPort<int>> output_port_;
  std::unique_ptr<ConnectedInputPort<int>> port_;
  int64_t latest_version_ = -1;

 private:
  void Invalidate() {
    latest_version_ = port_->get_version();
  }
};

TEST_F(ConnectedInputPortTest, Access) {
  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, port_->get_vector_data()->get_value());
  EXPECT_EQ(42.0, port_->get_sample_time_sec());
}

// Tests that changes on the output port are propagated to the input port that
// is connected to it.
TEST_F(ConnectedInputPortTest, Mutation) {
  EXPECT_EQ(0, port_->get_version());
  output_port_->GetMutableVectorData()->get_mutable_value() << 7, 8;

  // Check that the version number was incremented.
  EXPECT_EQ(1, port_->get_version());

  // Check that the invalidation callback was called.
  EXPECT_EQ(1, latest_version_);

  // Check that the vector contents changed.
  VectorX<int> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected, port_->get_vector_data()->get_value());
}

}  // namespace systems
}  // namespace drake
