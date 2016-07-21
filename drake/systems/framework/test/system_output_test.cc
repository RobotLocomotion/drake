#include "drake/systems/framework/system_output.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

class TestOutputPortListener : public OutputPortListenerInterface {
 public:
  void Invalidate() override {
    ++invalidations_;
  };

  int get_invalidations() { return invalidations_; }

 private:
  int invalidations_ = 0;
};

class VectorOutputPortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
    vec->get_mutable_value() << 5, 6;
    port_.reset(new VectorOutputPort<int>(std::move(vec)));
  }

  std::unique_ptr<VectorOutputPort<int>> port_;
};

TEST_F(VectorOutputPortTest, Access) {
  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, port_->get_vector_data()->get_value());
}

TEST_F(VectorOutputPortTest, Mutation) {
  EXPECT_EQ(0, port_->get_version());
  port_->GetMutableVectorData()->get_mutable_value() << 7, 8;

  // Check that the version number was incremented.
  EXPECT_EQ(1, port_->get_version());

  // Check that the vector contents changed.
  VectorX<int> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected, port_->get_vector_data()->get_value());
}

TEST_F(VectorOutputPortTest, Listeners) {
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

}  // namespace systems
}  // namespace drake
