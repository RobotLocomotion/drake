#include "drake/systems/framework/abstract_values.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/test_utilities/pack_value.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {
namespace {

class AbstractStateTest : public ::testing::Test {
 public:
  AbstractStateTest() {
    data_.push_back(PackValue(42));
    data_.push_back(PackValue(76));
  }

 protected:
  std::vector<std::unique_ptr<AbstractValue>> data_;
};

TEST_F(AbstractStateTest, OwnedState) {
  AbstractValues xm(std::move(data_));
  EXPECT_EQ(42, UnpackIntValue(xm.get_value(0)));
  EXPECT_EQ(76, UnpackIntValue(xm.get_value(1)));
}

TEST_F(AbstractStateTest, UnownedState) {
  AbstractValues xm(
      std::vector<AbstractValue*>{data_[0].get(), data_[1].get()});
  EXPECT_EQ(42, UnpackIntValue(xm.get_value(0)));
  EXPECT_EQ(76, UnpackIntValue(xm.get_value(1)));
}

TEST_F(AbstractStateTest, SingleValueConstructor) {
  AbstractValues xm(PackValue<int>(1000));
  ASSERT_EQ(1, xm.size());
  EXPECT_EQ(1000, UnpackIntValue(xm.get_value(0)));
}

TEST_F(AbstractStateTest, Clone) {
  AbstractValues xm(std::move(data_));
  std::unique_ptr<AbstractValues> clone = xm.Clone();
  EXPECT_EQ(42, UnpackIntValue(clone->get_value(0)));
  EXPECT_EQ(76, UnpackIntValue(clone->get_value(1)));
}

}  // namespace
}  // namespace systems
}  // namespace drake
