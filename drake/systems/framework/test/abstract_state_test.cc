#include "drake/systems/framework/abstract_state.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/value.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

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
  AbstractState xm(std::move(data_));
  EXPECT_EQ(42, UnpackIntValue(xm.get_abstract_state(0)));
  EXPECT_EQ(76, UnpackIntValue(xm.get_abstract_state(1)));
}

TEST_F(AbstractStateTest, UnownedState) {
  AbstractState xm(std::vector<AbstractValue*>{data_[0].get(), data_[1].get()});
  EXPECT_EQ(42, UnpackIntValue(xm.get_abstract_state(0)));
  EXPECT_EQ(76, UnpackIntValue(xm.get_abstract_state(1)));
}

TEST_F(AbstractStateTest, Clone) {
  AbstractState xm(std::move(data_));
  std::unique_ptr<AbstractState> clone = xm.Clone();
  EXPECT_EQ(42, UnpackIntValue(clone->get_abstract_state(0)));
  EXPECT_EQ(76, UnpackIntValue(clone->get_abstract_state(1)));
}

}  // namespace
}  // namespace systems
}  // namespace drake
