#include "drake/systems/framework/modal_state.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {
namespace {


class ModalStateTest : public ::testing::Test {
 public:
  ModalStateTest() {
    data_.push_back(PackValue(42));
    data_.push_back(PackValue(76));
  }

 protected:
  std::unique_ptr<AbstractValue> PackValue(int value) {
    return std::unique_ptr<AbstractValue>(new Value<int>(value));
  }

  int UnpackValue(const AbstractValue& value) {
    return dynamic_cast<const Value<int>*>(&value)->get_value();
  }

  std::vector<std::unique_ptr<AbstractValue>> data_;
};


TEST_F(ModalStateTest, OwnedState) {
  ModalState xm(std::move(data_));
  EXPECT_EQ(42, UnpackValue(xm.get_modal_state(0)));
  EXPECT_EQ(76, UnpackValue(xm.get_modal_state(1)));
}

TEST_F(ModalStateTest, UnownedState) {
  ModalState xm(std::vector<AbstractValue*>{data_[0].get(), data_[1].get()});
  EXPECT_EQ(42, UnpackValue(xm.get_modal_state(0)));
  EXPECT_EQ(76, UnpackValue(xm.get_modal_state(1)));
}

TEST_F(ModalStateTest, Clone) {
  ModalState xm(std::move(data_));
  std::unique_ptr<ModalState> clone = xm.Clone();
  EXPECT_EQ(42, UnpackValue(clone->get_modal_state(0)));
  EXPECT_EQ(76, UnpackValue(clone->get_modal_state(1)));
}

}  // namespace
}  // namespace systems
}  // namespace drake
