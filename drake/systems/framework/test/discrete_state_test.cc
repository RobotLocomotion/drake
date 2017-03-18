#include "drake/systems/framework/discrete_state.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace {

class DiscreteStateTest : public ::testing::Test {
 public:
  DiscreteStateTest() {
    data_.push_back(BasicVector<double>::Make({1.0, 1.0}));
    data_.push_back(BasicVector<double>::Make({2.0, 3.0}));
  }

 protected:
  std::vector<std::unique_ptr<BasicVector<double>>> data_;
};

TEST_F(DiscreteStateTest, OwnedState) {
  DiscreteState<double> xd(std::move(data_));
  EXPECT_EQ(1.0, xd.get_discrete_state(0)->GetAtIndex(0));
  EXPECT_EQ(1.0, xd.get_discrete_state(0)->GetAtIndex(1));
  EXPECT_EQ(2.0, xd.get_discrete_state(1)->GetAtIndex(0));
  EXPECT_EQ(3.0, xd.get_discrete_state(1)->GetAtIndex(1));
}

TEST_F(DiscreteStateTest, UnownedState) {
  DiscreteState<double> xd(
      std::vector<BasicVector<double>*>{data_[0].get(), data_[1].get()});
  EXPECT_EQ(1.0, xd.get_discrete_state(0)->GetAtIndex(0));
  EXPECT_EQ(1.0, xd.get_discrete_state(0)->GetAtIndex(1));
  EXPECT_EQ(2.0, xd.get_discrete_state(1)->GetAtIndex(0));
  EXPECT_EQ(3.0, xd.get_discrete_state(1)->GetAtIndex(1));
}

TEST_F(DiscreteStateTest, Clone) {
  DiscreteState<double> xd(
      std::vector<BasicVector<double>*>{data_[0].get(), data_[1].get()});
  std::unique_ptr<DiscreteState<double>> clone = xd.Clone();
  EXPECT_EQ(1.0, clone->get_discrete_state(0)->GetAtIndex(0));
  EXPECT_EQ(1.0, clone->get_discrete_state(0)->GetAtIndex(1));
  EXPECT_EQ(2.0, clone->get_discrete_state(1)->GetAtIndex(0));
  EXPECT_EQ(3.0, clone->get_discrete_state(1)->GetAtIndex(1));
}

TEST_F(DiscreteStateTest, SetFrom) {
  DiscreteState<AutoDiffXd> ad_xd(BasicVector<AutoDiffXd>::Make(
      {AutoDiffXd(1.0), AutoDiffXd(2.0)}));
  DiscreteState<double> double_xd(BasicVector<double>::Make({3.0, 4.0}));
  ad_xd.SetFrom(double_xd);

  const BasicVector<AutoDiffXd>& vec = *ad_xd.get_discrete_state(0);
  EXPECT_EQ(3.0, vec[0].value());
  EXPECT_EQ(0, vec[0].derivatives().size());
  EXPECT_EQ(4.0, vec[1].value());
  EXPECT_EQ(0, vec[1].derivatives().size());
}

}  // namespace
}  // namespace systems
}  // namespace drake
