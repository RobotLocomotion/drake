#include "drake/systems/framework/discrete_values.h"

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace {

class DiscreteValuesTest : public ::testing::Test {
 public:
  DiscreteValuesTest() {
    data_.push_back(BasicVector<double>::Make({1.0, 1.0}));
    data_.push_back(BasicVector<double>::Make({2.0, 3.0}));
  }

 protected:
  std::vector<std::unique_ptr<BasicVector<double>>> data_;
};

TEST_F(DiscreteValuesTest, OwnedState) {
  DiscreteValues<double> xd(std::move(data_));
  EXPECT_EQ(1.0, xd.get_vector(0).GetAtIndex(0));
  EXPECT_EQ(1.0, xd.get_vector(0).GetAtIndex(1));
  EXPECT_EQ(2.0, xd.get_vector(1).GetAtIndex(0));
  EXPECT_EQ(3.0, xd.get_vector(1).GetAtIndex(1));
}

TEST_F(DiscreteValuesTest, UnownedState) {
  DiscreteValues<double> xd(
      std::vector<BasicVector<double>*>{data_[0].get(), data_[1].get()});
  EXPECT_EQ(1.0, xd.get_vector(0).GetAtIndex(0));
  EXPECT_EQ(1.0, xd.get_vector(0).GetAtIndex(1));
  EXPECT_EQ(2.0, xd.get_vector(1).GetAtIndex(0));
  EXPECT_EQ(3.0, xd.get_vector(1).GetAtIndex(1));
}

TEST_F(DiscreteValuesTest, NoNullsAllowed) {
  // Unowned.
  EXPECT_THROW(DiscreteValues<double>(
      std::vector<BasicVector<double>*>{nullptr, data_[1].get()}),
               std::logic_error);
  // Owned.
  data_.push_back(nullptr);
  EXPECT_THROW(DiscreteValues<double>(std::move(data_)), std::logic_error);
}

TEST_F(DiscreteValuesTest, Clone) {
  DiscreteValues<double> xd(
      std::vector<BasicVector<double>*>{data_[0].get(), data_[1].get()});
  std::unique_ptr<DiscreteValues<double>> clone = xd.Clone();
  EXPECT_EQ(1.0, clone->get_vector(0).GetAtIndex(0));
  EXPECT_EQ(1.0, clone->get_vector(0).GetAtIndex(1));
  EXPECT_EQ(2.0, clone->get_vector(1).GetAtIndex(0));
  EXPECT_EQ(3.0, clone->get_vector(1).GetAtIndex(1));
}

TEST_F(DiscreteValuesTest, SetFrom) {
  DiscreteValues<AutoDiffXd> ad_xd(
      BasicVector<AutoDiffXd>::Make({AutoDiffXd(1.0), AutoDiffXd(2.0)}));
  DiscreteValues<double> double_xd(BasicVector<double>::Make({3.0, 4.0}));
  ad_xd.SetFrom(double_xd);

  const BasicVector<AutoDiffXd>& vec = ad_xd.get_vector(0);
  EXPECT_EQ(3.0, vec[0].value());
  EXPECT_EQ(0, vec[0].derivatives().size());
  EXPECT_EQ(4.0, vec[1].value());
  EXPECT_EQ(0, vec[1].derivatives().size());
}

// Tests that the convenience accessors for a DiscreteValues that contains
// just one group work as documented.
GTEST_TEST(DiscreteValuesSingleGroupTest, ConvenienceSugar) {
  DiscreteValues<double> xd(BasicVector<double>::Make({42.0, 43.0}));
  EXPECT_EQ(2, xd.size());
  EXPECT_EQ(42.0, xd[0]);
  EXPECT_EQ(43.0, xd[1]);
  xd[0] = 100.0;
  EXPECT_EQ(100.0, xd.get_vector().GetAtIndex(0));
  xd.get_mutable_vector().SetAtIndex(1, 1000.0);
  EXPECT_EQ(1000.0, xd[1]);
}

}  // namespace
}  // namespace systems
}  // namespace drake
