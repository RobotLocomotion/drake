#include "drake/systems/framework/parameters.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/pack_value.h"

namespace drake {
namespace systems {
namespace {

class ParametersTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::vector<std::unique_ptr<BasicVector<double>>> numeric;
    numeric.push_back(BasicVector<double>::Make({3.0, 6.0}));
    numeric.push_back(BasicVector<double>::Make({9.0, 12.0}));

    std::vector<std::unique_ptr<AbstractValue>> abstract;
    abstract.push_back(PackValue<int>(72));
    abstract.push_back(PackValue<int>(144));
    params_ = std::make_unique<Parameters<double>>(
        std::move(numeric), std::move(abstract));
  }

  std::unique_ptr<Parameters<double>> params_;
};

TEST_F(ParametersTest, Numeric) {
  EXPECT_EQ(3.0, params_->get_numeric_parameter(0)->GetAtIndex(0));
  EXPECT_EQ(6.0, params_->get_numeric_parameter(0)->GetAtIndex(1));
  EXPECT_EQ(9.0, params_->get_numeric_parameter(1)->GetAtIndex(0));
  EXPECT_EQ(12.0, params_->get_numeric_parameter(1)->GetAtIndex(1));

  params_->get_mutable_numeric_parameter(0)->SetAtIndex(1, 42.0);
  EXPECT_EQ(42.0, params_->get_numeric_parameter(0)->GetAtIndex(1));
}

TEST_F(ParametersTest, Abstract) {
  EXPECT_EQ(72, UnpackIntValue(params_->get_abstract_parameter(0)));
  EXPECT_EQ(144, params_->template get_abstract_parameter<int>(1));
  params_->template get_mutable_abstract_parameter<int>(1) = 512;
  EXPECT_EQ(512, UnpackIntValue(params_->get_abstract_parameter(1)));
}

TEST_F(ParametersTest, Clone) {
  // Test that data is copied into the clone.
  auto clone = params_->Clone();
  EXPECT_EQ(3.0, clone->get_numeric_parameter(0)->GetAtIndex(0));
  EXPECT_EQ(72, UnpackIntValue(clone->get_abstract_parameter(0)));
  EXPECT_EQ(144, UnpackIntValue(clone->get_abstract_parameter(1)));

  // Test that changes to the clone don't write through to the original.
  // - numeric
  clone->get_mutable_numeric_parameter(0)->SetAtIndex(1, 42.0);
  EXPECT_EQ(6.0, params_->get_numeric_parameter(0)->GetAtIndex(1));
  // - abstract
  clone->get_mutable_abstract_parameter(0).SetValue<int>(256);
  EXPECT_EQ(72, UnpackIntValue(params_->get_abstract_parameter(0)));
}

}  // namespace
}  // namespace systems
}  // namespace drake
