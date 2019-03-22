#include "drake/systems/framework/parameters.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

namespace drake {
namespace systems {
namespace {

class ParametersTest : public ::testing::Test {
 protected:
  void SetUp() override {
    params_ = MakeParams<double>();
    BasicVector<double>& p0 = params_->get_mutable_numeric_parameter(0);
    p0[0] = 3.0;
    p0[1] = 6.0;
    BasicVector<double>& p1 = params_->get_mutable_numeric_parameter(1);
    p1[0] = 9.0;
    p1[1] = 12.0;
  }

  template <typename T>
  static std::unique_ptr<Parameters<T>> MakeParams() {
    std::vector<std::unique_ptr<BasicVector<T>>> numeric;
    // Two numeric parameters, each vectors of length 2.
    numeric.push_back(std::make_unique<BasicVector<T>>(2));
    numeric.push_back(std::make_unique<BasicVector<T>>(2));

    // Two abstract parameters, each integers.
    std::vector<std::unique_ptr<AbstractValue>> abstract;
    abstract.push_back(PackValue<int>(72));
    abstract.push_back(PackValue<int>(144));
    return std::make_unique<Parameters<T>>(std::move(numeric),
                                           std::move(abstract));
  }

  std::unique_ptr<Parameters<double>> params_;
};

TEST_F(ParametersTest, Numeric) {
  ASSERT_EQ(2, params_->num_numeric_parameter_groups());
  ASSERT_EQ(2, params_->get_numeric_parameters().num_groups());
  EXPECT_EQ(3.0, params_->get_numeric_parameter(0)[0]);
  EXPECT_EQ(6.0, params_->get_numeric_parameter(0)[1]);
  EXPECT_EQ(9.0, params_->get_numeric_parameter(1)[0]);
  EXPECT_EQ(12.0, params_->get_numeric_parameter(1)[1]);

  params_->get_mutable_numeric_parameter(0)[1] = 42.0;
  EXPECT_EQ(42.0, params_->get_numeric_parameter(0)[1]);
}

TEST_F(ParametersTest, Abstract) {
  ASSERT_EQ(2, params_->num_abstract_parameters());
  ASSERT_EQ(2, params_->get_abstract_parameters().size());
  EXPECT_EQ(72, UnpackIntValue(params_->get_abstract_parameter(0)));
  EXPECT_EQ(144, params_->template get_abstract_parameter<int>(1));
  params_->template get_mutable_abstract_parameter<int>(1) = 512;
  EXPECT_EQ(512, UnpackIntValue(params_->get_abstract_parameter(1)));
}

TEST_F(ParametersTest, Clone) {
  // Test that data is copied into the clone.
  auto clone = params_->Clone();
  EXPECT_EQ(3.0, clone->get_numeric_parameter(0)[0]);
  EXPECT_EQ(72, UnpackIntValue(clone->get_abstract_parameter(0)));
  EXPECT_EQ(144, UnpackIntValue(clone->get_abstract_parameter(1)));

  // Test that changes to the clone don't write through to the original.
  // - numeric
  clone->get_mutable_numeric_parameter(0)[1] = 42.0;
  EXPECT_EQ(6.0, params_->get_numeric_parameter(0)[1]);
  // - abstract
  clone->get_mutable_abstract_parameter(0).set_value<int>(256);
  EXPECT_EQ(72, UnpackIntValue(params_->get_abstract_parameter(0)));
}

// Tests we can promote Parameters<double> to Parameters<symbolic::Expression>.
TEST_F(ParametersTest, SetSymbolicFromDouble) {
  auto symbolic_params = MakeParams<symbolic::Expression>();
  symbolic_params->SetFrom(*params_);

  // The numeric parameters have been converted to symbolic constants.
  const auto& p0 = symbolic_params->get_numeric_parameter(0);
  EXPECT_EQ("3", p0[0].to_string());
  EXPECT_EQ("6", p0[1].to_string());
  const auto& p1 = symbolic_params->get_numeric_parameter(1);
  EXPECT_EQ("9", p1[0].to_string());
  EXPECT_EQ("12", p1[1].to_string());

  // The abstract parameters have simply been cloned.
  EXPECT_EQ(72, UnpackIntValue(symbolic_params->get_abstract_parameter(0)));
  EXPECT_EQ(144, UnpackIntValue(symbolic_params->get_abstract_parameter(1)));
}

// Tests we can demote Parameters<symbolic::Expression> to Parameters<double>.
TEST_F(ParametersTest, SetDoubleFromSymbolic) {
  auto symbolic_params = MakeParams<symbolic::Expression>();
  auto& p0 = symbolic_params->get_mutable_numeric_parameter(0);
  auto& p1 = symbolic_params->get_mutable_numeric_parameter(1);
  p0[0] = 4.0;
  p0[1] = 7.0;
  p1[0] = 10.0;
  p1[1] = 13.0;

  params_->SetFrom(*symbolic_params);
  const auto& actual_p0 = params_->get_numeric_parameter(0);
  const auto& actual_p1 = params_->get_numeric_parameter(1);
  EXPECT_EQ(4.0, actual_p0[0]);
  EXPECT_EQ(7.0, actual_p0[1]);
  EXPECT_EQ(10.0, actual_p1[0]);
  EXPECT_EQ(13.0, actual_p1[1]);
}

// Constructs an AutoDiffXd parameters with the same dimensions as
// params_, and tests we can upconvert the latter into the former.
TEST_F(ParametersTest, SetAutodiffFromDouble) {
  auto autodiff_params = MakeParams<AutoDiffXd>();
  autodiff_params->SetFrom(*params_);

  // The numeric parameters have been converted to autodiff.
  const auto& p0 = autodiff_params->get_numeric_parameter(0);
  EXPECT_EQ(3.0, p0[0].value());
  EXPECT_EQ(6.0, p0[1].value());
  const auto& p1 = autodiff_params->get_numeric_parameter(1);
  EXPECT_EQ(9.0, p1[0].value());
  EXPECT_EQ(12.0, p1[1].value());

  // The abstract parameters have simply been cloned.
  EXPECT_EQ(72, UnpackIntValue(autodiff_params->get_abstract_parameter(0)));
  EXPECT_EQ(144, UnpackIntValue(autodiff_params->get_abstract_parameter(1)));
}

}  // namespace
}  // namespace systems
}  // namespace drake
