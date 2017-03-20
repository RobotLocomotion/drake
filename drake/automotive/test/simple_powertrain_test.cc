#include "drake/automotive/simple_powertrain.h"

#include <memory>

#include <gtest/gtest.h>

using std::make_unique;

namespace drake {
namespace automotive {
namespace {

// Specify a gain and a time constant for the lag model.
static constexpr double kPowertrainTimeConstant{0.2}; /* [s] */
static constexpr double kPowertrainGain{5.};          /* [N] */

class SimplePowertrainTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new SimplePowertrain<double>(kPowertrainTimeConstant,
                                            kPowertrainGain));
  }
  std::unique_ptr<SimplePowertrain<double>> dut_;  //< The device under test.
};

// Verifies the supplied data can be accessed.
TEST_F(SimplePowertrainTest, Accessors) {
  EXPECT_EQ(kPowertrainTimeConstant, dut_->get_time_constant());
  EXPECT_EQ(kPowertrainGain, dut_->get_gain());
}

// Verifies the correctness of the model.
TEST_F(SimplePowertrainTest, SystemMatrices) {
  // Check the properties of the system and the coefficients of the state and
  // output equations.
  EXPECT_EQ(false, dut_->HasAnyDirectFeedthrough());
  EXPECT_EQ(Vector1<double>(-1. / kPowertrainTimeConstant), dut_->A());
  EXPECT_EQ(Vector1<double>(kPowertrainGain), dut_->B());
  EXPECT_EQ(Vector1<double>(1. / kPowertrainTimeConstant), dut_->C());
}

}  // namespace
}  // namespace automotive
}  // namespace drake
