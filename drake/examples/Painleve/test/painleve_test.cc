#include "drake/examples/Painleve/painleve.h"

#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace painleve {
namespace {

class PainleveTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Painleve<double>>();
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  systems::VectorBase<double>* continuous_state() {
    return context_->get_mutable_continuous_state_vector();
  }

  std::unique_ptr<Painleve<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(PainleveTest, Inconsistent) {
  EXPECT_THROW(dut_->EvalTimeDerivatives(*context_, derivatives_.get()),
                   std::runtime_error);
}

}  // namespace
}  // namespace painleve
}  // namespace drake
