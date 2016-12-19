#include "drake/examples/painleve/painleve.h"

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

  // Sets a secondary initial Painleve configuration.
  void SetSecondInitialConfig() {
    const double half_len = dut_->get_rod_length()/2;
    const double r22 = std::sqrt(2)/2;
    systems::ContinuousState<double>& v = *context_->
                                               get_mutable_continuous_state();

    // This configuration is symmetric to the default Painleve configuration.
    v[0] = -half_len*r22;
    v[1] = half_len*r22;
    v[2] = 3*M_PI/4.0;
    v[3] = 1.0;
    v[4] = 0.0;
    v[5] = 0.0;
  }

  std::unique_ptr<Painleve<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

/// Verify the Painleve configuration occurs.
TEST_F(PainleveTest, Inconsistent) {
  EXPECT_THROW(dut_->EvalTimeDerivatives(*context_, derivatives_.get()),
                   std::runtime_error);
}

/// Verify that Painleve configuration does not result in a state change.
TEST_F(PainleveTest, ImpactNoChange) {
  // Setup state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec));
  dut_->HandleImpact(*context_, &new_cstate);
  for (int i=0; i< state_dim; ++i)
    EXPECT_EQ(new_cstate[i], (*context_->get_continuous_state())[i]);
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveTest, InfFrictionImpactThenNoImpact) {
  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim/2, state_dim/2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to infinite. This forces the Painleve code
  // to go through the first impact path.
  dut_->set_mu_Coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i=0; i< state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveTest, NoFrictionImpactThenNoImpact) {
  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim/2, state_dim/2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to zero. This forces the Painleve code
  // to go through the second impact path.
  dut_->set_mu_Coulomb(0.0);

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i=0; i< state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

// Verify that no exceptions thrown for a non-sliding configuration.
TEST_F(PainleveTest, NoSliding) {
  const double half_len = dut_->get_rod_length()/2;
  const double r22 = std::sqrt(2)/2;
  systems::ContinuousState<double>& v = *context_->
                                             get_mutable_continuous_state();

  // This configuration has no sliding velocity. 
  v[0] = -half_len*r22;
  v[1] = half_len*r22;
  v[2] = 3*M_PI/4.0;
  v[3] = 0.0;
  v[4] = 0.0;
  v[5] = 0.0;

  // No exceptions should be thrown.
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());
}

/// Verify the second Painleve configuration occurs.
TEST_F(PainleveTest, Inconsistent2) {
  SetSecondInitialConfig();
  EXPECT_THROW(dut_->EvalTimeDerivatives(*context_, derivatives_.get()),
                   std::runtime_error);
}

/// Verify that Painleve configuration does not result in a state change.
TEST_F(PainleveTest, ImpactNoChange2) {
  SetSecondInitialConfig();

  // Setup state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec));
  dut_->HandleImpact(*context_, &new_cstate);
  for (int i=0; i< state_dim; ++i)
    EXPECT_EQ(new_cstate[i], (*context_->get_continuous_state())[i]);
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveTest, InfFrictionImpactThenNoImpact2) {
  SetSecondInitialConfig();

  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim/2, state_dim/2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to infinite. This forces the Painleve code
  // to go through the first impact path.
  dut_->set_mu_Coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i=0; i< state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveTest, NoFrictionImpactThenNoImpact2) {
  SetSecondInitialConfig();

  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim/2, state_dim/2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to zero. This forces the Painleve code
  // to go through the second impact path.
  dut_->set_mu_Coulomb(0.0);

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i=0; i< state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace painleve
}  // namespace drake
