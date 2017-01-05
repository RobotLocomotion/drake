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
    const double half_len = dut_->get_rod_length() / 2;
    const double r22 = std::sqrt(2) / 2;
    systems::ContinuousState<double>& v =
        *context_->get_mutable_continuous_state();

    // This configuration is symmetric to the default Painleve configuration.
    v[0] = -half_len * r22;
    v[1] = half_len * r22;
    v[2] = 3 * M_PI / 4.0;
    v[3] = 1.0;
    v[4] = 0.0;
    v[5] = 0.0;
  }

  std::unique_ptr<Painleve<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

/// Tests parameter getting and setting.
TEST_F(PainleveTest, Parameters) {
  // Set parameters to non-default values.
  const double g = -1.0;
  const double mass = 0.125;
  const double mu = 0.5;
  const double ell = 0.0625;
  const double J = 0.25;
  dut_->set_gravitational_acceleration(g);
  dut_->set_rod_mass(mass);
  dut_->set_mu_coulomb(mu);
  dut_->set_rod_length(ell);
  dut_->set_rod_moment_of_inertia(J);
  EXPECT_EQ(dut_->get_gravitational_acceleration(), g);
  EXPECT_EQ(dut_->get_rod_mass(), mass);
  EXPECT_EQ(dut_->get_mu_coulomb(), mu);
  EXPECT_EQ(dut_->get_rod_length(), ell);
  EXPECT_EQ(dut_->get_rod_moment_of_inertia(), J);
}

/// Verify the Painleve configuration occurs.
TEST_F(PainleveTest, Inconsistent) {
  EXPECT_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()),
               std::runtime_error);
}

// Verify the second Painleve configuration occurs.
TEST_F(PainleveTest, Inconsistent2) {
  SetSecondInitialConfig();
  EXPECT_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()),
               std::runtime_error);
}

// Verify that the (non-impacting) Painleve configuration does not result in a
// state change.
TEST_F(PainleveTest, ImpactNoChange) {
  // Setup state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec));
  dut_->HandleImpact(*context_, &new_cstate);
  for (int i = 0; i < state_dim; ++i)
    EXPECT_EQ(new_cstate[i], (*context_->get_continuous_state())[i]);
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveTest, InfFrictionImpactThenNoImpact) {
  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim / 2, state_dim / 2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to infinite. This forces the Painleve code
  // to go through the first impact path.
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i = 0; i < state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveTest, NoFrictionImpactThenNoImpact) {
  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim / 2, state_dim / 2, 0);

  // Set the initial state to be impacting
  systems::ContinuousState<double>& v =
      *context_->get_mutable_continuous_state();
  v[3] = -1.0;

  // Set the coefficient of friction to zero. This forces the Painleve code
  // to go through the second impact path.
  dut_->set_mu_coulomb(0.0);

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i = 0; i < state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

// Verify that no exceptions thrown for a non-sliding configuration.
TEST_F(PainleveTest, NoSliding) {
  const double half_len = dut_->get_rod_length() / 2;
  const double r22 = std::sqrt(2) / 2;
  systems::ContinuousState<double>& v =
      *context_->get_mutable_continuous_state();

  // Set the coefficient of friction to zero (triggering the case on the
  // edge of the friction cone).
  dut_->set_mu_coulomb(0.0);

  // This configuration has no sliding velocity.
  v[0] = -half_len * r22;
  v[1] = half_len * r22;
  v[2] = 3 * M_PI / 4.0;
  v[3] = 0.0;
  v[4] = 0.0;
  v[5] = 0.0;

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()));

  // Set the coefficient of friction to effective no-slip (triggering the
  // case strictly inside the friction cone).
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()));
}

// Test multiple (two-point) contact configurations.
TEST_F(PainleveTest, MultiPoint) {
  systems::ContinuousState<double>& v =
      *context_->get_mutable_continuous_state();

  // This configuration has no sliding velocity. It should throw no exceptions.
  v[0] = 0;
  v[1] = 0;
  v[2] = 0;
  v[3] = 0.0;
  v[4] = 0.0;
  v[5] = 0.0;
  EXPECT_NO_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()));

  // This configuration has sliding velocity. It should throw an exception.
  v[0] = 0;
  v[1] = 0;
  v[2] = 0;
  v[3] = 1.0;
  v[4] = 0.0;
  v[5] = 0.0;
  EXPECT_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()),
               std::logic_error);
}

/// Verify that Painleve configuration does not result in a state change.
TEST_F(PainleveTest, ImpactNoChange2) {
  SetSecondInitialConfig();

  // Setup state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec));
  dut_->HandleImpact(*context_, &new_cstate);
  for (int i = 0; i < state_dim; ++i)
    EXPECT_EQ(new_cstate[i], (*context_->get_continuous_state())[i]);
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveTest, InfFrictionImpactThenNoImpact2) {
  SetSecondInitialConfig();

  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim / 2, state_dim / 2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to infinite. This forces the Painleve code
  // to go through the first impact path.
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i = 0; i < state_dim; ++i)
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
                                              state_dim / 2, state_dim / 2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to zero. This forces the Painleve code
  // to go through the second impact path.
  dut_->set_mu_coulomb(0.0);

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i = 0; i < state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace painleve
}  // namespace drake
