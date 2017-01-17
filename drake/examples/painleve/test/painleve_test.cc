#include "drake/examples/painleve/painleve.h"
#include "drake/systems/analysis/simulator.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

using namespace drake::systems;

namespace drake {
namespace painleve {
namespace {

/// Class for testing the Painleve Paradox example using a piecewise DAE
/// approach.
class PainleveDAETest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Painleve<double>>();
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  std::unique_ptr<ContinuousState<double>> CreateNewContinuousState()
                                                                         const {
    const int state_dim = 6;
    auto cstate_vec = std::make_unique<BasicVector<double>>(state_dim);
    return std::make_unique<ContinuousState<double>>(
        std::move(cstate_vec), state_dim / 2, state_dim / 2, 0);
  }

  systems::VectorBase<double>* continuous_state() {
    return context_->get_mutable_continuous_state_vector();
  }

  // Sets a secondary initial Painlevé configuration.
  void SetSecondInitialConfig() {
    // Set the configuration to an inconsistent (Painlevé) type state with
    // the rod at a 135 degree counter-clockwise angle with respect to the
    // x-axis. The rod in [Stewart, 2000] is at a 45 degree counter-clockwise
    // angle with respect to the x-axis.
    // * [Stewart, 2000]  D. Stewart, "Rigid-Body Dynamics with Friction and
    //                    Impact. SIAM Rev., 42(1), 3-39, 2000.
    using std::sqrt;
    const double half_len = dut_->get_rod_length() / 2;
    const double r22 = std::sqrt(2) / 2;
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[0] = -half_len * r22;
    xc[1] = half_len * r22;
    xc[2] = 3 * M_PI / 4.0;
    xc[3] = 1.0;
    xc[4] = 0.0;
    xc[5] = 0.0;
  }

  // Sets the rod to an arbitrary impacting state.
  void SetImpactingState() {
    // This state is identical to that obtained from SetSecondInitialConfig()
    // but with the vertical component of velocity set such that the state
    // corresponds to an impact.
    SetSecondInitialConfig();
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[4] = -1.0;
  }

  std::unique_ptr<Painleve<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

/// Class for testing the Painleve Paradox example using a first order time
/// stepping approach.
class PainleveTimeSteppingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double dt = 1e-3;
    dut_ = std::make_unique<Painleve<double>>(dt);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
  }

  systems::BasicVector<double>* discrete_state() {
    return context_->get_mutable_discrete_state(0);
  }

  // Sets a secondary initial Painleve configuration.
  void SetSecondInitialConfig() {
    // Set the configuration to an inconsistent (Painlevé) type state with
    // the rod at a 135 degree counter-clockwise angle with respect to the
    // x-axis. The rod in [Stewart, 2000] is at a 45 degree counter-clockwise
    // angle with respect to the x-axis.
    // * [Stewart, 2000]  D. Stewart, "Rigid-Body Dynamics with Friction and
    //                    Impact. SIAM Rev., 42(1), 3-39, 2000.
    using std::sqrt;
    const double half_len = dut_->get_rod_length() / 2;
    const double r22 = std::sqrt(2) / 2;
    auto xd = discrete_state()->get_mutable_value();

    xd[0] = -half_len * r22;
    xd[1] = half_len * r22;
    xd[2] = 3 * M_PI / 4.0;
    xd[3] = 1.0;
    xd[4] = 0.0;
    xd[5] = 0.0;
  }

  std::unique_ptr<Painleve<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

// Checks that the output port represents the state.
TEST_F(PainleveDAETest, Output) {
  const ContinuousState<double>& xc = *context_->get_continuous_state();
  std::unique_ptr<SystemOutput<double>> output =
      dut_->AllocateOutput(*context_);
  dut_->CalcOutput(*context_, output.get());
  for (int i=0; i< xc.size(); ++i)
    EXPECT_EQ(xc[i], output->get_vector_data(0)->get_value()(i));
}

// Verifies that setting dut to an impacting state actually results in an
// impacting state.
TEST_F(PainleveDAETest, ImpactingState) {
  SetImpactingState();
  EXPECT_TRUE(dut_->IsImpacting(*context_));
}

// Tests parameter getting and setting.
TEST_F(PainleveDAETest, Parameters) {
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

// Verify that impact handling works as expected.
TEST_F(PainleveDAETest, ImpactWorks) {
  // Set writable state.
  std::unique_ptr<ContinuousState<double>> new_cstate =
      CreateNewContinuousState();

  // Cause the initial state to be impacting, with center of mass directly
  // over the point of contact.
  const double half_len = dut_->get_rod_length() / 2;
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = -1.0;
  xc[5] = 0.0;
  EXPECT_TRUE(dut_->IsImpacting(*context_));

  // Handle the impact.
  dut_->HandleImpact(*context_, new_cstate.get());
  context_->get_mutable_continuous_state()->SetFrom(*new_cstate);

  // Verify that the state has been modified such that the body is no longer
  // in an impacting state and the configuration has not been modified.
  const double tol = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(xc[0], 0.0, tol);
  EXPECT_NEAR(xc[1], half_len, tol);
  EXPECT_NEAR(xc[2], M_PI_2, tol);
  EXPECT_NEAR(xc[3], 0.0, tol);
  EXPECT_NEAR(xc[4], 0.0, tol);
  EXPECT_NEAR(xc[5], 0.0, tol);
}

// Verify that derivatives match what we expect from a non-inconsistent,
// ballistic configuration.
TEST_F(PainleveDAETest, ConsistentDerivativesBallistic) {
  // Set the initial state to ballistic motion.
  const double half_len = dut_->get_rod_length() / 2;
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = 10*half_len;
  xc[2] = M_PI_2;
  xc[3] = 1.0;
  xc[4] = 2.0;
  xc[5] = 3.0;

  // Calculate the derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify that the derivatives match what we expect for this non-inconsistent
  // ballistic system.
  const double tol = std::numeric_limits<double>::epsilon();
  const double g = dut_->get_gravitational_acceleration();
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);  // qdot = v ...
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);  // ... for this ...
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);  // ... system.
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);   // Zero horizontal acceleration.
  EXPECT_NEAR((*derivatives_)[4], g, tol);     // Gravitational acceleration.
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);   // Zero rotational acceleration.
}

// Verify that derivatives match what we expect from a non-inconsistent
// contacting configuration.
TEST_F(PainleveDAETest, ConsistentDerivativesContacting) {
  // Set the initial state to sustained contact with zero tangential velocity
  // at the point of contact.
  const double half_len = dut_->get_rod_length() / 2;
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;

  // Calculate the derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify that derivatives match what we expect from a non-inconsistent
  // contacting configuration. In this case, there is no initial sliding,
  // velocity and the rod is oriented vertically, so we expect no sliding
  // to begin to occur.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);

  // Set the coefficient of friction to zero, update the sliding velocity,
  // and try again. Derivatives should be exactly the same because no frictional
  // force can be applied.
  xc[3] = -1.0;
  dut_->set_mu_coulomb(0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);
}

// Verify the inconsistent (Painlevé Paradox) configuration occurs.
TEST_F(PainleveDAETest, Inconsistent) {
  EXPECT_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()),
               std::runtime_error);
}

// Verify the second inconsistent (Painlevé Paradox) configuration occurs.
TEST_F(PainleveDAETest, Inconsistent2) {
  SetSecondInitialConfig();
  EXPECT_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()),
               std::runtime_error);
}

// Verify that the (non-impacting) Painlevé configuration does not result in a
// state change.
TEST_F(PainleveDAETest, ImpactNoChange) {
  // Set state.
  std::unique_ptr<ContinuousState<double>> new_cstate =
      CreateNewContinuousState();
  EXPECT_FALSE(dut_->IsImpacting(*context_));
  dut_->HandleImpact(*context_, new_cstate.get());
  EXPECT_TRUE(CompareMatrices(new_cstate->get_vector().CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying the impact model to an impacting configuration results
// in a non-impacting configuration. This test exercises the model for the case
// where impulses that yield tangential sticking lie within the friction cone.
TEST_F(PainleveDAETest, InfFrictionImpactThenNoImpact) {
  // Set writable state.
  std::unique_ptr<ContinuousState<double>> new_cstate =
      CreateNewContinuousState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to infinite. This forces the Painlevé code
  // to go through the first impact path (impulse within the friction cone).
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, new_cstate.get());
  context_->get_mutable_continuous_state()->SetFrom(*new_cstate);
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, new_cstate.get());
  EXPECT_TRUE(CompareMatrices(new_cstate->get_vector().CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying an impact model to an impacting state results in a
// non-impacting state. This test exercises the model for the case
// where impulses that yield tangential sticking lie outside the friction cone.
TEST_F(PainleveDAETest, NoFrictionImpactThenNoImpact) {
  // Set the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to zero. This forces the Painlevé code
  // to go through the second impact path (impulse corresponding to sticking
  // friction post-impact lies outside of the friction cone).
  dut_->set_mu_coulomb(0.0);

  // Handle the impact and copy the result to the context.
  std::unique_ptr<ContinuousState<double>> new_cstate =
      CreateNewContinuousState();
  dut_->HandleImpact(*context_, new_cstate.get());
  context_->get_mutable_continuous_state()->SetFrom(*new_cstate);
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Do one more impact- there should now be no change.
  // Verify that there is no further change from this second impact.
  dut_->HandleImpact(*context_, new_cstate.get());
  EXPECT_TRUE(CompareMatrices(new_cstate->get_vector().CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that no exceptions thrown for a non-sliding configuration.
TEST_F(PainleveDAETest, NoSliding) {
  const double half_len = dut_->get_rod_length() / 2;
  const double r22 = std::sqrt(2) / 2;
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();

  // Set the coefficient of friction to zero (triggering the case on the
  // edge of the friction cone).
  dut_->set_mu_coulomb(0.0);

  // This configuration has no sliding velocity.
  xc[0] = -half_len * r22;
  xc[1] = half_len * r22;
  xc[2] = 3 * M_PI / 4.0;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()));

  // Set the coefficient of friction to effective no-slip (triggering the
  // case strictly inside the friction cone).
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
}

// Test multiple (two-point) contact configurations.
TEST_F(PainleveDAETest, MultiPoint) {
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();

  // This configuration has no sliding velocity. It should throw no exceptions.
  const double tol = std::numeric_limits<double>::epsilon();
  xc[0] = 0.0;
  xc[1] = 0.0;
  xc[2] = 0.0;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  for (int i=0; i< derivatives_->size(); ++i)
    EXPECT_NEAR((*derivatives_)[i], 0.0, tol);

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // This configuration has sliding velocity. It should throw an exception.
  xc[0] = 0;
  xc[1] = 0;
  xc[2] = 0;
  xc[3] = 1.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  EXPECT_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()),
               std::logic_error);

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));
}

// Verify that the Painlevé configuration does not correspond to an impacting
// state.
TEST_F(PainleveDAETest, ImpactNoChange2) {
  SetSecondInitialConfig();

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Set writable state.
  std::unique_ptr<ContinuousState<double>> new_cstate =
      CreateNewContinuousState();
  dut_->HandleImpact(*context_, new_cstate.get());
  EXPECT_TRUE(CompareMatrices(new_cstate->get_vector().CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying the impact model to an impacting state results
// in a non-impacting state.
TEST_F(PainleveDAETest, InfFrictionImpactThenNoImpact2) {
  // Set writable state.
  std::unique_ptr<ContinuousState<double>> new_cstate =
      CreateNewContinuousState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to infinite. This forces the Painlevé code
  // to go through the first impact path.
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, new_cstate.get());
  context_->get_mutable_continuous_state()->SetFrom(*new_cstate);

  // Verify the state no longer corresponds to an impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, new_cstate.get());
  EXPECT_TRUE(CompareMatrices(new_cstate->get_vector().CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying the impact model to an impacting state results in a
// non-impacting state.
TEST_F(PainleveDAETest, NoFrictionImpactThenNoImpact2) {
  // Set writable state.
  std::unique_ptr<ContinuousState<double>> new_cstate =
      CreateNewContinuousState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to zero. This forces the Painlevé code
  // to go through the second impact path.
  dut_->set_mu_coulomb(0.0);

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, new_cstate.get());
  context_->get_mutable_continuous_state()->SetFrom(*new_cstate);
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, new_cstate.get());
  EXPECT_TRUE(CompareMatrices(new_cstate->get_vector().CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verifies that rod in a ballistic state does not correspond to an impact.
TEST_F(PainleveDAETest, BallisticNoImpact) {
  // Set writable state.
  std::unique_ptr<ContinuousState<double>> new_cstate =
  CreateNewContinuousState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Move the rod upward vertically so that it is no longer impacting.
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[1] += 10.0;

  // Verify that no impact occurs.
  EXPECT_FALSE(dut_->IsImpacting(*context_));
}

/// Verify that Painleve Paradox system can be effectively simulated using
/// first-order time stepping approach.
TEST_F(PainleveTimeSteppingTest, TimeStepping) {
  // Set the initial state to an inconsistent configuration.
  SetSecondInitialConfig();

  // Init the simulator.
  systems::Simulator<double> simulator(*dut_, std::move(context_));

  // Integrate forward to a point where the rod should be at rest.
  const double t_final = 2.5;
  simulator.StepTo(t_final);

  // Get angular orientation and velocity.
  const auto v = simulator.get_context().get_discrete_state(0)->get_value();
  const double theta = v(2);
  const double theta_dot = v(5);

  // After sufficiently long, theta should be 0 or M_PI and the velocity
  // should be nearly zero.
  EXPECT_TRUE(std::fabs(theta) < 1e-6 || std::fabs(theta - M_PI) < 1e-6);
  EXPECT_NEAR(theta_dot, 0.0, 1e-6);

  // TODO(edrumwri): Introduce more extensive tests that cross-validates the
  // time-stepping based approach against the piecewise DAE-based approach.
}


}  // namespace
}  // namespace painleve
}  // namespace drake
