#include "drake/examples/rod2d/rod2d.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/analysis/simulator.h"

using drake::systems::VectorBase;
using drake::systems::BasicVector;
using drake::systems::ContinuousState;
using drake::systems::State;
using drake::systems::SystemOutput;
using drake::systems::AbstractValues;
using drake::systems::Simulator;
using drake::systems::Context;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace drake {
namespace examples {
namespace rod2d {
namespace {

/// Class for testing the Rod2D example using a piecewise DAE
/// approach.
class Rod2DDAETest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kPiecewiseDAE, 0.0);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Set a zero input force (this is the default).
    std::unique_ptr<BasicVector<double>> ext_input =
        std::make_unique<BasicVector<double>>(3);
    ext_input->SetAtIndex(0, 0.0);
    ext_input->SetAtIndex(1, 0.0);
    ext_input->SetAtIndex(2, 0.0);
    context_->FixInputPort(0, std::move(ext_input));
  }

  std::unique_ptr<State<double>> CloneState() const {
    return context_->CloneState();
  }

  VectorBase<double> *continuous_state() {
    return context_->get_mutable_continuous_state_vector();
  }

  // Sets a secondary initial Painlevé configuration.
  void SetSecondInitialConfig() {
    // Set the configuration to an inconsistent (Painlevé) type state with
    // the rod at a 135 degree counter-clockwise angle with respect to the
    // x-axis. The rod in [Stewart, 2000] is at a 45 degree counter-clockwise
    // angle with respect to the x-axis.
    // * [Stewart, 2000]  D. Stewart, "Rigid-Body Dynamics with Friction and
    //                    Impact". SIAM Rev., 42(1), 3-39, 2000.
    using std::sqrt;
    const double half_len = dut_->get_rod_half_length();
    const double r22 = std::sqrt(2) / 2;
    ContinuousState<double> &xc =
        *context_->get_mutable_continuous_state();
    xc[0] = -half_len * r22;
    xc[1] = half_len * r22;
    xc[2] = 3 * M_PI / 4.0;
    xc[3] = 1.0;
    xc[4] = 0.0;
    xc[5] = 0.0;

    // Indicate that the rod is in the single contact sliding mode.
    AbstractValues* abs_state =
        context_->get_mutable_state()->get_mutable_abstract_state();
    abs_state->get_mutable_value(0)
        .template GetMutableValue<Rod2D<double>::Mode>() =
        Rod2D<double>::kSlidingSingleContact;

    // Determine the point of contact.
    const double theta = xc[2];
    const int k = (std::sin(theta) > 0) ? -1 : 1;
    abs_state->get_mutable_value(1).template GetMutableValue<int>() = k;
  }

  // Sets the rod to a state that corresponds to ballistic motion.
  void SetBallisticState() {
    const double half_len = dut_->get_rod_half_length();
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[0] = 0.0;
    xc[1] = 10 * half_len;
    xc[2] = M_PI_2;
    xc[3] = 1.0;
    xc[4] = 2.0;
    xc[5] = 3.0;

    // Set the mode to ballistic.
    AbstractValues* abs_state =
        context_->get_mutable_state()->get_mutable_abstract_state();
    abs_state->get_mutable_value(0)
        .template GetMutableValue<Rod2D<double>::Mode>() =
        Rod2D<double>::kBallisticMotion;

    // Note: contact point mode is now arbitrary.
  }

  // Sets the rod to an arbitrary impacting state.
  void SetImpactingState() {
    // This state is identical to that obtained from SetSecondInitialConfig()
    // but with the vertical component of velocity set such that the state
    // corresponds to an impact.
    SetSecondInitialConfig();
    ContinuousState<double> &xc =
        *context_->get_mutable_continuous_state();
    xc[4] = -1.0;

    // Indicate that the rod is in the single contact sliding mode.
    AbstractValues* abs_state =
        context_->get_mutable_state()->get_mutable_abstract_state();
    abs_state->get_mutable_value(0)
        .template GetMutableValue<Rod2D<double>::Mode>() =
        Rod2D<double>::kSlidingSingleContact;

    // Determine the point of contact.
    const double theta = xc[2];
    const int k = (std::sin(theta) > 0) ? -1 : 1;
    abs_state->get_mutable_value(1).template GetMutableValue<int>() = k;
  }

  std::unique_ptr<Rod2D<double>> dut_;  //< The device under test.
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

// Checks that the output port represents the state.
TEST_F(Rod2DDAETest, Output) {
  const ContinuousState<double>& xc = *context_->get_continuous_state();
  std::unique_ptr<SystemOutput<double>> output =
      dut_->AllocateOutput(*context_);
  dut_->CalcOutput(*context_, output.get());
  for (int i = 0; i < xc.size(); ++i)
    EXPECT_EQ(xc[i], output->get_vector_data(0)->get_value()(i));
}

// Verifies that setting dut to an impacting state actually results in an
// impacting state.
TEST_F(Rod2DDAETest, ImpactingState) {
  SetImpactingState();
  EXPECT_TRUE(dut_->IsImpacting(*context_));
}

// Tests parameter getting and setting.
TEST_F(Rod2DDAETest, Parameters) {
  // Set parameters to non-default values.
  const double g = -1.0;
  const double mass = 0.125;
  const double mu = 0.5;
  const double h = 0.03125;
  const double J = 0.25;
  dut_->set_gravitational_acceleration(g);
  dut_->set_rod_mass(mass);
  dut_->set_mu_coulomb(mu);
  dut_->set_rod_half_length(h);
  dut_->set_rod_moment_of_inertia(J);
  EXPECT_EQ(dut_->get_gravitational_acceleration(), g);
  EXPECT_EQ(dut_->get_rod_mass(), mass);
  EXPECT_EQ(dut_->get_mu_coulomb(), mu);
  EXPECT_EQ(dut_->get_rod_half_length(), h);
  EXPECT_EQ(dut_->get_rod_moment_of_inertia(), J);
}

// Verify that impact handling works as expected.
TEST_F(Rod2DDAETest, ImpactWorks) {
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting, with center of mass directly
  // over the point of contact.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = -1.0;
  xc[5] = 0.0;

  // Set the mode variables.
  context_->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kStickingSingleContact;
  const double theta = xc[3];
  const int k = (std::sin(theta) > 0) ? -1 : 1;
  context_->template get_mutable_abstract_state<int>(1) = k;

  // Rod should not be impacting.
  EXPECT_TRUE(dut_->IsImpacting(*context_));

  // Handle the impact.
  dut_->HandleImpact(*context_, new_state.get());
  context_->get_mutable_state()->SetFrom(*new_state);

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
TEST_F(Rod2DDAETest, ConsistentDerivativesBallistic) {
  // Set the initial state to ballistic motion.
  SetBallisticState();

  // Calculate the derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify that the derivatives match what we expect for this non-inconsistent
  // ballistic system.
  const double tol = std::numeric_limits<double>::epsilon();
  const double g = dut_->get_gravitational_acceleration();
  const ContinuousState<double> &xc = *context_->get_continuous_state();
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);  // qdot = v ...
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);  // ... for this ...
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);  // ... system.
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);   // Zero horizontal acceleration.
  EXPECT_NEAR((*derivatives_)[4], g, tol);     // Gravitational acceleration.
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);   // Zero rotational acceleration.

  // Verify the mode is still ballistic.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kBallisticMotion);
}

// Verify that derivatives match what we expect from a non-inconsistent
// contacting configuration.
TEST_F(Rod2DDAETest, ConsistentDerivativesContacting) {
  // Set the initial state to sustained contact with zero tangential velocity
  // at the point of contact.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  context_->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kStickingSingleContact;

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
  context_->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kSlidingSingleContact;
  dut_->set_mu_coulomb(0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);
}

// Verify that derivatives match what we expect from a sticking contact
// configuration.
TEST_F(Rod2DDAETest, DerivativesContactingAndSticking) {
  // Set the initial state to sustained contact with zero tangential velocity
  // at the point of contact and the rod being straight up.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  context_->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kStickingSingleContact;

  // Set a constant horizontal input force, as if applied at the bottom of
  // the rod.
  std::unique_ptr<BasicVector<double>> ext_input =
      std::make_unique<BasicVector<double>>(3);
  const double f_x = 1.0;
  const double f_y = -1.0;
  ext_input->SetAtIndex(0, f_x);
  ext_input->SetAtIndex(1, f_y);
  ext_input->SetAtIndex(2, f_x * dut_->get_rod_half_length());
  const Vector3<double> fext = ext_input->CopyToVector();
  context_->FixInputPort(0, std::move(ext_input));

  // Set the coefficient of friction such that the contact forces are right
  // on the edge of the friction cone. Determine the predicted normal force
  // (this simple formula is dependent upon the upright rod configuration).
  const double mu_stick = f_x / (dut_->get_rod_mass() *
                                 -dut_->get_gravitational_acceleration() -
                                 f_y);
  dut_->set_mu_coulomb(mu_stick);

  // Calculate the derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify that derivatives match what we expect: sticking should continue.
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);

  // Set the coefficient of friction to 99.9% of the sticking value and then
  // verify that the contact state transitions from a sticking one to a
  // non-sticking one.
  const double mu_slide = 0.999 * mu_stick;
  dut_->set_mu_coulomb(mu_slide);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_GT((*derivatives_)[3], tol);  // horizontal accel. should be nonzero.

  // Set the coefficient of friction to zero and try again.
  context_->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kStickingSingleContact;
  dut_->set_mu_coulomb(0.0);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  // Rod should now accelerate in the direction of any external forces.
  EXPECT_NEAR((*derivatives_)[3], fext(0)/dut_->get_rod_mass(), tol);
  // There should still be no vertical acceleration.
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  // The moment caused by applying the force should result in a
  // counter-clockwise acceleration.
  EXPECT_NEAR((*derivatives_)[5],
              fext(2)/dut_->get_rod_moment_of_inertia(), tol);
}

// Verify the inconsistent (Painlevé Paradox) configuration occurs.
TEST_F(Rod2DDAETest, Inconsistent) {
  EXPECT_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()),
               std::runtime_error);
}

// Verify the second inconsistent (Painlevé Paradox) configuration occurs.
TEST_F(Rod2DDAETest, Inconsistent2) {
  SetSecondInitialConfig();

  EXPECT_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()),
               std::runtime_error);
}

// Verify that the (non-impacting) Painlevé configuration does not result in a
// state change.
TEST_F(Rod2DDAETest, ImpactNoChange) {
  // Set state.
  std::unique_ptr<State<double>> new_state = CloneState();
  EXPECT_FALSE(dut_->IsImpacting(*context_));
  dut_->HandleImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Verify that the mode is still sliding.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kSlidingSingleContact);
}

// Verify that applying the impact model to an impacting configuration results
// in a non-impacting configuration. This test exercises the model for the case
// where impulses that yield tangential sticking lie within the friction cone.
TEST_F(Rod2DDAETest, InfFrictionImpactThenNoImpact) {
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Verify that the state is in a sliding mode.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kSlidingSingleContact);

  // Set the coefficient of friction to infinite. This forces the rod code
  // to go through the first impact path (impulse within the friction cone).
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, new_state.get());
  context_->get_mutable_state()->SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Verify that the state is now in a sticking mode.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kStickingSingleContact);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying an impact model to an impacting state results in a
// non-impacting state. This test exercises the model for the case
// where impulses that yield tangential sticking lie outside the friction cone.
TEST_F(Rod2DDAETest, NoFrictionImpactThenNoImpact) {
  // Set the initial state to be impacting.
  SetImpactingState();

  // Verify that the state is in a sliding mode.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kSlidingSingleContact);

  // Set the coefficient of friction to zero. This forces the rod code
  // to go through the second impact path (impulse corresponding to sticking
  // friction post-impact lies outside of the friction cone).
  dut_->set_mu_coulomb(0.0);

  // Handle the impact and copy the result to the context.
  std::unique_ptr<State<double>> new_state = CloneState();
  dut_->HandleImpact(*context_, new_state.get());
  context_->get_mutable_state()->SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Verify that the state is still in a sliding mode.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kSlidingSingleContact);

  // Do one more impact- there should now be no change.
  // Verify that there is no further change from this second impact.
  dut_->HandleImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Verify that the state is still in a sliding mode.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kSlidingSingleContact);
}

// Verify that no exceptions thrown for a non-sliding configuration.
TEST_F(Rod2DDAETest, NoSliding) {
  const double half_len = dut_->get_rod_half_length();
  const double r22 = std::sqrt(2) / 2;
  ContinuousState<double> &xc =
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
  context_->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kStickingSingleContact;

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
TEST_F(Rod2DDAETest, MultiPoint) {
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
  context_->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kStickingTwoContacts;
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  for (int i = 0; i < derivatives_->size(); ++i)
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
  context_->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kSlidingTwoContacts;
  EXPECT_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()),
               std::logic_error);

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));
}

// Verify that the Painlevé configuration does not correspond to an impacting
// state.
TEST_F(Rod2DDAETest, ImpactNoChange2) {
  SetSecondInitialConfig();

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();
  dut_->HandleImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying the impact model to an impacting state results
// in a non-impacting state.
TEST_F(Rod2DDAETest, InfFrictionImpactThenNoImpact2) {
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to infinite. This forces the rod code
  // to go through the first impact path.
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, new_state.get());
  context_->get_mutable_state()->SetFrom(*new_state);

  // Verify the state no longer corresponds to an impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Verify that the state is now in a sticking mode.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kStickingSingleContact);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}


// Verify that applying the impact model to an impacting state results in a
// non-impacting state.
TEST_F(Rod2DDAETest, NoFrictionImpactThenNoImpact2) {
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Verify that the state is still in a sliding configuration.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kSlidingSingleContact);

  // Set the coefficient of friction to zero. This forces the rod code
  // to go through the second impact path.
  dut_->set_mu_coulomb(0.0);

  // Verify that the state is still in a sliding configuration.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kSlidingSingleContact);

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, new_state.get());
  context_->get_mutable_state()->SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Verify that the state is still in a sliding configuration.
  EXPECT_EQ(context_->template get_abstract_state<Rod2D<double>::Mode>(0),
            Rod2D<double>::kSlidingSingleContact);
}

// Verifies that rod in a ballistic state does not correspond to an impact.
TEST_F(Rod2DDAETest, BallisticNoImpact) {
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Move the rod upward vertically so that it is no longer impacting and
  // set the mode to ballistic motion.
  ContinuousState<double> &xc =
      *context_->get_mutable_continuous_state();
  xc[1] += 10.0;
  context_->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kBallisticMotion;

  // Verify that no impact occurs.
  EXPECT_FALSE(dut_->IsImpacting(*context_));
}

/// Class for testing the Rod 2D example using a first order time
/// stepping approach.
class Rod2DTimeSteppingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double dt = 1e-2;
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kTimeStepping, dt);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Set a zero input force (this is the default).
    std::unique_ptr<BasicVector<double>> ext_input =
        std::make_unique<BasicVector<double>>(3);
    ext_input->SetAtIndex(0, 0.0);
    ext_input->SetAtIndex(1, 0.0);
    ext_input->SetAtIndex(2, 0.0);
    context_->FixInputPort(0, std::move(ext_input));
  }

  BasicVector<double> *mutable_discrete_state() {
    return context_->get_mutable_discrete_state(0);
  }
// Sets a secondary initial Rod2D configuration.
  void SetSecondInitialConfig() {
    // Set the configuration to an inconsistent (Painlevé) type state with
    // the rod at a 135 degree counter-clockwise angle with respect to the
    // x-axis. The rod in [Stewart, 2000] is at a 45 degree counter-clockwise
    // angle with respect to the x-axis.
    // * [Stewart, 2000]  D. Stewart, "Rigid-Body Dynamics with Friction and
    //                    Impact". SIAM Rev., 42(1), 3-39, 2000.
    using std::sqrt;
    const double half_len = dut_->get_rod_half_length();
    const double r22 = std::sqrt(2) / 2;
    auto xd = mutable_discrete_state()->get_mutable_value();

    xd[0] = -half_len * r22;
    xd[1] = half_len * r22;
    xd[2] = 3 * M_PI / 4.0;
    xd[3] = 1.0;
    xd[4] = 0.0;
    xd[5] = 0.0;
  }

  std::unique_ptr<Rod2D<double>> dut_;  //< The device under test.
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

/// Verify that Rod 2D system eventually goes to rest using the
/// first-order time stepping approach (this tests expected meta behavior).
TEST_F(Rod2DTimeSteppingTest, RodGoesToRest) {
  // Set the initial state to an inconsistent configuration.
  SetSecondInitialConfig();

  // Init the simulator.
  Simulator<double> simulator(*dut_, std::move(context_));

  // Integrate forward to a point where the rod should be at rest.
  const double t_final = 10;
  simulator.StepTo(t_final);

  // Get angular orientation and velocity.
  const auto xd = simulator.get_context().get_discrete_state(0)->get_value();
  const double theta = xd(2);
  const double theta_dot = xd(5);

  // After sufficiently long, theta should be 0 or M_PI and the velocity
  // should be nearly zero.
  EXPECT_TRUE(std::fabs(theta) < 1e-6 || std::fabs(theta - M_PI) < 1e-6);
  EXPECT_NEAR(theta_dot, 0.0, 1e-6);
}

// This test checks to see whether a single semi-explicit step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system.
GTEST_TEST(Rod2DCrossValidationTest, OneStepSolutionSliding) {
  // Create two Rod2D systems.
  const double dt = 1e-1;
  Rod2D<double> ts(Rod2D<double>::SimulationType::kTimeStepping, dt);
  Rod2D<double> pdae(Rod2D<double>::SimulationType::kPiecewiseDAE, 0.0);

  // Set the coefficient of friction to a small value for both.
  const double mu = 0.01;
  ts.set_mu_coulomb(mu);
  pdae.set_mu_coulomb(mu);

  // Set "one step" constraint stabilization (not generally recommended, but
  // works for a single step) and small regularization.
  ts.set_cfm(std::numeric_limits<double>::epsilon());
  ts.set_erp(1.0);

  // Create contexts for both.
  std::unique_ptr<Context<double>> context_ts = ts.CreateDefaultContext();
  std::unique_ptr<Context<double>> context_pdae = pdae.CreateDefaultContext();

  // Set zero input forces for both.
  Vector3<double> fext(0, 0, 0);
  std::unique_ptr<BasicVector<double>> ext_input =
    std::make_unique<BasicVector<double>>(fext);
  context_ts->FixInputPort(0, std::move(ext_input));
  ext_input = std::make_unique<BasicVector<double>>(fext);
  context_pdae->FixInputPort(0, std::move(ext_input));

  // Init the simulator for the time stepping system.
  Simulator<double> simulator_ts(ts, std::move(context_ts));

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_ts.StepTo(dt);
  EXPECT_EQ(simulator_ts.get_num_discrete_updates(), 1);

  // Manually integrate the continuous state forward for the piecewise DAE
  // based approach.
  std::unique_ptr<ContinuousState<double>> f = pdae.AllocateTimeDerivatives();
  pdae.CalcTimeDerivatives(*context_pdae, f.get());
  auto xc = context_pdae->get_mutable_continuous_state_vector();
  xc->SetAtIndex(3, xc->GetAtIndex(3) + dt * ((*f)[3]));
  xc->SetAtIndex(4, xc->GetAtIndex(4) + dt * ((*f)[4]));
  xc->SetAtIndex(5, xc->GetAtIndex(5) + dt * ((*f)[5]));
  xc->SetAtIndex(0, xc->GetAtIndex(0) + dt * xc->GetAtIndex(3));
  xc->SetAtIndex(1, xc->GetAtIndex(1) + dt * xc->GetAtIndex(4));
  xc->SetAtIndex(2, xc->GetAtIndex(2) + dt * xc->GetAtIndex(5));

  // See whether the states are equal.
  const Context<double> &context_ts_new = simulator_ts.get_context();
  const auto &xd = context_ts_new.get_discrete_state(0)->get_value();

  // Check that the solution is nearly identical.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(xc->GetAtIndex(0), xd[0], tol);
  EXPECT_NEAR(xc->GetAtIndex(1), xd[1], tol);
  EXPECT_NEAR(xc->GetAtIndex(2), xd[2], tol);
  EXPECT_NEAR(xc->GetAtIndex(3), xd[3], tol);
  EXPECT_NEAR(xc->GetAtIndex(4), xd[4], tol);
  EXPECT_NEAR(xc->GetAtIndex(5), xd[5], tol);

  // TODO(edrumwri): Introduce more extensive tests that cross-validate the
  // time-stepping based approach against the piecewise DAE-based approach for
  // the case of sliding contacts at multiple points.
}

// This test checks to see whether a single semi-explicit step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system for a sticking contact scenario.
GTEST_TEST(Rod2DCrossValidationTest, OneStepSolutionSticking) {
  // Create two Rod2D systems.
  const double dt = 1e-1;
  Rod2D<double> ts(Rod2D<double>::SimulationType::kTimeStepping, dt);
  Rod2D<double> pdae(Rod2D<double>::SimulationType::kPiecewiseDAE, 0.0);

  // Set the coefficient of friction to a large value for both.
  const double mu = 100.0;
  ts.set_mu_coulomb(mu);
  pdae.set_mu_coulomb(mu);

  // Set "one step" constraint stabilization (not generally recommended, but
  // works for a single step) and small regularization.
  ts.set_cfm(std::numeric_limits<double>::epsilon());
  ts.set_erp(1.0);

  // Create contexts for both.
  std::unique_ptr<Context<double>> context_ts = ts.CreateDefaultContext();
  std::unique_ptr<Context<double>> context_pdae = pdae.CreateDefaultContext();

  // This configuration has no sliding velocity.
  const double half_len = pdae.get_rod_half_length();
  ContinuousState<double>& xc =
      *context_pdae->get_mutable_continuous_state();
  auto xd = context_ts->get_mutable_discrete_state(0)->get_mutable_value();
  xc[0] = xd[0] = 0.0;
  xc[1] = xd[1] = half_len;
  xc[2] = xd[2] = M_PI_2;
  xc[3] = xd[3] = 0.0;
  xc[4] = xd[4] = 0.0;
  xc[5] = xd[5] = 0.0;
  context_pdae->template get_mutable_abstract_state<Rod2D<double>::Mode>(0) =
      Rod2D<double>::kStickingSingleContact;

  // Set constant input forces for both.
  const double x = 1.0;
  Vector3<double> fext(x, 0, x * ts.get_rod_half_length());
  std::unique_ptr<BasicVector<double>> ext_input =
    std::make_unique<BasicVector<double>>(fext);
  context_ts->FixInputPort(0, std::move(ext_input));
  ext_input = std::make_unique<BasicVector<double>>(fext);
  context_pdae->FixInputPort(0, std::move(ext_input));

  // Init the simulator for the time stepping system.
  Simulator<double> simulator_ts(ts, std::move(context_ts));

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_ts.StepTo(dt);
  EXPECT_EQ(simulator_ts.get_num_discrete_updates(), 1);

  // Manually integrate the continuous state forward for the piecewise DAE
  // based approach.
  std::unique_ptr<ContinuousState<double>> f = pdae.AllocateTimeDerivatives();
  pdae.CalcTimeDerivatives(*context_pdae, f.get());
  xc[3] += + dt * ((*f)[3]);
  xc[4] += + dt * ((*f)[4]);
  xc[5] += + dt * ((*f)[5]);
  xc[0] += + dt * xc[3];
  xc[1] += + dt * xc[4];
  xc[2] += + dt * xc[5];

  // Check that the solution is nearly identical.
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(xc[0], xd[0], tol);
  EXPECT_NEAR(xc[1], xd[1], tol);
  EXPECT_NEAR(xc[2], xd[2], tol);
  EXPECT_NEAR(xc[3], xd[3], tol);
  EXPECT_NEAR(xc[4], xd[4], tol);
  EXPECT_NEAR(xc[5], xd[5], tol);
}

/// Class for testing the Rod 2D example using compliant contact
/// thus permitting integration as an ODE.
class Rod2DCompliantTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kCompliant, 0.0);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Using default compliant contact parameters.

    // Set a zero input force (this is the default).
    std::unique_ptr<BasicVector<double>> ext_input =
        std::make_unique<BasicVector<double>>(3);
    ext_input->SetAtIndex(0, 0.0);
    ext_input->SetAtIndex(1, 0.0);
    ext_input->SetAtIndex(2, 0.0);
    context_->FixInputPort(0, std::move(ext_input));
  }

  // Calculate time derivatives using the context member and writing to
  // the derivatives member.
  void CalcTimeDerivatives() {
    dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  }

  std::unique_ptr<State<double>> CloneState() const {
    return context_->CloneState();
  }

  // Return the state x,y,θ,xdot,ydot,θdot as a Vector6.
  Vector6d get_state() const {
    const ContinuousState<double>& xc = *context_->get_continuous_state();
    return Vector6d(xc.CopyToVector());
  }

  // Return d/dt state xdot,ydot,θdot,xddot,yddot,θddot as a Vector6.
  Vector6d get_state_dot() const {
    const ContinuousState<double>& xcd = *derivatives_;
    return Vector6d(xcd.CopyToVector());
  }

  // Sets the planar pose in the context.
  void set_pose(double x, double y, double theta) {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[0] = x; xc[1] = y; xc[2] = theta;
  }

  // Sets the planar velocity in the context.
  void set_velocity(double xdot, double ydot, double thetadot) {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[3] = xdot; xc[4] = ydot; xc[5] = thetadot;
  }

  // Returns planar pose derivative (should be planar velocities xdot,
  // ydot, thetadot).
  Vector3d get_pose_dot() const {
    const ContinuousState<double>& xcd = *derivatives_;
    return Vector3d(xcd[0], xcd[1], xcd[2]);
  }

  // Returns planar acceleration (xddot, yddot, thetaddot).
  Vector3d get_accelerations() const {
    const ContinuousState<double>& xcd = *derivatives_;
    return Vector3d(xcd[3], xcd[4], xcd[5]);
  }

  // Sets the rod to a state that corresponds to ballistic motion.
  void SetBallisticState() {
    const double half_len = dut_->get_rod_half_length();
    set_pose(0, 10*half_len, M_PI_2);
    set_velocity(1, 2, 3);
  }

  // Sets the rod to a perfectly vertical position in which either the left or
  // right endpoint is contacting, or a perfectly horizontal position in which
  // both endpoints are contacting. In all cases the penetration depth into the
  // halfplane is set to 1 cm, and the rod velocity is set to zero.
  // k=-1,0,1 -> left, both, right.
  void SetContactingState(int k) {
    DRAKE_DEMAND(-1 <= k && k <= 1);
    const double half_len = dut_->get_rod_half_length();
    const double penetration = 0.01;  // 1 cm
    set_pose(0, std::abs(k)*half_len - penetration, -k*M_PI_2);
    set_velocity(0, 0, 0);
  }

  std::unique_ptr<Rod2D<double>> dut_;  //< The device under test.
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

/// Verify that the compliant contact resists penetration.
TEST_F(Rod2DCompliantTest, ForcesHaveRightSign) {
  // We expect only roundoff errors, scaled by force magnitude (~1e-14).
  const double kTightTol = 50 * std::numeric_limits<double>::epsilon();

  SetContactingState(-1);  // left

  Vector3d F_Ro_W_left = dut_->CalcCompliantContactForces(*context_);

  CalcTimeDerivatives();
  Vector6d xcd = get_state_dot();

  EXPECT_EQ(xcd[0], 0);  // xdot, ydot, thetadot
  EXPECT_EQ(xcd[1], 0);
  EXPECT_EQ(xcd[2], 0);

  // Total acceleration is gravity plus acceleration due to contact forces;
  // extract just the contact contribution. It should point up!
  const double a_contact = xcd[4] - dut_->get_gravitational_acceleration();

  // Vertical acceleration is >> 1; just checking for correct sign. Rod is
  // vertical so horizontal and angular accelerations are zero.
  EXPECT_NEAR(xcd[3], 0, kTightTol);  // no x acceleration
  EXPECT_GT(a_contact, 1.);           // + y acceleration
  EXPECT_NEAR(xcd[5], 0, kTightTol);  // no angular acceleration

  // Now add some downward velocity; that should *increase* the force we
  // calculated above from just penetration. We're checking that the sign is
  // correct by making the penetration rate large enough to at least double
  // the overall force.
  set_velocity(0, -10, 0);
  Vector3d F_Ro_W_ldown = dut_->CalcCompliantContactForces(*context_);
  EXPECT_GT(F_Ro_W_ldown[1], 2*F_Ro_W_left[1]);  // Did it double?

  // An extreme upwards velocity should be a "pull out" situation resulting
  // in (exactly) zero force rather than a negative force.
  set_velocity(0, 1000, 0);
  Vector3d F_Ro_W_lup = dut_->CalcCompliantContactForces(*context_);
  EXPECT_TRUE(F_Ro_W_lup == Vector3d::Zero());

  // Sliding -x should produce a +x friction force and a positive torque;
  // no effect on y force.
  set_velocity(-10, 0, 0);
  Vector3d F_Ro_W_nx = dut_->CalcCompliantContactForces(*context_);
  EXPECT_GT(F_Ro_W_nx[0], 1.);
  EXPECT_NEAR(F_Ro_W_nx[1], F_Ro_W_left[1], kTightTol);
  EXPECT_GT(F_Ro_W_nx[2], 1.);

  // Sliding +x should produce a -x friction force and a negative torque;
  // no effect on y force.
  set_velocity(10, 0, 0);
  Vector3d F_Ro_W_px = dut_->CalcCompliantContactForces(*context_);
  EXPECT_LT(F_Ro_W_px[0], -1.);
  EXPECT_NEAR(F_Ro_W_px[1], F_Ro_W_left[1], kTightTol);
  EXPECT_LT(F_Ro_W_px[2], -1.);


  SetContactingState(1);  // Right should behave same as left.
  Vector3d F_Ro_W_right = dut_->CalcCompliantContactForces(*context_);
  EXPECT_TRUE(F_Ro_W_right.isApprox(F_Ro_W_left, kTightTol));

  // With both ends in contact the force should double and there should
  // be zero moment.
  SetContactingState(0);
  Vector3d F_Ro_W_both = dut_->CalcCompliantContactForces(*context_);
  EXPECT_TRUE(F_Ro_W_both.isApprox(F_Ro_W_left+F_Ro_W_right, kTightTol));
  EXPECT_NEAR(F_Ro_W_both[2], 0., kTightTol);
}

}  // namespace
}  // namespace rod2d
}  // namespace examples
}  // namespace drake
