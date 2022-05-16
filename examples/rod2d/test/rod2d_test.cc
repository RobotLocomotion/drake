#include "drake/examples/rod2d/rod2d.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/multibody/constraint/constraint_problem_data.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/analysis/simulator.h"

using drake::multibody::constraint::ConstraintAccelProblemData;
using drake::multibody::constraint::ConstraintVelProblemData;
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
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace rod2d {

/// Class for testing the Rod2D example using a piecewise DAE
/// approach.
class Rod2DDAETest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SystemType::kPiecewiseDAE, 0.0);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
    derivatives_ = dut_->AllocateTimeDerivatives();

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Set cfm to be very small, so that the complementarity problems are
    // well conditioned but the system is still nearly perfectly rigid. erp is
    // to be set to 0.2 (a reasonable default).
    const double cfm = 100 * std::numeric_limits<double>::epsilon();
    const double erp = 0.2;
    dut_->SetStiffnessAndDissipation(cfm, erp);

    // Set a zero input force (this is the default).
    const Vector3<double> ext_input(0, 0, 0);
    dut_->get_input_port(0).FixValue(context_.get(), ext_input);
  }

  std::unique_ptr<State<double>> CloneState() const {
    return context_->CloneState();
  }

  VectorBase<double>& continuous_state() {
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
    ContinuousState<double>& xc =
        context_->get_mutable_continuous_state();
    xc[0] = -half_len * r22;
    xc[1] = half_len * r22;
    xc[2] = 3 * M_PI / 4.0;
    xc[3] = 1.0;
    xc[4] = 0.0;
    xc[5] = 0.0;

    // TODO(edrumwri): Put the rod into the single contact sliding mode.
  }

  // Sets the rod to a state that corresponds to ballistic motion.
  void SetBallisticState() {
    const double half_len = dut_->get_rod_half_length();
    ContinuousState<double>& xc =
        context_->get_mutable_continuous_state();
    xc[0] = 0.0;
    xc[1] = 10 * half_len;
    xc[2] = M_PI_2;
    xc[3] = 1.0;
    xc[4] = 2.0;
    xc[5] = 3.0;

    // TODO(edrumwri): Set the mode to ballistic.
  }

  // Sets the rod to an interpenetrating configuration without modifying the
  // velocity or any mode variables.
  void SetInterpenetratingConfig() {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;    // com horizontal position
    xc[1] = -1.0;   // com vertical position
    xc[2] = 0.0;    // rod rotation
  }

  // Sets the rod to a resting horizontal configuration without modifying the
  // mode variables.
  void SetRestingHorizontalConfig() {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;     // com horizontal position
    xc[1] = 0.0;     // com vertical position
    xc[2] = 0.0;     // rod rotation
    xc[3] = xc[4] = xc[5] = 0.0;   // velocity variables
  }

  // Sets the rod to a resting vertical configuration without modifying the
  // mode variables.
  void SetRestingVerticalConfig() {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc[0] = 0.0;                             // com horizontal position
    xc[1] = dut_->get_rod_half_length();     // com vertical position
    xc[2] = M_PI_2;                          // rod rotation
    xc[3] = xc[4] = xc[5] = 0.0;             // velocity variables
  }

  // Sets the contact mode such that the left endpoint is contacting. The caller
  // sets whether or not the contact should be considered to be sliding.
  void SetLeftEndpointContacting(State<double>* state, bool sliding) {
    // TODO(edrumwri): Implement this.
  }

  // Sets the rod to an arbitrary impacting state.
  void SetImpactingState() {
    // This state is identical to that obtained from SetSecondInitialConfig()
    // but with the vertical component of velocity set such that the state
    // corresponds to an impact.
    SetSecondInitialConfig();
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc[4] = -1.0;    // com horizontal velocity

    // TODO(edrumwri): Set the rod mode to indicate that it is in the single
    // contact sliding mode.
  }

  // Computes rigid impact data.
  void CalcRigidImpactVelProblemData(
      ConstraintVelProblemData<double>* data) {
    // Get the points of contact.
    std::vector<Vector2d> contacts;
    dut_->GetContactPoints(*context_, &contacts);

    // Compute the problem data.
    dut_->CalcImpactProblemData(*context_, contacts, data);
  }

  // Computes rigid contact data.
  void CalcConstraintAccelProblemData(
      ConstraintAccelProblemData<double>* data) {
    // TODO(edrumwri): Compute the rigid contact data.
  }

  // Models an impact.
  void ModelImpact() {
    ConstraintVelProblemData<double> data(3 /* ngc */);
    CalcRigidImpactVelProblemData(&data);
    VectorX<double> cf;
    contact_solver_.SolveImpactProblem(data, &cf);

    // Get the update to the generalized velocity.
    VectorX<double> delta_v;
    contact_solver_.ComputeGeneralizedVelocityChange(data, cf, &delta_v);

    // Update the velocity part of the state.
    context_->get_mutable_continuous_state().get_mutable_generalized_velocity()
        .SetFromVector(data.solve_inertia(data.Mv) + delta_v);
  }

  // Gets the number of generalized coordinates for the rod.
  int get_rod_num_coordinates() { return 3; }

  // Gets the output dimension of a Jacobian multiplication operator.
  int GetOperatorDim(std::function<VectorX<double>(const VectorX<double>&)> J) {
    return J(VectorX<double>(get_rod_num_coordinates())).size();
  }

  // Checks the consistency of a transpose operator.
  void CheckTransOperatorDim(
      std::function<VectorX<double>(const VectorX<double>&)> JT,
  int num_constraints) {
    EXPECT_EQ(JT(VectorX<double>(num_constraints)).size(),
              get_rod_num_coordinates());
  }

  // Checks consistency of rigid contact problem data.
  void CheckProblemConsistency(
      const ConstraintAccelProblemData<double>& data,
      int num_contacts) {
    // TODO(edrumwri): Stop short-circuiting this after piecewise DAE
    // implementation repaired.
    return;

    EXPECT_EQ(num_contacts, data.sliding_contacts.size() +
        data.non_sliding_contacts.size());
    EXPECT_EQ(GetOperatorDim(data.N_mult), num_contacts);
    CheckTransOperatorDim(data.N_minus_muQ_transpose_mult, num_contacts);
    EXPECT_EQ(GetOperatorDim(data.L_mult), data.kL.size());
    CheckTransOperatorDim(data.L_transpose_mult, data.kL.size());
    EXPECT_EQ(data.tau.size(), get_rod_num_coordinates());
    EXPECT_EQ(data.kN.size(), num_contacts);
    EXPECT_EQ(data.kF.size(), data.non_sliding_contacts.size());
    EXPECT_EQ(data.mu_non_sliding.size(), data.non_sliding_contacts.size());
    EXPECT_EQ(data.mu_sliding.size(), data.sliding_contacts.size());
    EXPECT_EQ(data.r.size(), data.non_sliding_contacts.size());
    EXPECT_TRUE(data.solve_inertia);
    EXPECT_TRUE(std::is_sorted(data.sliding_contacts.begin(),
                               data.sliding_contacts.end()));
    EXPECT_TRUE(std::is_sorted(data.non_sliding_contacts.begin(),
                               data.non_sliding_contacts.end()));

    // Only true because this problem is 2D.
    EXPECT_EQ(GetOperatorDim(data.F_mult), data.non_sliding_contacts.size());
    CheckTransOperatorDim(data.F_transpose_mult,
                          data.non_sliding_contacts.size());
  }

  // Checks consistency of rigid impact problem data.
  void CheckProblemConsistency(
      const ConstraintVelProblemData<double>& data,
      int num_contacts) {
    EXPECT_EQ(num_contacts, data.mu.size());
    EXPECT_EQ(num_contacts, data.r.size());
    EXPECT_EQ(GetOperatorDim(data.N_mult), num_contacts);
    CheckTransOperatorDim(data.N_transpose_mult, num_contacts);
    EXPECT_EQ(data.kN.size(), num_contacts);
    EXPECT_EQ(data.Mv.size(), get_rod_num_coordinates());
    EXPECT_TRUE(data.solve_inertia);
    EXPECT_EQ(GetOperatorDim(data.F_mult), num_contacts);
    CheckTransOperatorDim(data.F_transpose_mult, num_contacts);
    EXPECT_EQ(data.kF.size(), num_contacts);
    EXPECT_EQ(GetOperatorDim(data.L_mult), data.kL.size());
    CheckTransOperatorDim(data.L_transpose_mult, data.kL.size());
  }

  std::unique_ptr<Rod2D<double>> dut_;  //< The device under test.
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  drake::multibody::constraint::ConstraintSolver<double>
      contact_solver_;
};

// Verifies that the state vector functions throw no exceptions.
TEST_F(Rod2DDAETest, NamedStateVectorsNoThrow) {
  DRAKE_EXPECT_NO_THROW(Rod2D<double>::get_mutable_state(context_.get()));
  DRAKE_EXPECT_NO_THROW(Rod2D<double>::get_state(*context_));
  DRAKE_EXPECT_NO_THROW(
      Rod2D<double>::get_state(context_->get_continuous_state()));
  DRAKE_EXPECT_NO_THROW(Rod2D<double>::get_mutable_state(
      &context_->get_mutable_continuous_state()));
}

// Tests that named state vector components are at expected indices.
TEST_F(Rod2DDAETest, ExpectedIndices) {
  // Set the state.
  Rod2dStateVector<double>& state = Rod2D<double>::get_mutable_state(
      context_.get());
  state.set_x(1.0);
  state.set_y(2.0);
  state.set_theta(3.0);
  state.set_xdot(5.0);
  state.set_ydot(7.0);
  state.set_thetadot(11.0);

  // Check the indices.
  const VectorBase<double>& x = context_->get_continuous_state_vector();
  EXPECT_EQ(state.x(), x[0]);
  EXPECT_EQ(state.y(), x[1]);
  EXPECT_EQ(state.theta(), x[2]);
  EXPECT_EQ(state.xdot(), x[3]);
  EXPECT_EQ(state.ydot(), x[4]);
  EXPECT_EQ(state.thetadot(), x[5]);
}

// Checks that the output port represents the state.
TEST_F(Rod2DDAETest, Output) {
  const ContinuousState<double>& xc = context_->get_continuous_state();
  std::unique_ptr<SystemOutput<double>> output =
      dut_->AllocateOutput();
  dut_->CalcOutput(*context_, output.get());
  for (int i = 0; i < xc.size(); ++i)
    EXPECT_EQ(xc[i], output->get_vector_data(0)->value()(i));
}

// Verifies that setting dut to an impacting state actually results in an
// impacting state.
TEST_F(Rod2DDAETest, ImpactingState) {
  // TODO(edrumwri): Set this state to impacting.

  // TODO(edrumwri): Verify that the rod system thinks that it is impacting.
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
  // Cause the initial state to be impacting, with center of mass directly
  // over the point of contact.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = -1.0;
  xc[5] = 0.0;

  // TODO(edrumwri): Verify that the rod is in an impacting state.

  // TODO(edrumwri): Model the impact (as fully inelastic), updating the state.

  // TODO(edrumwri): Verify that the state has been modified such that the body
  // is no longer in an impacting state and the configuration has not been
  // modified.
}

// Verify that derivatives match what we expect from a non-inconsistent,
// ballistic configuration.
TEST_F(Rod2DDAETest, ConsistentDerivativesBallistic) {
  // Set the initial state to ballistic motion.
  SetBallisticState();

  // TODO(edrumwri): Calculate the derivatives.

  // TODO(edrumwri): Verify that the derivatives match what we expect for this
  // non-inconsistent ballistic system.
}

// Verify that derivatives match what we expect from a non-inconsistent
// contacting configuration.
TEST_F(Rod2DDAETest, ConsistentDerivativesContacting) {
  // Set the initial state to sustained contact with zero tangential velocity
  // at the point of contact.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  SetLeftEndpointContacting(&context_->get_mutable_state(), false);

  // TODO(edrumwri): Calculate the derivatives.

  // TODO(edrumwri): Verify that derivatives match what we expect from a
  // non-inconsistent contacting configuration. In this case, there is no
  // initial sliding, velocity and the rod is oriented vertically, so we expect
  // no sliding to begin to occur.

  // TODO(edrumwri): Set the coefficient of friction to zero, update the
  // sliding velocity, and try again. Derivatives should be exactly the same
  // because no frictional force can be applied.
  xc[3] = -1.0;
  SetLeftEndpointContacting(&context_->get_mutable_state(), true);
  dut_->set_mu_coulomb(0.0);
  // TODO(edrumwri): Calculate and check derivatives here.

  // TODO(edrumwri): Add a large upward force and ensure that the rod
  // accelerates upward.
  const double fup = 100.0;
  const Vector3<double> ext_input(0, fup, 0);
  dut_->get_input_port(0).FixValue(context_.get(), ext_input);
  // TODO(edrumwri): Calculate and check derivatives here.
}

// Verify that derivatives match what we expect from a sticking contact
// configuration.
TEST_F(Rod2DDAETest, DerivativesContactingAndSticking) {
  // Set the initial state to sustained contact with zero tangential velocity
  // at the point of contact and the rod being straight up.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  // TODO(edrumwri): Set the mode variable appropriately.

  // Set a constant horizontal input force, as if applied at the bottom of
  // the rod.
  const double f_x = 1.0;
  const double f_y = -1.0;
  const Vector3<double> ext_input(f_x, f_y, f_x * dut_->get_rod_half_length());
  dut_->get_input_port(0).FixValue(context_.get(), ext_input);

  // Set the coefficient of friction such that the contact forces are right
  // on the edge of the friction cone. Determine the predicted normal force
  // (this simple formula is dependent upon the upright rod configuration).
  const double mu_stick = f_x / (dut_->get_rod_mass() *
                                 -dut_->get_gravitational_acceleration() -
                                 f_y);
  dut_->set_mu_coulomb(mu_stick);

  // TODO(edrumwri): Calculate the derivatives.

  // TODO(edrumwri): Verify that derivatives match what we expect: sticking
  // should continue.

  // Set the coefficient of friction to 99.9% of the sticking value and then
  // verify that the contact state transitions from a sticking one to a
  // non-sticking one.
  const double mu_slide = 0.999 * mu_stick;
  dut_->set_mu_coulomb(mu_slide);
  // TODO(edrumwri): Calculate and check derivatives here.

  // Set the coefficient of friction to zero and try again.
  dut_->set_mu_coulomb(0.0);
  // TODO(edrumwri): Calculate and check derivatives here.
}

// Verify that the (non-impacting) Painlevé configuration does not result in a
// state change.
TEST_F(Rod2DDAETest, ImpactNoChange) {
  // TODO(edrumwri): Verify not impacting at initial state.

  // Get the continuous state.
  const VectorX<double> xc_old = context_->get_continuous_state().
      get_vector().CopyToVector();

  // Model the impact and get the continuous state out.
  // TODO(edrumwri): Model the impact here.
  const VectorX<double> xc = context_->get_continuous_state().
      get_vector().CopyToVector();

  // TODO(edrumwri): Verify the continuous state did not change.
}

// Verify that applying the impact model to an impacting configuration results
// in a non-impacting configuration. This test exercises the model for the case
// where impulses that yield tangential sticking lie within the friction cone.
TEST_F(Rod2DDAETest, InfFrictionImpactThenNoImpact) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to infinite.
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  std::unique_ptr<State<double>> new_state = CloneState();
  // TODO(edrumwri): Model the impact.
  context_->get_mutable_state().SetFrom(*new_state);
  // TODO(edrumwri): Verify the state is no longer an impacting one.

  // TODO(edrumwri): Attempt to model one more impact- and verify that there is
  // no change in the continuous state.
}

// Verify that applying an impact model to an impacting state results in a
// non-impacting state. This test exercises the model for the case
// where impulses that yield tangential sticking lie outside the friction cone.
TEST_F(Rod2DDAETest, NoFrictionImpactThenNoImpact) {
  // Set the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to zero.
  dut_->set_mu_coulomb(0.0);

  //  Model the impact and copy the result to the context.
  std::unique_ptr<State<double>> new_state = CloneState();
  // TODO(edrumwri): Model the impact here.
  context_->get_mutable_state().SetFrom(*new_state);
  // TODO(edrumwri): Verify that the state does not denote impact.

  // Do one more impact- there should now be no change.
  // TODO(edrumwri): Model the impact here.
  // TODO(edrumwri): Verify that there was no further change from the second
  // impact.
}

// Verify that no exceptions thrown for a non-sliding configuration.
TEST_F(Rod2DDAETest, NoSliding) {
  const double half_len = dut_->get_rod_half_length();
  const double r22 = std::sqrt(2) / 2;
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();

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
  SetLeftEndpointContacting(&context_->get_mutable_state(), false);

  // TODO(edrumwri): Verify no impact.

  // TODO(edrumwri): Verify no exceptions thrown when calculating derivatives.

  // Set the coefficient of friction to infinite.
  dut_->set_mu_static(std::numeric_limits<double>::infinity());
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // No exceptions should be thrown.
  // TODO(edrumwri): Verify no exceptions thrown when calculating derivatives.
}

// Test multiple (two-point) contact configurations.
TEST_F(Rod2DDAETest, MultiPoint) {
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();

  // Set the rod to a horizontal, two-contact configuration.
  xc[0] = 0;
  xc[1] = 0;
  xc[2] = 0;

  // Set the velocity on the rod such that it is moving horizontally.
  xc[3] = 1.0;
  // TODO(edrumwri): Verify no impact.

  // Set the coefficient of friction to zero.
  dut_->set_mu_coulomb(0.0);

  // Compute the derivatives and verify that the linear and angular acceleration
  // are approximately zero.
  // TODO(edrumwri): Calculate the derivatives.
  // TODO(edrumwri): Verify derivatives(3:5) are all approximately zero.

  // Set the coefficient of friction to "very large".
  const double large = 100.0;
  dut_->set_mu_coulomb(large);

  // TODO(edrumwri): Calculate the derivatives.
  // TODO(edrumwri): Verify derivatives(3) is approximately (scaled by the
  // magnitude of large) equal to the graviational acceleration.
  // TODO(edrumwri): Verify derivatives(4:5) are all approximately zero.

  // Set the rod velocity to zero.
  xc[3] = 0.0;
  // TODO(edrumwri): Verify no impact.

  // Set a constant force pushing the rod.
  const double fX = 1.0;
  const Vector3<double> ext_input(fX, 0, 0);
  dut_->get_input_port(0).FixValue(context_.get(), ext_input);

  // Verify that the linear and angular acceleration are still zero.
  // TODO(edrumwri): Calculate the derivatives and verify derivatives(3:5) are
  // all approximately zero.

  // Set the coefficient of friction to zero. Now the force should result
  // in the rod being pushed to the right.
  dut_->set_mu_coulomb(0.0);
  // TODO(edrumwri): Calculate the derivatives.
  // TODO(edrumwri): Verify derivatives(3) is approximately equal to the
  // fX divided by the rod mass.
  // TODO(edrumwri): Verify derivatives(4:5) are all approximately zero.
}

// Verify that the Painlevé configuration does not correspond to an impacting
// state.
TEST_F(Rod2DDAETest, ImpactNoChange2) {
  SetSecondInitialConfig();

  // TODO(edrumwri): Model the impact.

  // TODO(edrumwri): Verify the state has not changed.
}

// Verify that applying the impact model to an impacting state results
// in a non-impacting state.
TEST_F(Rod2DDAETest, InfFrictionImpactThenNoImpact2) {
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to infinite.
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Model the impact and copy the result to the context.
  // TODO(edrumwri): Model the impact.
  context_->get_mutable_state().SetFrom(*new_state);

  // TODO(edrumwri): Verify the state no longer corresponds to an impact.

  // Model one more impact- there should now be no change.
  // TODO(edrumwri): Model the impact.
  // TODO(edrumwri): Verify the state has not changed.
}

// Verify that applying the impact model to an impacting state results in a
// non-impacting state.
TEST_F(Rod2DDAETest, NoFrictionImpactThenNoImpact2) {
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // TODO(edrumwri):

  // Set the coefficient of friction to zero.
  dut_->set_mu_coulomb(0.0);

  // Model the impact and copy the result to the context.
  // TODO(edrumwri): Model the impact.
  context_->get_mutable_state().SetFrom(*new_state);
  // TODO(edrumwri): Verify that the state does not correspond to an impact.

  // Do one more impact- there should now be no change.
  // TODO(edrumwri): Model the impact.
  // TODO(edrumwri): Verify the state has not changed.
}

// Verifies that rod in a ballistic state does not correspond to an impact.
TEST_F(Rod2DDAETest, BallisticNoImpact) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Move the rod upward vertically so that it is no longer impacting.
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] += 10.0;

  // TODO(edrumwri): Verify that no impact occurs.
  EXPECT_FALSE(dut_->IsImpacting(*context_));
}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a ballistic state.
TEST_F(Rod2DDAETest, RigidContactProblemDataBallistic) {
  SetBallisticState();

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(get_rod_num_coordinates());
  CalcConstraintAccelProblemData(&data);

  // Verify that the data has reasonable values.
  const int num_contacts = 0;
  CheckProblemConsistency(data, num_contacts);
}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a two-contact, at-rest configuration.
TEST_F(Rod2DDAETest, RigidContactProblemDataHorizontalResting) {
  // Set the rod to a resting horizontal configuration.
  SetRestingHorizontalConfig();

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(3 /* gen. vel. dim */);
  CalcConstraintAccelProblemData(&data);
  const int num_contacts = 2;
  CheckProblemConsistency(data, num_contacts);

  // TODO(edrumwri): Verify that both contacts are not sliding.
}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a two-contact, sliding configuration.
TEST_F(Rod2DDAETest, RigidContactProblemDataHorizontalSliding) {
  // Set the rod to a sliding horizontal configuration.
  SetRestingHorizontalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[3] = 1.0;  // horizontal velocity of the rod center-of-mass.

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(3 /* gen. vel. dim */);
  CalcConstraintAccelProblemData(&data);
  const int num_contacts = 2;
  CheckProblemConsistency(data, num_contacts);

  // TODO(edrumwri): Verify that both contacts are sliding.
}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a single-contact, at-rest configuration.
TEST_F(Rod2DDAETest, RigidContactProblemDataVerticalResting) {
  // Set the rod to a resting vertical configuration.
  SetRestingVerticalConfig();

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(3 /* gen. vel. dim */);
  CalcConstraintAccelProblemData(&data);
  const int num_contacts = 1;
  CheckProblemConsistency(data, num_contacts);

  // TODO(edrumwri): Verify that contact is not sliding.

  // TODO(edrumwri): Most tests are necessary to better stress the validity of
  // N, N - μQ, F, dN/dt⋅v, and dF/dt⋅v.
  // TODO(edrumwri): Verify that N has no angular component.
}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a two-contact, sliding configuration.
TEST_F(Rod2DDAETest, RigidContactProblemDataVerticalSliding) {
  // Set the rod to a sliding vertical configuration.
  SetRestingVerticalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[3] = 1.0;

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(3 /* gen. vel. dim */);
  CalcConstraintAccelProblemData(&data);
  const int num_contacts = 1;
  CheckProblemConsistency(data, num_contacts);

  // TODO(edrumwri): Verify that contact is sliding.
}

/// Class for testing the Rod 2D example using a first order time
/// stepping approach.
class Rod2DDiscretizedTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double dt = 1e-2;
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SystemType::kDiscretized, dt);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Set a zero input force (this is the default).
    const Vector3<double> ext_input(0, 0, 0);
    dut_->get_input_port(0).FixValue(context_.get(), ext_input);
  }

  BasicVector<double>& mutable_discrete_state() {
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
    auto xd = mutable_discrete_state().get_mutable_value();

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
/// first-order discretization approach (this tests expected meta behavior).
TEST_F(Rod2DDiscretizedTest, RodGoesToRest) {
  // Set the initial state to an inconsistent configuration.
  SetSecondInitialConfig();

  // Init the simulator.
  Simulator<double> simulator(*dut_, std::move(context_));

  // Integrate forward to a point where the rod should be at rest.
  const double t_final = 10;
  simulator.AdvanceTo(t_final);

  // Get angular orientation and velocity.
  const VectorXd& xd = simulator.get_context().get_discrete_state(0).value();
  const double theta = xd(2);
  const double theta_dot = xd(5);

  // After sufficiently long, theta should be 0 or M_PI and the velocity
  // should be nearly zero.
  EXPECT_TRUE(std::fabs(theta) < 1e-6 || std::fabs(theta - M_PI) < 1e-6);
  EXPECT_NEAR(theta_dot, 0.0, 1e-6);
}

// Validates the number of witness functions is determined correctly.
TEST_F(Rod2DDiscretizedTest, NumWitnessFunctions) {
  EXPECT_EQ(dut_->DetermineNumWitnessFunctions(*context_), 0);
}

// This test checks to see whether a single semi-explicit step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system.
GTEST_TEST(Rod2DCrossValidationTest, OneStepSolutionSliding) {
  // Create two Rod2D systems.
  const double dt = 1e-1;
  Rod2D<double> ts(Rod2D<double>::SystemType::kDiscretized, dt);
  Rod2D<double> pdae(Rod2D<double>::SystemType::kPiecewiseDAE, 0.0);

  // Set the coefficient of friction to a small value for both.
  const double mu = 0.01;
  ts.set_mu_coulomb(mu);
  pdae.set_mu_coulomb(mu);

  // Set "one step" constraint stabilization (not generally recommended, but
  // works for a single step) and small regularization.
  const double cfm = std::numeric_limits<double>::epsilon();
  const double erp = 1.0;
  ts.SetStiffnessAndDissipation(cfm, erp);

  // Create contexts for both.
  std::unique_ptr<Context<double>> context_ts = ts.CreateDefaultContext();
  std::unique_ptr<Context<double>> context_pdae = pdae.CreateDefaultContext();

  // Set zero input forces for both.
  Vector3<double> fext(0, 0, 0);
  ts.get_input_port(0).FixValue(context_ts.get(), fext);
  pdae.get_input_port(0).FixValue(context_pdae.get(), fext);

  // Init the simulator for the discretized (time stepping) system.
  Simulator<double> simulator_ts(ts, std::move(context_ts));

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_ts.AdvanceTo(dt);
  EXPECT_EQ(simulator_ts.get_num_discrete_updates(), 1);

  // Manually integrate the continuous state forward for the piecewise DAE
  // based approach.
  std::unique_ptr<ContinuousState<double>> f = pdae.AllocateTimeDerivatives();
  // TODO(edrumwri): Calculate the derivatives here.
  auto& xc = context_pdae->get_mutable_continuous_state_vector();
  xc.SetAtIndex(3, xc.GetAtIndex(3) + dt * ((*f)[3]));
  xc.SetAtIndex(4, xc.GetAtIndex(4) + dt * ((*f)[4]));
  xc.SetAtIndex(5, xc.GetAtIndex(5) + dt * ((*f)[5]));
  xc.SetAtIndex(0, xc.GetAtIndex(0) + dt * xc.GetAtIndex(3));
  xc.SetAtIndex(1, xc.GetAtIndex(1) + dt * xc.GetAtIndex(4));
  xc.SetAtIndex(2, xc.GetAtIndex(2) + dt * xc.GetAtIndex(5));

  // TODO(edrumwri): Check that the solution is nearly identical.

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
  Rod2D<double> ts(Rod2D<double>::SystemType::kDiscretized, dt);
  Rod2D<double> pdae(Rod2D<double>::SystemType::kPiecewiseDAE, 0.0);

  // Set the coefficient of friction to a large value for both.
  const double mu = 100.0;
  ts.set_mu_coulomb(mu);
  pdae.set_mu_coulomb(mu);

  // Set "one step" constraint stabilization (not generally recommended, but
  // works for a single step) and small regularization.
  const double cfm = std::numeric_limits<double>::epsilon();
  const double erp = 1.0;
  ts.SetStiffnessAndDissipation(cfm, erp);

  // Create contexts for both.
  std::unique_ptr<Context<double>> context_ts = ts.CreateDefaultContext();
  std::unique_ptr<Context<double>> context_pdae = pdae.CreateDefaultContext();

  // This configuration has no sliding velocity.
  const double half_len = pdae.get_rod_half_length();
  ContinuousState<double>& xc = context_pdae->get_mutable_continuous_state();
  auto xd = context_ts->get_mutable_discrete_state(0).get_mutable_value();
  xc[0] = xd[0] = 0.0;
  xc[1] = xd[1] = half_len;
  xc[2] = xd[2] = M_PI_2;
  xc[3] = xd[3] = 0.0;
  xc[4] = xd[4] = 0.0;
  xc[5] = xd[5] = 0.0;

  // Set constant input forces for both.
  const double x = 1.0;
  Vector3<double> fext(x, 0, x * ts.get_rod_half_length());
  ts.get_input_port(0).FixValue(context_ts.get(), fext);
  pdae.get_input_port(0).FixValue(context_pdae.get(), fext);

  // Init the simulator for the time stepping system.
  Simulator<double> simulator_ts(ts, std::move(context_ts));

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the discretized system, so stepping to dt should yield
  // exactly one step.
  simulator_ts.AdvanceTo(dt);
  EXPECT_EQ(simulator_ts.get_num_discrete_updates(), 1);

  // TODO(edrumwri): Manually integrate the continuous state forward for the
  // piecewise DAE based approach.
  std::unique_ptr<ContinuousState<double>> f = pdae.AllocateTimeDerivatives();
  // TODO(edrumwri): Compute derivatives here.
  xc[3] += + dt * ((*f)[3]);
  xc[4] += + dt * ((*f)[4]);
  xc[5] += + dt * ((*f)[5]);
  xc[0] += + dt * xc[3];
  xc[1] += + dt * xc[4];
  xc[2] += + dt * xc[5];

  // TODO(edrumwri): Check that the solution is nearly identical.
}

/// Class for testing the Rod 2D example using compliant contact
/// thus permitting integration as an ODE.
class Rod2DContinuousTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SystemType::kContinuous, 0.0);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();
    derivatives_ = dut_->AllocateTimeDerivatives();

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Using default compliant contact parameters.

    // Set a zero input force (this is the default).
    const Vector3<double> ext_input(0, 0, 0);
    dut_->get_input_port(0).FixValue(context_.get(), ext_input);
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
    const ContinuousState<double>& xc = context_->get_continuous_state();
    return Vector6d(xc.CopyToVector());
  }

  // Return d/dt state xdot,ydot,θdot,xddot,yddot,θddot as a Vector6.
  Vector6d get_state_dot() const {
    const ContinuousState<double>& xcd = *derivatives_;
    return Vector6d(xcd.CopyToVector());
  }

  // Sets the planar pose in the context.
  void set_pose(double x, double y, double theta) {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc[0] = x; xc[1] = y; xc[2] = theta;
  }

  // Sets the planar velocity in the context.
  void set_velocity(double xdot, double ydot, double thetadot) {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
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
TEST_F(Rod2DContinuousTest, ForcesHaveRightSign) {
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

// Validates the number of witness functions is determined correctly.
TEST_F(Rod2DContinuousTest, NumWitnessFunctions) {
  EXPECT_EQ(dut_->DetermineNumWitnessFunctions(*context_), 0);
}

// Verifies that output ports give expected values.
GTEST_TEST(Rod2DCrossValidationTest, Outputs) {
  // Create two Rod2D systems, one time stepping (i.e., discretized),
  // one with continuous state.
  const double dt = 1e-1;
  Rod2D<double> ts(Rod2D<double>::SystemType::kDiscretized, dt);
  Rod2D<double> pdae(Rod2D<double>::SystemType::kPiecewiseDAE, 0.0);

  // Create contexts for both.
  std::unique_ptr<Context<double>> context_ts = ts.CreateDefaultContext();
  std::unique_ptr<Context<double>> context_pdae = pdae.CreateDefaultContext();

  // Allocate outputs for both.
  auto output_ts = ts.AllocateOutput();
  auto output_pdae = pdae.AllocateOutput();

  // Compute outputs.
  ts.CalcOutput(*context_ts, output_ts.get());
  pdae.CalcOutput(*context_pdae, output_pdae.get());

  // Set port indices.
  const int state_port = 0;

  // Verify that state outputs are identical.
  const double eq_tol = 10 * std::numeric_limits<double>::epsilon();
  const VectorXd x_ts = output_ts->get_vector_data(state_port)->CopyToVector();
  VectorXd x_pdae = output_pdae->get_vector_data(state_port)->CopyToVector();
  EXPECT_LT((x_ts - x_pdae).lpNorm<Eigen::Infinity>(), eq_tol);

  // Transform the rod and verify that pose output is as expected.
  x_pdae[0] = 0;
  x_pdae[1] = pdae.get_rod_half_length();
  x_pdae[2] = M_PI_2;
  context_pdae->SetContinuousState(x_pdae);
  pdae.CalcOutput(*context_pdae, output_pdae.get());
}

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
