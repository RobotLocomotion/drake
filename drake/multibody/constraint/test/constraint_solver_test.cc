#include "drake/multibody/constraint/constraint_solver.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/rod2d/rod2d.h"

using drake::systems::ContinuousState;
using drake::systems::Context;
using drake::examples::rod2d::Rod2D;
using drake::systems::BasicVector;
using Vector2d = Eigen::Vector2d;

namespace drake {
namespace multibody {
namespace constraint {
namespace {

class Constraint2DSolverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rod_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kPiecewiseDAE, 0);
    context_ = rod_->CreateDefaultContext();

    // Use a non-unit mass.
    rod_->set_rod_mass(2.0);

    // Set a zero input force (this is the default).
    std::unique_ptr<BasicVector<double>> ext_input =
        std::make_unique<BasicVector<double>>(3);
    ext_input->SetAtIndex(0, 0.0);
    ext_input->SetAtIndex(1, 0.0);
    ext_input->SetAtIndex(2, 0.0);
    context_->FixInputPort(0, std::move(ext_input));

    // Construct the problem data for the 2D rod.
    const int gv_dim = 3;
    accel_data_ = std::make_unique<ConstraintAccelProblemData<double>>(gv_dim);
    vel_data_ = std::make_unique<ConstraintVelProblemData<double>>(gv_dim);

    // Set epsilon.
    eps_ = 200 * std::max(std::numeric_limits<double>::epsilon(), cfm_);
  }

  double cfm_{0};    // Regularization parameter.
  double eps_{-1};   // Zero tolerance (< 0 indicates not set).
  ConstraintSolver<double> solver_;
  std::unique_ptr<Rod2D<double>> rod_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<ConstraintAccelProblemData<double>> accel_data_;
  std::unique_ptr<ConstraintVelProblemData<double>> vel_data_;

  // Gets the frame for a sliding contact.
  Matrix2<double> GetSlidingContactFrameToWorldTransform(
      double xaxis_velocity) const {
    return rod_->GetSlidingContactFrameToWorldTransform(xaxis_velocity);
  }

  // Gets the frame for a non-sliding contact.
  Matrix2<double> GetNonSlidingContactFrameToWorldTransform() const {
    return rod_->GetNonSlidingContactFrameToWorldTransform();
  }

  // Sets the rod to a resting horizontal configuration without modifying the
  // rod's mode variables.
  void SetRodToRestingHorizontalConfig() {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;     // com horizontal position
    xc[1] = 0.0;     // com vertical position
    xc[2] = 0.0;     // rod rotation
    xc[3] = xc[4] = xc[5] = 0.0;   // velocity variables
  }

  // Sets the rod to an upward moving horizontal configuration without modifying
  // the rod's mode variables.
  void SetRodToUpwardMovingHorizontalConfig() {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;     // com horizontal position
    xc[1] = 0.0;     // com vertical position
    xc[2] = 0.0;     // rod rotation
    xc[3] = 0.0;     // no horizontal velocity.
    xc[4] = 1.0;     // upward velocity.
    xc[5] = 0.0;     // no angular velocity.
  }

  // Sets the rod to an impacting, sliding velocity with the rod configured to
  // lie upon its side and without modifying the rod's mode variables.
  void SetRodToSlidingImpactingHorizontalConfig() {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;     // com horizontal position
    xc[1] = 0.0;     // com vertical position
    xc[2] = 0.0;     // rod rotation
    xc[3] = 1.0;     // sliding horizontal velocity.
    xc[4] = -1.0;    // impacting velocity.
    xc[5] = 0.0;     // no angular velocity.
  }

  // Sets the rod to a resting vertical configuration without modifying the
  // mode variables.
  void SetRodToRestingVerticalConfig() {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[0] = 0.0;                             // com horizontal position
    xc[1] = rod_->get_rod_half_length();     // com vertical position
    xc[2] = M_PI_2;                          // rod rotation
    xc[3] = xc[4] = xc[5] = 0.0;             // velocity variables
  }

  // Computes rigid contact data.
  void CalcConstraintAccelProblemData(
      ConstraintAccelProblemData<double>* data) {
    // Get the points of contact and contact tangent velocities.
    std::vector<Vector2d> contacts;
    std::vector<double> tangent_vels;
    rod_->GetContactPoints(*context_, &contacts);
    rod_->GetContactPointsTangentVelocities(*context_, contacts, &tangent_vels);

    // Compute the problem data.
    rod_->CalcConstraintProblemData(*context_, contacts, tangent_vels,
                                         data);

    // Check the consistency of the data.
    CheckProblemConsistency(*data, contacts.size());
  }

  // Computes rigid impacting contact data.
  void CalcConstraintVelProblemData(
      ConstraintVelProblemData<double>* data) {
    // Get the points of contact.
    std::vector<Vector2d> contacts;
    rod_->GetContactPoints(*context_, &contacts);

    // Compute the problem data.
    rod_->CalcImpactProblemData(*context_, contacts, data);

    // Check the consistency of the data.
    CheckProblemConsistency(*data, contacts.size());
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
    const int ngc = get_rod_num_coordinates();
    EXPECT_EQ(num_contacts, data.sliding_contacts.size() +
        data.non_sliding_contacts.size());
    EXPECT_EQ(GetOperatorDim(data.N_mult), num_contacts);
    CheckTransOperatorDim(data.N_minus_muQ_transpose_mult, num_contacts);
    EXPECT_EQ(GetOperatorDim(data.F_mult), data.non_sliding_contacts.size());
    EXPECT_EQ(GetOperatorDim(data.L_mult), data.num_limit_constraints);
    CheckTransOperatorDim(data.L_transpose_mult, data.num_limit_constraints);
    EXPECT_EQ(data.f.size(), ngc);
    EXPECT_EQ(data.Ndot_x_v.size(), num_contacts);
    EXPECT_EQ(data.Fdot_x_v.size(), data.non_sliding_contacts.size());
    EXPECT_EQ(data.Ldot_x_v.size(), data.num_limit_constraints);
    EXPECT_EQ(data.mu_non_sliding.size(), data.non_sliding_contacts.size());
    EXPECT_EQ(data.mu_sliding.size(), data.sliding_contacts.size());
    EXPECT_EQ(data.r.size(), data.non_sliding_contacts.size());
    EXPECT_TRUE(data.solve_inertia);
    EXPECT_TRUE(std::is_sorted(data.sliding_contacts.begin(),
                               data.sliding_contacts.end()));
    EXPECT_TRUE(std::is_sorted(data.non_sliding_contacts.begin(),
                               data.non_sliding_contacts.end()));
  }

  // Checks consistency of rigid impact problem data.
  void CheckProblemConsistency(
      const ConstraintVelProblemData<double>& data,
      int num_contacts) {
    const int ngc = get_rod_num_coordinates();
    const int num_spanning_directions = std::accumulate(
        data.r.begin(), data.r.end(), 0);
    EXPECT_EQ(GetOperatorDim(data.N_mult), num_contacts);
    CheckTransOperatorDim(data.N_transpose_mult, num_contacts);
    EXPECT_EQ(GetOperatorDim(data.F_mult), num_spanning_directions);
    CheckTransOperatorDim(data.F_transpose_mult, num_spanning_directions);
    EXPECT_EQ(GetOperatorDim(data.L_mult), data.num_limit_constraints);
    CheckTransOperatorDim(data.L_transpose_mult, data.num_limit_constraints);
    EXPECT_EQ(data.v.size(), ngc);
    EXPECT_EQ(data.mu.size(), num_contacts);
    EXPECT_EQ(data.r.size(), num_contacts);
    EXPECT_TRUE(data.solve_inertia);
  }
};

// Tests the rod in a two-point configuration, in a situation where a force
// pulls the rod upward (and no contact forces should be applied).
TEST_F(Constraint2DSolverTest, TwoPointPulledUpward) {
  // Set the state of the rod to resting on its side with no velocity.
  SetRodToRestingHorizontalConfig();

  // Compute the problem data.
  CalcConstraintAccelProblemData(accel_data_.get());

  // Add a force pulling the rod upward.
  accel_data_->f[1] += 100.0;

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveConstraintProblem(cfm_, *accel_data_, &cf);

  // Verify that the contact forces are zero.
  EXPECT_LT(cf.norm(), eps_);
}

// Tests the rod in a two-point configuration, in a situation where the rod
// is moving upward, so no impulsive forces should be applied.
TEST_F(Constraint2DSolverTest, NoImpactImpliesNoImpulses) {
  // Set the state of the rod to resting on its side with upward velocity.
  SetRodToUpwardMovingHorizontalConfig();

  // Compute the problem data.
  CalcConstraintVelProblemData(vel_data_.get());

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveImpactProblem(cfm_, *vel_data_, &cf);

  // Verify that the impact forces are zero.
  EXPECT_LT(cf.norm(), eps_);
}

// Tests the rod in a two-point sticking configuration.
TEST_F(Constraint2DSolverTest, TwoPointSticking) {
  // Set the state of the rod to resting on its side with no velocity.
  SetRodToRestingHorizontalConfig();

  // Set the rod to large friction.
  rod_->set_mu_coulomb(15.0);
  rod_->set_mu_static(15.0);

  // Compute the problem data.
  CalcConstraintAccelProblemData(accel_data_.get());
  EXPECT_TRUE(accel_data_->sliding_contacts.empty());

  // Add a force pulling the rod horizontally.
  const double horz_f = 100.0;
  accel_data_->f[0] += horz_f;

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveConstraintProblem(cfm_, *accel_data_, &cf);

  // Construct the contact frames.
  std::vector<Matrix2<double>> frames;
  frames.push_back(GetNonSlidingContactFrameToWorldTransform());
  frames.push_back(GetNonSlidingContactFrameToWorldTransform());

  // Verify that the x-axis of the contact frame, which corresponds to the
  // contact normal, points along the world y-axis, and the y-axis of the
  // contact frame, which corresponds to a contact tangent vector, points
  // along the world x-axis.
  for (size_t i = 0; i < frames.size(); ++i) {
    EXPECT_LT(std::fabs(frames[i].col(0).dot(Vector2<double>::UnitY()) - 1.0),
      std::numeric_limits<double>::epsilon());
    EXPECT_LT(std::fabs(frames[i].col(1).dot(Vector2<double>::UnitX()) - 1.0),
      std::numeric_limits<double>::epsilon());
  }

  // Get the contact forces expressed in the contact frames.
  std::vector<Vector2<double>> contact_forces;
  ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
      *accel_data_, frames, &contact_forces);

  // Verify that the number of contact force vectors is correct.
  ASSERT_EQ(contact_forces.size(), 2);

  // Verify that the frictional forces equal the horizontal forces. Frictional
  // forces are in the second component of each vector.
  EXPECT_NEAR(std::fabs(contact_forces.front()[1]) +
              std::fabs(contact_forces.back()[1]), horz_f, eps_);

  // Verify that the generalized acceleration of the rod is equal to zero.
  VectorX<double> ga;
  solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
  EXPECT_LT(ga.norm(), eps_);
}

// Tests the rod in a two-point impacting and sticking configuration.
TEST_F(Constraint2DSolverTest, TwoPointImpactingAndSticking) {
  // Set the state of the rod to lying on its side with both impacting velocity
  // and horizontally moving velocity.
  SetRodToSlidingImpactingHorizontalConfig();

  // Set the rod to large friction.
  rod_->set_mu_coulomb(15.0);

  // Compute the impact problem data.
  CalcConstraintVelProblemData(vel_data_.get());

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveImpactProblem(cfm_, *vel_data_, &cf);

  // Construct the contact frames.
  std::vector<Matrix2<double>> frames;
  frames.push_back(GetNonSlidingContactFrameToWorldTransform());
  frames.push_back(GetNonSlidingContactFrameToWorldTransform());

  // Verify that the x-axis of the contact frame, which corresponds to the
  // contact normal, points along the world y-axis, and the y-axis of the
  // contact frame, which corresponds to a contact tangent vector, points
  // along the world x-axis.
  for (size_t i = 0; i < frames.size(); ++i) {
    EXPECT_LT(std::fabs(frames[i].col(0).dot(Vector2<double>::UnitY()) - 1.0),
              std::numeric_limits<double>::epsilon());
    EXPECT_LT(std::fabs(frames[i].col(1).dot(Vector2<double>::UnitX()) - 1.0),
              std::numeric_limits<double>::epsilon());
  }

  // Get the impulsive contact forces expressed in the contact frames.
  std::vector<Vector2<double>> contact_forces;
  ConstraintSolver<double>::CalcImpactForcesInContactFrames(cf, *vel_data_,
    frames, &contact_forces);

  // Verify that the number of contact force vectors is correct.
  ASSERT_EQ(contact_forces.size(), 2);

  // Verify that the generalized velocity of the rod is equal to zero.
  VectorX<double> dgv;
  solver_.ComputeGeneralizedVelocityChange(*vel_data_, cf, &dgv);
  EXPECT_LT((vel_data_->v + dgv).norm(), eps_);
}

// Tests the rod in a single-point sticking configuration.
TEST_F(Constraint2DSolverTest, SinglePointSticking) {
  // Set the state of the rod to resting on its side with no velocity.
  SetRodToRestingVerticalConfig();

  // Set the rod to large friction.
  rod_->set_mu_coulomb(15.0);
  rod_->set_mu_static(15.0);

  // Compute the problem data.
  CalcConstraintAccelProblemData(accel_data_.get());
  EXPECT_TRUE(accel_data_->sliding_contacts.empty());

  // Add a force, acting at the point of contact, that pulls the rod
  // horizontally.
  const double horz_f = 100.0;
  accel_data_->f[0] += horz_f;
  accel_data_->f[2] += horz_f * rod_->get_rod_half_length();

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveConstraintProblem(cfm_, *accel_data_, &cf);

  // Construct the contact frame.
  std::vector<Matrix2<double>> frames;
  frames.push_back(GetNonSlidingContactFrameToWorldTransform());

  // Verify that the x-axis of the contact frame, which corresponds to the
  // contact normal, points along the world y-axis, and the y-axis of the
  // contact frame, which corresponds to a contact tangent vector, points
  // along the world x-axis.
  for (size_t i = 0; i < frames.size(); ++i) {
    EXPECT_LT(std::fabs(frames[i].col(0).dot(Vector2<double>::UnitY()) - 1.0),
      std::numeric_limits<double>::epsilon());
    EXPECT_LT(std::fabs(frames[i].col(1).dot(Vector2<double>::UnitX()) - 1.0),
      std::numeric_limits<double>::epsilon());
  }

  // Get the contact forces expressed in the contact frame.
  std::vector<Vector2<double>> contact_forces;
  ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
      *accel_data_, frames, &contact_forces);

  // Verify that the number of contact force vectors is correct.
  ASSERT_EQ(contact_forces.size(), 1);

  // Verify that the ℓ₁ norm of frictional forces equals the horizontal force
  // magnitude.
  const int nc = accel_data_->non_sliding_contacts.size();
  EXPECT_NEAR(cf.segment(nc, cf.size() - nc).lpNorm<1>(), horz_f, eps_);

  // Verify that the generalized acceleration of the rod is equal to zero.
  VectorX<double> ga;
  solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
  EXPECT_LT(ga.norm(), eps_);
}

// Tests the rod in a two-point non-sticking configuration that will transition
// to sliding.
TEST_F(Constraint2DSolverTest, TwoPointNonSlidingToSliding) {
  // Set the state of the rod to resting on its side with no velocity.
  SetRodToRestingHorizontalConfig();

  // Set the coefficient of friction.
  rod_->set_mu_coulomb(0.1);
  rod_->set_mu_static(0.1);

  // Compute the problem data.
  CalcConstraintAccelProblemData(accel_data_.get());

  // Add a force pulling the rod horizontally.
  const double horz_f = 100.0;
  accel_data_->f[0] += horz_f;

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveConstraintProblem(cfm_, *accel_data_, &cf);

  EXPECT_TRUE(accel_data_->sliding_contacts.empty());

  // Construct the contact frames.
  std::vector<Matrix2<double>> frames;
  frames.push_back(GetNonSlidingContactFrameToWorldTransform());
  frames.push_back(GetNonSlidingContactFrameToWorldTransform());

  // Verify that the x-axis of the contact frame, which corresponds to the
  // contact normal, points along the world y-axis, and the y-axis of the
  // contact frame, which corresponds to a contact tangent vector, points
  // along the world x-axis.
  for (size_t i = 0; i < frames.size(); ++i) {
    EXPECT_LT(std::fabs(frames[i].col(0).dot(Vector2<double>::UnitY()) - 1.0),
      std::numeric_limits<double>::epsilon());
    EXPECT_LT(std::fabs(frames[i].col(1).dot(Vector2<double>::UnitX()) - 1.0),
      std::numeric_limits<double>::epsilon());
  }

  // Get the contact forces expressed in the contact frames.
  std::vector<Vector2<double>> contact_forces;
  ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
      *accel_data_, frames, &contact_forces);

  // Verify that the number of contact force vectors is correct.
  ASSERT_EQ(contact_forces.size(), 2);

  // Verify that the frictional forces are not zero and are less than the
  // horizontal forces. Frictional forces are in the second component of each
  // vector.
  EXPECT_GT(std::fabs(contact_forces.front()[1]) +
              std::fabs(contact_forces.back()[1]), eps_);
  EXPECT_LT(std::fabs(contact_forces.front()[1]) +
            std::fabs(contact_forces.back()[1]), horz_f);

  // Verify that the horizontal acceleration is to the right.
  VectorX<double> ga;
  solver_.ComputeGeneralizedAcceleration(*accel_data_, cf, &ga);
  EXPECT_GT(ga[0], 0);
}

// Tests the rod in a two-point sliding configuration.
TEST_F(Constraint2DSolverTest, TwoPointSliding) {
  // Set the state of the rod to resting on its side with horizontal velocity.
  SetRodToRestingHorizontalConfig();
  ContinuousState<double>& xc = *context_->
      get_mutable_continuous_state();
  xc[3] = 1.0;

  // Set the coefficient of friction.
  rod_->set_mu_coulomb(0.0);

  // Compute the problem data.
  CalcConstraintAccelProblemData(accel_data_.get());

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveConstraintProblem(cfm_, *accel_data_, &cf);

  // Get the contact tangent velocities.
  std::vector<Vector2d> contacts;
  std::vector<double> tangent_vels;
  rod_->GetContactPoints(*context_, &contacts);
  rod_->GetContactPointsTangentVelocities(*context_, contacts, &tangent_vels);

  // Construct the contact frame.
  std::vector<Matrix2<double>> frames;
  frames.push_back(
    GetSlidingContactFrameToWorldTransform(tangent_vels.front()));
  frames.push_back(GetSlidingContactFrameToWorldTransform(tangent_vels.back()));

  // Verify that the x-axis of the contact frame, which corresponds to the
  // contact normal, points along the world y-axis.
  EXPECT_LT(std::fabs(frames.back().col(0).dot(Vector2<double>::UnitY()) - 1.0),
    std::numeric_limits<double>::epsilon());

  // Verify that the y-axis of the contact frame, which corresponds to the
  // direction of sliding, points along the world x-axis.
  EXPECT_LT(std::fabs(frames.back().col(1).dot(Vector2<double>::UnitX()) - 1.0),
    std::numeric_limits<double>::epsilon());

  // Get the contact forces expressed in the contact frame.
  std::vector<Vector2<double>> contact_forces;
  ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
      *accel_data_, frames, &contact_forces);

  // Verify that the number of contact force vectors is correct.
  ASSERT_EQ(contact_forces.size(), 2);

  // Verify that there are no non-sliding frictional forces.
  EXPECT_TRUE(accel_data_->non_sliding_contacts.empty());
  const int nc = accel_data_->sliding_contacts.size();
  EXPECT_EQ(cf.size(), nc);

  // Verify that the normal contact forces exactly oppose gravity (the
  // frictional forces should be zero).
  const double mg = std::fabs(rod_->get_gravitational_acceleration()) *
      rod_->get_rod_mass();
  EXPECT_NEAR(std::fabs(contact_forces.front()[0]) +
              std::fabs(contact_forces.back()[0]), mg, eps_);
  EXPECT_NEAR(std::fabs(contact_forces.front()[1]) +
              std::fabs(contact_forces.back()[1]), 0, eps_);
}

// Tests the rod in a two-point contacting configuration *realized through
// a configuration limit constraint*. No frictional forces are applied, so
// any velocity projections along directions other than the contact normal
// will be irrelevant.
TEST_F(Constraint2DSolverTest, TwoPointAsLimit) {
  // Set the state of the rod to resting on its side.
  SetRodToRestingHorizontalConfig();

  // First, construct the acceleration-level problem data as normal to set
  // inertia solver and external forces.
  CalcConstraintAccelProblemData(accel_data_.get());

  // Construct the problem as a limit constraint preventing movement in the
  // downward direction.
  const int ngc = get_rod_num_coordinates();
  accel_data_->sliding_contacts.resize(0);
  accel_data_->non_sliding_contacts.resize(0);
  accel_data_->mu_sliding.resize(0);
  accel_data_->mu_non_sliding.resize(0);
  accel_data_->r.resize(0);
  accel_data_->N_mult = [](const VectorX<double>&) {
    return VectorX<double>(0);
  };
  accel_data_->Ndot_x_v.resize(0);
  accel_data_->F_mult = [](const VectorX<double>&) {
    return VectorX<double>(0);
  };
  accel_data_->F_transpose_mult = [ngc](const VectorX<double>&) {
    return VectorX<double>::Zero(ngc);
  };
  accel_data_->Fdot_x_v.resize(0);
  accel_data_->Ldot_x_v.resize(1);
  accel_data_->N_minus_muQ_transpose_mult = [ngc](const VectorX<double>&) {
    return VectorX<double>::Zero(ngc);
  };

  // Set the Jacobian entry- in this case, the limit is a lower limit on the
  // second coordinate (vertical position).
  accel_data_->num_limit_constraints = 1;
  MatrixX<double> L(accel_data_->num_limit_constraints, ngc);
  L.setZero();
  L(0, 1) = 1;
  accel_data_->L_mult = [&L](const VectorX<double>& v) -> VectorX<double> {
    return L * v;
  };
  accel_data_->L_transpose_mult = [&L](const VectorX<double>& v) ->
    VectorX<double> {
    return L.transpose() * v;
  };
  accel_data_->Ldot_x_v.setZero();

  // Compute the constraint forces.
  VectorX<double> cf;
  solver_.SolveConstraintProblem(cfm_, *accel_data_, &cf);

  // Verify the size of cf is as expected.
  EXPECT_EQ(cf.size(), 1);

  // Verify that the normal force exactly opposes gravity.
  const double mg = std::fabs(rod_->get_gravitational_acceleration()) *
      rod_->get_rod_mass();
  EXPECT_NEAR(cf[0], mg, 10 * std::numeric_limits<double>::epsilon());

  // Set the Jacobian entry- in this case, the limit is an upper limit on the
  // second coordinate (vertical position).
  L *= -1;

  // Reverse the external force (gravity) on the rod.
  accel_data_->f *= -1;

  // Recompute the constraint forces, and verify that they're still equal
  // to the force from gravity. Note: if the forces were to be applied to the
  // rod, one will need to compute Lᵀcf[0] to obtain the generalized force;
  // this is how we can handle upper and lower limits with only non-negativity
  // constraints.
  solver_.SolveConstraintProblem(cfm_, *accel_data_, &cf);
  EXPECT_EQ(cf.size(), 1);
  EXPECT_NEAR(cf[0], mg, 10 * std::numeric_limits<double>::epsilon());
}

// Tests the rod in a two-point configuration *realized through a configuration
// limit constraint*, velocity-level version. No frictional forces are applied,
// so any velocity projections along directions other than the contact normal
// will be irrelevant.
TEST_F(Constraint2DSolverTest, TwoPointImpactAsLimit) {
  // Set the state of the rod to impacting on its side.
  SetRodToSlidingImpactingHorizontalConfig();
  ContinuousState<double>& xc = *context_->
      get_mutable_continuous_state();
  const double vert_vel = xc[4];

  // First, construct the velocity-level problem data as normal to set
  // inertia solver and external forces.
  CalcConstraintVelProblemData(vel_data_.get());

  // Construct the problem as a limit constraint preventing movement in the
  // downward direction.
  const int ngc = 3;  // number of generalized coordinates for the 2D rod.
  vel_data_->mu.resize(0);
  vel_data_->r.resize(0);
  vel_data_->N_mult = [](const VectorX<double>&) {
    return VectorX<double>(0);
  };
  vel_data_->N_transpose_mult = [ngc](const VectorX<double>&) {
    return VectorX<double>::Zero(ngc);
  };
  vel_data_->F_mult = [](const VectorX<double>&) {
    return VectorX<double>(0);
  };
  vel_data_->F_transpose_mult = [ngc](const VectorX<double>&) {
    return VectorX<double>::Zero(ngc);
  };
  vel_data_->num_limit_constraints = 1;

  // Set the Jacobian entry- in this case, the limit is a lower limit on the
  // second coordinate (vertical position).
  MatrixX<double> L(vel_data_->num_limit_constraints, ngc);
  L.setZero();
  L(0, 1) = 1;
  vel_data_->L_mult = [&L](const VectorX<double>& v) -> VectorX<double> {
    return L * v;
  };
  vel_data_->L_transpose_mult = [&L](const VectorX<double>& v) ->
    VectorX<double> {
    return L.transpose() * v;
  };

  // Compute the constraint impulses.
  VectorX<double> cf;
  solver_.SolveImpactProblem(cfm_, *vel_data_, &cf);

  // Verify the size of cf is as expected.
  EXPECT_EQ(cf.size(), 1);

  // Verify that the normal force exactly opposes the momentum.
  const double mv = std::fabs(vert_vel) * rod_->get_rod_mass();
  EXPECT_NEAR(cf[0], mv, 10 * std::numeric_limits<double>::epsilon());

  // Set the Jacobian entry- in this case, the limit is an upper limit on the
  // second coordinate (vertical position).
  L *= -1;

  // Reverse the velocity on the rod.
  vel_data_->v *= -1;

  // Recompute the constraint impulses, and verify that they're still equal
  // to the momentum. Note: if the impulses were to be applied to the
  // rod, one will need to compute Lᵀcf[0] to obtain the generalized impulse;
  // this is how we can handle upper and lower limits with only non-negativity
  // constraints.
  solver_.SolveImpactProblem(cfm_, *vel_data_, &cf);
  EXPECT_EQ(cf.size(), 1);
  EXPECT_NEAR(cf[0], mv, 10 * std::numeric_limits<double>::epsilon());
}

// Tests the rod in a single point sliding configuration.
TEST_F(Constraint2DSolverTest, SinglePointSliding) {
  // Set the state of the rod to resting on its side with horizontal velocity.
  SetRodToRestingVerticalConfig();
  ContinuousState<double>& xc = *context_->
      get_mutable_continuous_state();
  xc[3] = 1.0;

  // Set the coefficient of friction.
  rod_->set_mu_coulomb(0.0);

  // Compute the problem data.
  CalcConstraintAccelProblemData(accel_data_.get());

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveConstraintProblem(cfm_, *accel_data_, &cf);

  // Verify that there are no non-sliding frictional forces.
  EXPECT_TRUE(accel_data_->non_sliding_contacts.empty());
  const int nc = accel_data_->sliding_contacts.size();
  EXPECT_EQ(cf.size(), nc);

  // Get the contact tangent velocities.
  std::vector<Vector2d> contacts;
  std::vector<double> tangent_vels;
  rod_->GetContactPoints(*context_, &contacts);
  rod_->GetContactPointsTangentVelocities(*context_, contacts, &tangent_vels);

  // Construct the contact frame.
  std::vector<Matrix2<double>> frames;
  frames.push_back(
      GetSlidingContactFrameToWorldTransform(tangent_vels.front()));

  // Verify that the x-axis of the contact frame, which corresponds to the
  // contact normal, points along the world y-axis.
  EXPECT_LT(std::fabs(frames.back().col(0).dot(Vector2<double>::UnitY()) - 1.0),
      std::numeric_limits<double>::epsilon());

  // Verify that the y-axis of the contact frame, which corresponds to the
  // direction of sliding, points along the world x-axis.
  EXPECT_LT(std::fabs(frames.back().col(1).dot(Vector2<double>::UnitX()) - 1.0),
            std::numeric_limits<double>::epsilon());

  // Get the contact forces expressed in the contact frame.
  std::vector<Vector2<double>> contact_forces;
  ConstraintSolver<double>::CalcContactForcesInContactFrames(cf,
      *accel_data_, frames, &contact_forces);

  // Verify that the number of contact force vectors is correct.
  ASSERT_EQ(contact_forces.size(), 1);

  // Verify that the normal contact forces exactly oppose gravity and there are
  // no frictional forces).
  const double mg = std::fabs(rod_->get_gravitational_acceleration()) *
      rod_->get_rod_mass();
  EXPECT_NEAR(contact_forces.front()[0], mg, eps_);
  EXPECT_NEAR(contact_forces.front()[1], 0, eps_);
}

}  // namespace
}  // namespace constraint
}  // namespace multibody
}  // namespace drake
