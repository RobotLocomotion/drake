#include "drake/multibody/rigid_contact/rigid_contact_solver.h"
#include "drake/examples/rod2d/rod2d.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

using drake::systems::ContinuousState;
using drake::systems::Context;
using drake::examples::rod2d::Rod2D;

namespace drake {
namespace multibody {
namespace rigid_contact {
namespace {

class RigidContact2DSolverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rod_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kPiecewiseDAE, 0);
    context_ = rod_->CreateDefaultContext();
  }

  double cfm_{1e-8};   // Regularization parameter.
  RigidContactSolver<double> solver_;
  std::unique_ptr<Rod2D<double>> rod_;
  std::unique_ptr<Context<double>> context_;
  RigidContactAccelProblemData<double> data_;
}

// Tests the rod in a ballistic configuration (non-contacting) configuration.
TEST_F(RigidContact2DSolverTest, NonContacting) {
  // Set the state of the rod to reseting on its side with no velocity.
  SetRestingHorizontal();

  // Set the vertical position to strictly positive.
  ContinuousState<double>& xc = *context_->
      get_mutable_continuous_state();
  xc[1] = 1.0;

  // Compute the rigid contact data.
  SetProblemData(&data_, 1.0 /* Coulomb friction coefficient */);

  // Verify there are no contacts.
  EXPECT_TRUE(data_.sliding_contacts.empty());
  EXPECT_TRUE(data_.non_sliding_contacts.empty());

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveContactProblem(cfm_, data_, &cf);

  // Verify that the contact forces are zero.
  EXPECT_LT(cf.norm(), cfm_);
}

// Tests the rod in a two-point non-sliding configuration.
TEST_F(RigidContact2DSolverTest, TwoPointNonSliding) {
  // Set the state of the rod to reseting on its side with no velocity.
  SetRestingHorizontal();

  // Compute the rigid contact data.
  SetProblemData(&data_, 1.0 /* Coulomb friction coefficient */);

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveContactProblem(cfm_, data_, &cf);

  // Verify that there are no frictional forces.
  EXPECT_TRUE(data_.sliding_contacts.empty());
  const int nc = data_.non_sliding_contacts.size();
  EXPECT_LT(cf.segment(nc, cf.size() - nc).norm(), 10 * cfm_);

  // Verify that the normal contact forces exactly oppose gravity (there should
  // be no frictional forces).
  const double mg = GetRodGravitationalForce().norm();
  EXPECT_NEAR(cf.segment(0, nc).lpNorm<1>(), mg, 10 * cfm_);
}

// Tests the rod in a two-point sliding configuration.
TEST_F(RigidContact2DSolverTest, TwoPointSliding) {
  // Set the state of the rod to reseting on its side with horizontal velocity.
  SetRestingHorizontal();
  ContinuousState<double>& xc = *context_->
      get_mutable_continuous_state();
  xc[3] = 1.0;

  // Compute the rigid contact data.
  SetProblemData(&data_, 0.0 /* frictionless contact */);

  // Compute the contact forces.
  VectorX<double> cf;
  solver_.SolveContactProblem(cfm_, data_, &cf);

  // Verify that there are no frictional forces.
  EXPECT_TRUE(data_.non_sliding_contacts.empty());
  const int nc = data_.sliding_contacts.size();
  EXPECT_LT(cf.segment(nc, cf.size() - nc).norm(), 10 * cfm_);

  // Verify that the normal contact forces exactly oppose gravity (there should
  // be no frictional forces).
  const double mg = GetRodGravitationalForce().norm();
  EXPECT_NEAR(cf.segment(0, nc).lpNorm<1>(), mg, 10 * cfm_);
}

}  // namespace
}  // namespace rigid_contact
}  // namespace multibody
}  // namespace drake
