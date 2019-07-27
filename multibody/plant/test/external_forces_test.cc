#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/plant/test/kuka_iiwa_model_tests.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/frame.h"

namespace drake {

namespace multibody {
namespace multibody_plant {

using test::KukaIiwaModelTests;

namespace {

TEST_F(KukaIiwaModelTests, ExternalBodyForces) {
  SetArbitraryConfiguration();

  // An arbitrary point on the end effector frame E.
  Vector3<double> p_EP(0.1, -0.05, 0.3);

  // An arbitrary spatial force applied at point P on the end effector,
  // expressed in the end effector frame E.
  const SpatialForce<double> F_Ep_E(
      Vector3<double>(1.0, 2.0, 3.0), Vector3<double>(-5.0, -4.0, -2.0));

  int nv = plant_->num_velocities();

  // Build a vector of generalized accelerations with arbitrary values.
  const VectorX<double> vdot = VectorX<double>::LinSpaced(nv, -5.0, 5.0);

  // Compute inverse dynamics with an externally applied force.
  MultibodyForces<double> forces(*plant_);
  end_effector_link_->AddInForce(
      *context_, p_EP, F_Ep_E, end_effector_link_->body_frame(), &forces);
  const VectorX<double> tau_id =
      plant_->CalcInverseDynamics(*context_, vdot, forces);
  { // Repeat the computation to confirm the heap behavior.  We allow the
    // method to heap-allocate 2 temporaries and 1 return value.
    drake::test::LimitMalloc guard({ .max_num_allocations = 3 });
    auto dummy = plant_->CalcInverseDynamics(*context_, vdot, forces);
    unused(dummy);
  }

  MatrixX<double> M(nv, nv);
  plant_->CalcMassMatrixViaInverseDynamics(*context_, &M);

  VectorX<double> C(nv);
  plant_->CalcBiasTerm(*context_, &C);

  // Frame Jacobian for point p_EP.
  MatrixX<double> Jv_WEp(6, nv);
  const Frame<double>& frame_W = plant_->world_frame();
  plant_->CalcJacobianSpatialVelocity(*context_,
                                      multibody::JacobianWrtVariable::kV,
                                      end_effector_link_->body_frame(), p_EP,
                                      frame_W, frame_W, &Jv_WEp);

  // Compute the expected value of inverse dynamics when external forcing is
  // considered.
  const math::RotationMatrix<double>& R_WE =
      end_effector_link_->EvalPoseInWorld(*context_).rotation();
  const SpatialForce<double> F_Ep_W = R_WE * F_Ep_E;
  const VectorX<double> tau_id_expected =
      M * vdot + C - Jv_WEp.transpose() * F_Ep_W.get_coeffs();

  // Numerical tolerance used to verify numerical results.
  // Error loss is expected in both forward kinematics and inverse dynamics
  // computations since errors accumulate during inboard/outboard passes.
  const double kTolerance = 50 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(
      tau_id, tau_id_expected,
      kTolerance, MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, BodyForceApi) {
  SetArbitraryConfiguration();
  MultibodyForces<double> forces(*plant_);
  Vector6<double> F_expected;
  F_expected << 1, 2, 3, 4, 5, 6;
  SpatialForce<double> F_Bo_W(F_expected);
  end_effector_link_->AddInForceInWorld(*context_, F_Bo_W, &forces);
  EXPECT_TRUE(CompareMatrices(
      end_effector_link_->GetForceInWorld(*context_, forces).get_coeffs(),
      F_expected));
  // Test frame-specfic, and ensure we accumulate.
  end_effector_link_->AddInForce(
      *context_, Vector3<double>::Zero(), F_Bo_W, plant_->world_frame(),
      &forces);
  EXPECT_TRUE(CompareMatrices(
      end_effector_link_->GetForceInWorld(*context_, forces).get_coeffs(),
      2 * F_expected));
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
