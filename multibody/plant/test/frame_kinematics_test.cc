/// @file
/// This file contains tests for kinematics methods in the Frame class.
/// There are similar tests in multibody_plant_kinematics_test.cc which test
/// kinematics methods in the MultibodyPlant class.
#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/plant/test/kuka_iiwa_model_tests.h"
#include "drake/multibody/test_utilities/add_fixed_objects_to_plant.h"
#include "drake/multibody/test_utilities/spatial_derivative.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {

namespace multibody {

using math::RigidTransformd;
using test::KukaIiwaModelTests;

namespace {

// Numerical tolerance used to verify numerical results.
constexpr double kTolerance = 32 * std::numeric_limits<double>::epsilon();

TEST_F(KukaIiwaModelTests, FramesKinematics) {
  SetArbitraryConfigurationAndMotion();

  // Test for a simple identity case of CalcRelativeTransform().
  const RigidTransformd X_HH =
      plant_->CalcRelativeTransform(*context_, *frame_H_, *frame_H_);
  EXPECT_TRUE(CompareMatrices(X_HH.rotation().matrix(),
                              Matrix3<double>::Identity(), kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(X_HH.translation(), Vector3<double>::Zero(),
                              kTolerance, MatrixCompareType::relative));

  // Test for a simple identity case of CalcRelativeRotationMatrix().
  const RotationMatrixd R_HH =
      plant_->CalcRelativeRotationMatrix(*context_, *frame_H_, *frame_H_);
  EXPECT_TRUE(CompareMatrices(R_HH.matrix(), Matrix3<double>::Identity(),
                              kTolerance, MatrixCompareType::relative));

  const RigidTransform<double>& X_WE =
      end_effector_link_->EvalPoseInWorld(*context_);
  const RigidTransform<double> X_WH = frame_H_->CalcPoseInWorld(*context_);
  const RigidTransform<double> X_WH_expected = X_WE * X_EH_;
  EXPECT_TRUE(CompareMatrices(X_WH.GetAsMatrix34(),
                              X_WH_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));

  // Alternatively, we can get the pose X_WE using the plant's output port for
  // poses.
  const auto& X_WB_all =
      plant_->get_body_poses_output_port()
          .Eval<std::vector<RigidTransform<double>>>(*context_);
  ASSERT_EQ(X_WB_all.size(), plant_->num_bodies());
  const RigidTransform<double>& X_WE_from_port =
      X_WB_all[end_effector_link_->index()];
  EXPECT_TRUE(CompareMatrices(X_WE.GetAsMatrix34(),
                              X_WE_from_port.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));

  // Verify the invariance X_WB_all[0] = RigidTransform<double>::Identity().
  EXPECT_TRUE(
      CompareMatrices(X_WB_all[0].GetAsMatrix34(),
                      RigidTransform<double>::Identity().GetAsMatrix34(),
                      kTolerance, MatrixCompareType::relative));

  const RotationMatrix<double> R_WH =
      frame_H_->CalcRotationMatrixInWorld(*context_);
  const RotationMatrix<double>& R_WH_expected = X_WH_expected.rotation();
  EXPECT_TRUE(CompareMatrices(R_WH.matrix(), R_WH_expected.matrix(), kTolerance,
                              MatrixCompareType::relative));

  const RigidBody<double>& link3 = plant_->GetBodyByName("iiwa_link_3");
  const Frame<double>& frame_L3 = link3.body_frame();
  const RigidTransform<double> X_HL3 = frame_L3.CalcPose(*context_, *frame_H_);
  const RigidTransform<double> X_WL3 = frame_L3.CalcPoseInWorld(*context_);
  const RigidTransform<double> X_HL3_expected = X_WH.InvertAndCompose(X_WL3);
  EXPECT_TRUE(CompareMatrices(X_HL3.GetAsMatrix34(),
                              X_HL3_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));

  const RotationMatrix<double> R_HL3 =
      frame_L3.CalcRotationMatrix(*context_, *frame_H_);
  const RotationMatrix<double> R_WL3 =
      frame_L3.CalcRotationMatrixInWorld(*context_);
  const RotationMatrix<double> R_HL3_expected = R_WH.InvertAndCompose(R_WL3);
  EXPECT_TRUE(CompareMatrices(R_HL3.matrix(), R_HL3_expected.matrix(),
                              kTolerance, MatrixCompareType::relative));

  const SpatialVelocity<double> V_WE =
      end_effector_link_->EvalSpatialVelocityInWorld(*context_);
  const SpatialVelocity<double> V_WH =
      frame_H_->CalcSpatialVelocityInWorld(*context_);
  const Vector3<double> p_EH =
      frame_H_->GetFixedPoseInBodyFrame().translation();
  const RotationMatrix<double>& R_WE = X_WE.rotation();
  const Vector3<double> p_EH_W = R_WE * p_EH;
  const SpatialVelocity<double> V_WH_expected = V_WE.Shift(p_EH_W);
  EXPECT_TRUE(CompareMatrices(V_WH.get_coeffs(), V_WH_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));

  // Alternatively, we can get the spatial velocity V_WE using the plant's
  // output port for spatial velocities.
  const auto& V_WB_all =
      plant_->get_body_spatial_velocities_output_port()
          .Eval<std::vector<SpatialVelocity<double>>>(*context_);
  ASSERT_EQ(V_WB_all.size(), plant_->num_bodies());
  const SpatialVelocity<double>& V_WE_from_port =
      V_WB_all[end_effector_link_->index()];
  EXPECT_EQ(V_WE.get_coeffs(), V_WE_from_port.get_coeffs());

  // Spatial velocity of link 3 measured in frame H and expressed in the
  // end-effector frame E.
  const Frame<double>& frame_E = end_effector_link_->body_frame();
  const SpatialVelocity<double> V_HL3_E =
      frame_L3.CalcSpatialVelocity(*context_, *frame_H_, frame_E);
  // Compute V_HL3_E_expected.
  const RotationMatrix<double> R_EW = R_WE.inverse();
  const SpatialVelocity<double> V_WH_E = R_EW * V_WH;
  const math::RotationMatrix<double> R_EH =
      frame_H_->GetFixedPoseInBodyFrame().rotation();
  const Vector3<double> p_HL3_E = R_EH * X_HL3.translation();
  const SpatialVelocity<double> V_WL3_E =
      R_EW * link3.EvalSpatialVelocityInWorld(*context_);
  // V_WL3_E = V_WH_E.Shift(p_HL3_E) + V_HL3_E
  const SpatialVelocity<double> V_HL3_E_expected =
      V_WL3_E - V_WH_E.Shift(p_HL3_E);
  EXPECT_TRUE(CompareMatrices(V_HL3_E.get_coeffs(),
                              V_HL3_E_expected.get_coeffs(), kTolerance,
                              MatrixCompareType::relative));

  // For frame H (which is a fixed offset frame fixed to end-effector E),
  // verify that the rotational and translational parts of frame H's spatial
  // velocity measured in frame E are zero.
  const Frame<double>& frame_W = plant_->world_frame();
  const SpatialVelocity<double> V_EH_W =
      frame_H_->CalcSpatialVelocity(*context_, frame_E, frame_W);
  EXPECT_TRUE(CompareMatrices(V_EH_W.get_coeffs(), Vector6<double>::Zero(),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, FrameAngularVelocity) {
  SetArbitraryConfigurationAndMotion();

  // Verify that fixed-offset Frame_H (which is fixed to end-effector frame_E)
  // has a zero angular velocity in frame E, i.e., ω_EH = 0.
  const FixedOffsetFrame<double>& frame_H = *frame_H_;
  const Frame<double>& frame_E = end_effector_link_->body_frame();
  const Vector3<double> w_EH_E =
      frame_H.CalcAngularVelocity(*context_, frame_E, frame_E);
  EXPECT_TRUE(CompareMatrices(w_EH_E, Vector3<double>::Zero(), kTolerance,
                              MatrixCompareType::relative));

  // Verify the direct calculation of ω_WE_W (frame H's angular velocity in
  // world W, expressed in W) by comparing it to the rotational part of V_WE_W
  // (frame H's spatial velocity in world W, expressed in W).
  const Vector3<double>& w_WH_W = frame_H.EvalAngularVelocityInWorld(*context_);
  const SpatialVelocity<double> V_WH_W =
      frame_H.CalcSpatialVelocityInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(w_WH_W, V_WH_W.rotational(), kTolerance,
                              MatrixCompareType::relative));

  // Verify CalcAngularVelocity() against EvalAngularVelocityInWorld().
  const RigidBody<double>& link3 = plant_->GetBodyByName("iiwa_link_3");
  const Frame<double>& frame_L3 = link3.body_frame();
  const Frame<double>& frame_W = plant_->world_frame();
  const Vector3<double> w_WL3_W =
      frame_L3.CalcAngularVelocity(*context_, frame_W, frame_W);
  const Vector3<double> w_WL3_W_expected =
      frame_L3.EvalAngularVelocityInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(w_WL3_W, w_WL3_W_expected, kTolerance,
                              MatrixCompareType::relative));

  // Verify CalcAngularVelocity() is consistent with the negation property for
  // angular velocities, e.g., ω_AB = -ω_BA or specifically ω_L3H = -ω_EL3.
  const Vector3<double> w_L3E_H =
      frame_E.CalcAngularVelocity(*context_, frame_L3, frame_H);
  const Vector3<double> w_EL3_H =
      frame_L3.CalcAngularVelocity(*context_, frame_E, frame_H);
  EXPECT_NE(w_L3E_H, Vector3<double>::Zero());
  EXPECT_TRUE(CompareMatrices(w_L3E_H, -w_EL3_H, kTolerance,
                              MatrixCompareType::relative));

  // Verify CalcAngularVelocity() produces the same results as
  // CalcSpatialVelocity().rotational() and handles the expressed-in frame.
  const Vector3<double> w_L3W_E_expected =
      frame_W.CalcSpatialVelocity(*context_, frame_L3, frame_E).rotational();
  const Vector3<double> w_L3W_E =
      frame_W.CalcAngularVelocity(*context_, frame_L3, frame_E);
  EXPECT_TRUE(CompareMatrices(w_L3W_E_expected, w_L3W_E, kTolerance,
                              MatrixCompareType::relative));

  // Verify CalcAngularVelocity() produces the same results as
  // CalcRelativeSpatialVelocity().rotational().
  const Vector3<double> w_L3E_E_expected =
      frame_E.CalcRelativeSpatialVelocity(*context_, frame_L3, frame_H, frame_E)
          .rotational();
  const Vector3<double> w_L3E_E =
      frame_E.CalcAngularVelocity(*context_, frame_L3, frame_E);
  EXPECT_TRUE(CompareMatrices(w_L3E_E_expected, w_L3E_E, kTolerance,
                              MatrixCompareType::relative));

  // Verify CalcAngularVelocity() is consistent with the angular velocity
  // addition theorem ω_WE = ω_WH + ω_HL3 + ω_L3E, where all these vectors are
  // expressed in the world frame W.
  const Vector3<double> w_WE_W_expected =
      frame_E.CalcSpatialVelocityInWorld(*context_).rotational();
  const Vector3<double> w_HL3_W =
      frame_L3.CalcAngularVelocity(*context_, frame_H, frame_W);
  const Vector3<double> w_L3E_W =
      frame_E.CalcAngularVelocity(*context_, frame_L3, frame_W);
  const Vector3<double> w_WE_W = w_WH_W + w_HL3_W + w_L3E_W;
  EXPECT_TRUE(CompareMatrices(w_WE_W_expected, w_WE_W, kTolerance,
                              MatrixCompareType::relative));

  // Verify CalcAngularVelocity() handles the expressed-in frame.
  const RotationMatrix<double> R_WE =
      frame_E.CalcRotationMatrixInWorld(*context_);
  const RotationMatrix<double> R_EW = R_WE.inverse();
  const Vector3<double> w_WE_E_expected = R_EW * w_WE_W_expected;
  const Vector3<double> w_WE_E =
      frame_E.CalcAngularVelocity(*context_, frame_W, frame_E);
  EXPECT_TRUE(CompareMatrices(w_WE_E_expected, w_WE_E, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, FramesCalcRelativeSpatialVelocity) {
  SetArbitraryConfigurationAndMotion();

  // Directly verify the calculation of V_W_L3H_W (frame H's spatial velocity
  // relative to frame L3, measured and expressed in the world frame W) is the
  // same as its definition which is V_WH_W - V_WL3_W.
  // Note: Frame H  is a fixed offset frame, fixed to end-effector E.
  const FixedOffsetFrame<double>& frame_H = *frame_H_;
  const RigidBody<double>& link3 = plant_->GetBodyByName("iiwa_link_3");
  const Frame<double>& frame_L3 = link3.body_frame();
  const SpatialVelocity<double> V_W_L3H_W =
      frame_H.CalcRelativeSpatialVelocityInWorld(*context_, frame_L3);
  const SpatialVelocity<double> V_WL3_W =
      frame_L3.CalcSpatialVelocityInWorld(*context_);
  const SpatialVelocity<double> V_WH_W =
      frame_H.CalcSpatialVelocityInWorld(*context_);
  const SpatialVelocity<double> V_W_L3H_W_expected = V_WH_W - V_WL3_W;
  EXPECT_TRUE(CompareMatrices(V_W_L3H_W.get_coeffs(),
                              V_W_L3H_W_expected.get_coeffs(), kTolerance,
                              MatrixCompareType::relative));

  // Also verify that frame H's spatial velocity relative to frame L3 is the
  // negative of frame L3's spatial velocity relative to frame H.
  const SpatialVelocity<double> V_W_HL3_W =
      frame_L3.CalcRelativeSpatialVelocityInWorld(*context_, frame_H);
  EXPECT_TRUE(CompareMatrices(V_W_L3H_W.get_coeffs(), -V_W_HL3_W.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));

  // Also verify that the rotational part of V_W_L3H_W is simply ω_L3H_W.
  const Frame<double>& frame_W = plant_->world_frame();
  const Vector3<double>& w_L3H_W = V_W_L3H_W.rotational();
  const Vector3<double> w_L3H_W_expected =
      frame_H.CalcSpatialVelocity(*context_, frame_L3, frame_W).rotational();
  EXPECT_TRUE(CompareMatrices(w_L3H_W, w_L3H_W_expected, kTolerance,
                              MatrixCompareType::relative));

  // Verify that relative spatial velocity is the same as absolute spatial if
  // the "relative-to" frame is the same as the "measured-in" frame.
  // First perform this test when the "measured-in" frame is the world frame W.
  // Next, perform this test when the "measured-in" frame is frame_L3.
  const SpatialVelocity<double> V_W_WH_W =
      frame_H.CalcRelativeSpatialVelocityInWorld(*context_, frame_W);
  const SpatialVelocity<double> V_W_WH_W_expected =
      frame_H.CalcSpatialVelocityInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(V_W_WH_W.get_coeffs(),
                              V_W_WH_W_expected.get_coeffs(), kTolerance,
                              MatrixCompareType::relative));

  const SpatialVelocity<double> V_L3_L3H_W =
      frame_H.CalcRelativeSpatialVelocity(*context_, frame_L3, frame_L3,
                                          frame_W);
  const SpatialVelocity<double> V_L3_L3H_W_expected =
      frame_H.CalcSpatialVelocity(*context_, frame_L3, frame_W);
  EXPECT_TRUE(CompareMatrices(V_L3_L3H_W.get_coeffs(),
                              V_L3_L3H_W_expected.get_coeffs(), kTolerance,
                              MatrixCompareType::relative));

  // For frame H (which is a fixed offset frame fixed to end-effector E),
  // verify that the rotational part of frame H's spatial velocity relative to
  // frame E measured in the world frame W is zero and verify the translational
  // part is equal to ω_WE x p_EoHo.
  const Frame<double>& frame_E = end_effector_link_->body_frame();
  const SpatialVelocity<double> V_W_EH_W =
      frame_H.CalcRelativeSpatialVelocityInWorld(*context_, frame_E);
  const Vector3<double>& w_EH_W = V_W_EH_W.rotational();
  EXPECT_TRUE(CompareMatrices(w_EH_W, Vector3<double>::Zero(), kTolerance,
                              MatrixCompareType::relative));
  const Vector3<double>& p_EoHo_E = X_EH_.translation();
  const Vector3<double>& w_WE_W = frame_E.EvalAngularVelocityInWorld(*context_);
  const RotationMatrix<double> R_WE =
      frame_E.CalcRotationMatrixInWorld(*context_);
  const Vector3<double> p_EoHo_W = R_WE * p_EoHo_E;
  const Vector3<double> w_cross_p = w_WE_W.cross(p_EoHo_W);
  EXPECT_TRUE(CompareMatrices(V_W_EH_W.translational(), w_cross_p, kTolerance,
                              MatrixCompareType::relative));

  // Verify that the calculation of separation speed (1ˢᵗ time-derivative of
  // separation distance) between Ho (frame H's origin) and L3o (frame L3's
  // origin) can be calculated by point Ho's translational velocity relative
  // to L3o, independent of the measured-in frame.
  const Vector3<double>& v_W_L3H_W = V_W_L3H_W.translational();
  const Vector3<double>& v_L3_L3H_W = V_L3_L3H_W.translational();
  // Form the unit vector directed from L3o to Ho.
  const Vector3<double> p_L3oHo_L3 =
      frame_H.CalcPose(*context_, frame_L3).translation();
  const RotationMatrix<double> R_WL3 =
      frame_L3.CalcRotationMatrixInWorld(*context_);
  const Vector3<double> p_L3oHo_W = R_WL3 * p_L3oHo_L3;
  const double separation_distance = p_L3oHo_W.norm();
  const Vector3<double> u_L3oHo_W = p_L3oHo_W / separation_distance;
  const double separation_speed1 = v_W_L3H_W.dot(u_L3oHo_W);
  const double separation_speed2 = v_L3_L3H_W.dot(u_L3oHo_W);
  EXPECT_NEAR(separation_speed1, separation_speed2, kTolerance);
}

TEST_F(KukaIiwaModelTests, CalcSpatialAcceleration) {
  SetArbitraryConfigurationAndMotion();

  // Verify a short-cut return from Frame::CalcSpatialAccelerationInWorld()
  // when dealing with a body_frame (as opposed to a generic frame).
  // Compare results with the A_WE_W from an associated plant method.
  const Frame<double>& frame_E = end_effector_link_->body_frame();
  const SpatialAcceleration<double> A_WE_W =
      frame_E.CalcSpatialAccelerationInWorld(*context_);
  const SpatialAcceleration<double> A_WE_W_expected1 =
      plant_->EvalBodySpatialAccelerationInWorld(*context_,
                                                 *end_effector_link_);
  EXPECT_EQ(A_WE_W.get_coeffs(), A_WE_W_expected1.get_coeffs());

  // Also verify A_WE_W against RigidBody::EvalSpatialAccelerationInWorld().
  const SpatialAcceleration<double> A_WE_W_expected2 =
      end_effector_link_->EvalSpatialAccelerationInWorld(*context_);
  EXPECT_EQ(A_WE_W.get_coeffs(), A_WE_W_expected2.get_coeffs());

  // Also verify A_WE_W from the plant's output port for spatial acceleration.
  const auto& A_WB_all =
      plant_->get_body_spatial_accelerations_output_port()
          .Eval<std::vector<SpatialAcceleration<double>>>(*context_);
  ASSERT_EQ(A_WB_all.size(), plant_->num_bodies());
  const SpatialAcceleration<double>& A_WE_W_expected3 =
      A_WB_all[end_effector_link_->index()];
  EXPECT_EQ(A_WE_W.get_coeffs(), A_WE_W_expected3.get_coeffs());

  // Verify A_WH_W, frame_H's spatial acceleration in world W, expressed in W.
  // Do this by differentiating the spatial velocity V_WH_W.
  const SpatialAcceleration<double> A_WH_W =
      frame_H_->CalcSpatialAccelerationInWorld(*context_);
  const Frame<double>& frame_W = plant_->world_frame();
  const SpatialAcceleration<double> A_WH_W_expected =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          *plant_, *context_, *frame_H_, frame_W, frame_W);
  EXPECT_TRUE(CompareMatrices(A_WH_W.get_coeffs(), A_WH_W_expected.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));

  // -------------- Start of CalcSpatialAcceleration() tests ----------------
  // For frame H (which is a fixed offset frame fixed to end-effector E),
  // verify that the rotational and translational parts of frame H's spatial
  // acceleration measured in frame E are zero.
  const SpatialAcceleration<double> A_EH_W =
      frame_H_->CalcSpatialAcceleration(*context_, frame_E, frame_W);
  EXPECT_TRUE(CompareMatrices(A_EH_W.get_coeffs(), Vector6<double>::Zero(),
                              kTolerance, MatrixCompareType::relative));

  // Verify special case of frame_L3.CalcSpatialAcceleration() when both the
  // measured-in frame and expressed-in frame are the world frame W.
  const RigidBody<double>& link3 = plant_->GetBodyByName("iiwa_link_3");
  const Frame<double>& frame_L3 = link3.body_frame();
  const SpatialAcceleration<double> A_WL3_W =
      frame_L3.CalcSpatialAcceleration(*context_, frame_W, frame_W);
  const SpatialAcceleration<double> A_WL3_W_expected =
      frame_L3.CalcSpatialAccelerationInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(A_WL3_W.get_coeffs(),
                              A_WL3_W_expected.get_coeffs(), kTolerance,
                              MatrixCompareType::relative));

  // Verify special case of frame_L3.CalcSpatialAcceleration() when the
  // measured-in frame is world W and the expressed-in frame is end-effector E.
  const SpatialAcceleration<double> A_WL3_E =
      frame_L3.CalcSpatialAcceleration(*context_, frame_W, frame_E);
  const RotationMatrix<double> R_WE =
      frame_E.CalcRotationMatrixInWorld(*context_);
  const RotationMatrix<double> R_EW = R_WE.inverse();
  const SpatialAcceleration<double> A_WL3_E_expected = R_EW * A_WL3_W_expected;
  EXPECT_TRUE(CompareMatrices(A_WL3_E.get_coeffs(),
                              A_WL3_E_expected.get_coeffs(), kTolerance,
                              MatrixCompareType::relative));

  // ------ Start of weird/useful CalcSpatialVelocity() test for V_HW_E. ------
  // This is a "warm-up" for the CalcSpatialAcceleration() test for A_HW_E that
  // follows and it generates useful ingredients for that test of A_HW_E.

  // Use CalcSpatialVelocity() to form V_HW_E which is the world frame W's
  // spatial velocity in frame H, expressed in the end-effector frame E.
  const SpatialVelocity<double> V_HW_E =
      frame_W.CalcSpatialVelocity(*context_, *frame_H_, frame_E);

  // Verify the rotational part of V_HW_E via the familiar formula ω_WH = -ω_HW.
  const SpatialVelocity<double> V_WH_W =
      frame_H_->CalcSpatialVelocityInWorld(*context_);
  const Vector3<double>& w_WH_W = V_WH_W.rotational();
  EXPECT_TRUE(CompareMatrices(V_HW_E.rotational(), R_EW * (-w_WH_W), kTolerance,
                              MatrixCompareType::relative));

  // Verify the translational part of V_HW_E via the following calculations.
  //   v_WHo =  DtW(p_WoHo)                    Definition of Ho's velocity in W.
  //         =  DtH(p_WoHo) + ω_WH x p_WoHo  Golden rule vector differentiation.
  //         = -DtH(p_HoWo) + ω_WH x p_WoHo     Use fact that p_HoWo = - p_WoHo.
  //         =      -v_HWo  + ω_WH x p_WoHo    Definition of Wo's velocity in H.
  //   v_HWo = -v_WHo + ω_WH x p_WoHo                   Rearrange previous line.
  const Vector3<double>& v_WHo_W = V_WH_W.translational();
  const RigidTransform<double> X_WH = frame_H_->CalcPoseInWorld(*context_);
  const Vector3<double>& p_WoHo_W = X_WH.translation();
  const Vector3<double> v_HWo_W_expected = -v_WHo_W + w_WH_W.cross(p_WoHo_W);
  EXPECT_TRUE(CompareMatrices(V_HW_E.translational(), R_EW * v_HWo_W_expected,
                              kTolerance, MatrixCompareType::relative));
  // ------ End of weird/useful CalcSpatialVelocity() test for V_HW_E. --------

  // Verify the weird/useful calculation of A_HW_E, which is the world frame W's
  // spatial acceleration in frame H, expressed in the end-effector frame E.
  // Reminder: frame_H is an offset frame fixed to the end-effector E.

  // Use CalcSpatialAcceleration() to form V_HW_E which is the world frame W's
  // spatial acceleration in frame H, expressed in the end-effector frame E.
  const SpatialAcceleration<double> A_HW_E =
      frame_W.CalcSpatialAcceleration(*context_, *frame_H_, frame_E);

  // Verify the rotational part of A_HW_E via the relationship α_HW = -α_WH.
  const Vector3<double>& alpha_WH_W = A_WH_W.rotational();
  EXPECT_TRUE(CompareMatrices(A_HW_E.rotational(), R_EW * (-alpha_WH_W),
                              kTolerance, MatrixCompareType::relative));

  // Verify translational part with by-hand calculations.
  // Verify the translational part of A_HW_E via the following calculations.
  //   a_WHo =  DtW(v_WHo)                 Definition of Ho's acceleration in W.
  //         =  DtW(-v_HWo + ω_WH x p_WoHo)      Substitute from previous v_WHo.
  //         = -DtW(v_HWo) + DtW(ω_WH x p_WoHo)  Distribute DtW() to both terms.
  // DtW(v_HWo) = DtH(v_HWo) + ω_WH x v_HWo  Golden rule vector differentiation.
  //            =     a_HWo  + ω_WH x v_HWo  Definition: Wo's acceleration in H.
  //            =     a_HWo  + ω_WH x (-v_WHo + ω_WH x p_WoHo)
  //            =     a_HWo  - ω_WH x   v_WHo + ω_WH x (ω_WH x p_WoHo)
  // DtW(ω_WH x p_WoHo) = DtW(ω_WH) x p_WoHo + ω_WH x DtW(p_WoHo)  Product rule.
  //                    =      α_WH x p_WoHo + ω_WH x v_WH          Definitions.
  //   a_WHo = -a_HWo + ω_WH x v_HWo - ω_WH x (ω_WH x p_WoHo)
  //         +  α_WH x p_WoHo + ω_WH x v_WHo        Combine terms and rearrange.
  //   a_HWo = -a_WHo + α_WH x p_WoHo - ω_WH x (ω_WH x p_WoHo) + 2ω_WH x v_WHo
  const Vector3<double>& a_WHo_W = A_WH_W.translational();
  const Vector3<double> alf_r = alpha_WH_W.cross(p_WoHo_W);
  const Vector3<double> wwr = w_WH_W.cross(w_WH_W.cross(p_WoHo_W));
  const Vector3<double> twowv = 2 * w_WH_W.cross(v_WHo_W);
  const Vector3<double> a_HWo_W_expected = -a_WHo_W + alf_r - wwr + twowv;
  EXPECT_TRUE(CompareMatrices(A_HW_E.translational(), R_EW * a_HWo_W_expected,
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, FramesCalcRelativeSpatialAcceleration) {
  SetArbitraryConfigurationAndMotion();

  // Verify frame_H's relative spatial acceleration to itself (frame_H),
  // measured in the world frame W is zero.
  // Note: Frame H is a fixed offset frame, fixed to end-effector E.
  const FixedOffsetFrame<double>& frame_H = *frame_H_;
  const SpatialAcceleration<double> A_W_HH_W =
      frame_H.CalcRelativeSpatialAccelerationInWorld(*context_, frame_H);
  EXPECT_TRUE(CompareMatrices(A_W_HH_W.get_coeffs(), Vector6<double>::Zero(),
                              kTolerance, MatrixCompareType::relative));

  // Verify frame_H's relative spatial acceleration to itself (frame_H),
  // measured in frame_L3 is zero.
  const RigidBody<double>& link3 = plant_->GetBodyByName("iiwa_link_3");
  const Frame<double>& frame_L3 = link3.body_frame();
  const Frame<double>& frame_W = plant_->world_frame();
  const SpatialAcceleration<double> A_L3_HH_W =
      frame_H.CalcRelativeSpatialAcceleration(*context_, frame_H, frame_L3,
                                              frame_W);
  EXPECT_TRUE(CompareMatrices(A_L3_HH_W.get_coeffs(), Vector6<double>::Zero(),
                              kTolerance, MatrixCompareType::relative));

  // To ensure the previous test is not degenerate, ensure frame_H's rotational
  // and translational velocities and accelerations in frame_L3 are non-zero.
  const SpatialVelocity<double> V_L3H_W =
      frame_H.CalcSpatialVelocity(*context_, frame_L3, frame_W);
  const SpatialAcceleration<double> A_L3H_W =
      frame_H.CalcSpatialAcceleration(*context_, frame_L3, frame_W);
  EXPECT_FALSE(CompareMatrices(V_L3H_W.rotational(), Vector3<double>::Zero(),
                               kTolerance, MatrixCompareType::absolute));
  EXPECT_FALSE(CompareMatrices(V_L3H_W.translational(), Vector3<double>::Zero(),
                               kTolerance, MatrixCompareType::absolute));
  EXPECT_FALSE(CompareMatrices(A_L3H_W.rotational(), Vector3<double>::Zero(),
                               kTolerance, MatrixCompareType::absolute));
  EXPECT_FALSE(CompareMatrices(A_L3H_W.translational(), Vector3<double>::Zero(),
                               kTolerance, MatrixCompareType::absolute));

  // Verify that relative spatial acceleration is the same as absolute spatial
  // acceleration if the "relative-to" frame is the "measured-in" frame.
  // First perform this test when the "measured-in" frame is the world frame W.
  // Next, perform this test when the "measured-in" frame is frame_L3.
  const SpatialAcceleration<double> A_W_WH_W =
      frame_H.CalcRelativeSpatialAccelerationInWorld(*context_, frame_W);
  const SpatialAcceleration<double> A_WH_W =
      frame_H.CalcSpatialAccelerationInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(A_W_WH_W.get_coeffs(), A_WH_W.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));

  const SpatialAcceleration<double> A_L3_L3H_W =
      frame_H.CalcRelativeSpatialAcceleration(*context_, frame_L3, frame_L3,
                                              frame_W);
  EXPECT_TRUE(CompareMatrices(A_L3_L3H_W.get_coeffs(), A_L3H_W.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));

  // Verify the Frame class's function to calculate A_W_L3H_W (frame H's spatial
  // acceleration relative to frame L3, measured and expressed in the world
  // frame W) by using the definition A_W_L3H_W ≡ A_WH_W - A_WL3_W.
  const SpatialAcceleration<double> A_W_L3H_W =
      frame_H.CalcRelativeSpatialAccelerationInWorld(*context_, frame_L3);
  const SpatialAcceleration<double> A_WL3_W =
      frame_L3.CalcSpatialAccelerationInWorld(*context_);
  const SpatialAcceleration<double> A_W_L3H_W_expected = A_WH_W - A_WL3_W;
  EXPECT_TRUE(CompareMatrices(A_W_L3H_W.get_coeffs(),
                              A_W_L3H_W_expected.get_coeffs(), kTolerance,
                              MatrixCompareType::relative));

  // Verify the Frame class's function to calculate A_W_L3H_E (frame H's spatial
  // acceleration relative to frame L3, measured in the world frame W and
  // expressed in the end-effector frame E) by using the definition
  // A_W_L3H_E = R_EW * (A_WH_W - A_WL3_W).
  const Frame<double>& frame_E = end_effector_link_->body_frame();
  const SpatialAcceleration<double> A_W_L3H_E =
      frame_H.CalcRelativeSpatialAcceleration(*context_, frame_L3, frame_W,
                                              frame_E);
  const RotationMatrix<double> R_EW =
      frame_W.CalcRotationMatrix(*context_, frame_E);
  const SpatialAcceleration<double> A_W_L3H_E_expected =
      R_EW * A_W_L3H_W_expected;
  EXPECT_TRUE(CompareMatrices(A_W_L3H_E.get_coeffs(),
                              A_W_L3H_E_expected.get_coeffs(), kTolerance,
                              MatrixCompareType::relative));

  // With the measured-in-frame being the world frame W, verify that
  // frame H's spatial acceleration relative to frame L3 is the negative of
  // frame L3's spatial acceleration relative to frame H.
  const SpatialAcceleration<double> A_W_HL3_W =
      frame_L3.CalcRelativeSpatialAccelerationInWorld(*context_, frame_H);
  EXPECT_TRUE(CompareMatrices(A_W_L3H_W.get_coeffs(), -A_W_HL3_W.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));

  // With the measured-in-frame being the end-effector frame E, verify that
  // frame H's spatial acceleration relative to frame L3 is the negative of
  // frame L3's spatial acceleration relative to frame H.
  const SpatialAcceleration<double> A_E_HL3_E =
      frame_L3.CalcRelativeSpatialAcceleration(*context_, frame_H, frame_E,
                                               frame_E);
  const SpatialAcceleration<double> A_E_L3H_E =
      frame_H.CalcRelativeSpatialAcceleration(*context_, frame_L3, frame_E,
                                              frame_E);
  EXPECT_TRUE(CompareMatrices(A_E_L3H_E.get_coeffs(), -A_E_HL3_E.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));

  // Verify the (perhaps odd) result that the rotational part of A_W_L3H_W is
  // not α_L3H_W, but instead is DtW(ω_L3H)_W (the time-derivative in frame W
  // of frame_H's angular velocity in frame L3, expressed in frame W).
  const Vector3<double>& DtW_w_L3H_W = A_W_L3H_W.rotational();
  const Vector3<double>& alpha_L3H_W = A_L3H_W.rotational();
  EXPECT_FALSE(CompareMatrices(DtW_w_L3H_W, alpha_L3H_W, kTolerance,
                               MatrixCompareType::relative));

  // Verify that the rotational part of A_W_L3H_W is:
  // DtW(ω_L3H)_W = α_L3H_W + ω_L3H_W x ω_L3H_W.
  const Vector3<double> w_L3H_W =
      frame_H.CalcAngularVelocity(*context_, frame_L3, frame_W);
  const Vector3<double> w_WL3_W =
      frame_L3.EvalAngularVelocityInWorld(*context_);
  const Vector3<double> cross_product_term = w_WL3_W.cross(w_L3H_W);
  EXPECT_TRUE(CompareMatrices(DtW_w_L3H_W, alpha_L3H_W + cross_product_term,
                              kTolerance, MatrixCompareType::relative));

  // For frame H (which is a fixed offset frame fixed to end-effector E), verify
  // that the rotational part of frame H's spatial acceleration relative to
  // frame E measured in the world frame W is zero and verify the translational
  // part is equal to α_WE x p_EoHo + ω_WE x (ω_WE x p_EoHo). This is proved by:
  // a_W_EoHo ≡ a_WHo - a_WEo, but since Eo and Ho are two points fixed to the
  // same rigid frame a_WHo = a_WEo + α_WE x p_EoHo + ω_WE x (ω_WE x p_EoHo),
  // hence a_W_EoHo ≡ a_WHo - a_WEo = α_WE x p_EoHo + ω_WE x (ω_WE x p_EoHo).
  const SpatialAcceleration<double> A_W_EH_W =
      frame_H.CalcRelativeSpatialAccelerationInWorld(*context_, frame_E);
  const Vector3<double>& a_W_EoHo_W = A_W_EH_W.translational();
  const Vector3<double>& w_WE_W = frame_E.EvalAngularVelocityInWorld(*context_);
  const Vector3<double>& p_EoHo_E = X_EH_.translation();
  const RotationMatrix<double> R_WE = R_EW.inverse();
  const Vector3<double> p_EoHo_W = R_WE * p_EoHo_E;
  const Vector3<double> wwr = w_WE_W.cross(w_WE_W.cross(p_EoHo_W));
  const Vector3<double> alpha_WE_W =
      frame_E.CalcSpatialAccelerationInWorld(*context_).rotational();
  const Vector3<double> alphar = alpha_WE_W.cross(p_EoHo_W);
  EXPECT_TRUE(CompareMatrices(a_W_EoHo_W, alphar + wwr, kTolerance,
                              MatrixCompareType::relative));

  // Verify that the calculation of separation acceleration (2ⁿᵈ time-derivative
  // of separation distance) between Ho (frame H's origin) and L3o (frame L3's
  // origin) can be calculated by point Ho's translational acceleration relative
  // to L3o, independent of the measured-in frame. The calculations below are
  // Section 10.6 Elongation (separation-speed) and elongation-rate from
  // [Mitiguy, 2017]: "Advanced Dynamics and Motion Simulation,
  //                   For professional engineers and scientists,"
  //                   Prodigy Press, Sunnyvale CA, 2017.
  //                   Available at www.MotionGenesis.com
  // Form the position vector and unit vector from L3o to Ho.
  const Vector3<double> p_L3oHo_L3 =
      frame_H.CalcPose(*context_, frame_L3).translation();
  const RotationMatrix<double> R_WL3 =
      frame_L3.CalcRotationMatrixInWorld(*context_);
  const Vector3<double> p_L3oHo_W = R_WL3 * p_L3oHo_L3;
  const double distance = p_L3oHo_W.norm();
  const Vector3<double> u_L3oHo_W = p_L3oHo_W / distance;  // unit vector.

  // Form the 2ⁿᵈ time-derivative of distance with measured-in frame_W.
  const Vector3<double>& a_W_L3H_W = A_W_L3H_W.translational();
  const Vector3<double> v_W_L3H_W =
      frame_H.CalcRelativeSpatialVelocityInWorld(*context_, frame_L3)
          .translational();
  const double separation_speed = v_W_L3H_W.dot(u_L3oHo_W);
  const double separation_speed_squared = separation_speed * separation_speed;
  const double v_W_L3H_W_magSquared = v_W_L3H_W.dot(v_W_L3H_W);
  const double separation_acceleration1 =
      a_W_L3H_W.dot(u_L3oHo_W) +
      1.0 / distance * (v_W_L3H_W_magSquared - separation_speed_squared);

  // Form the 2ⁿᵈ time-derivative of distance with measured_in frame_L3.
  const Vector3<double>& a_L3_L3H_W = A_L3_L3H_W.translational();
  const Vector3<double> v_L3_L3H_W =
      frame_H
          .CalcRelativeSpatialVelocity(*context_, frame_L3, frame_L3, frame_W)
          .translational();
  const double v_L3_L3H_W_magSquared = v_L3_L3H_W.dot(v_L3_L3H_W);
  const double separation_acceleration2 =
      a_L3_L3H_W.dot(u_L3oHo_W) +
      1.0 / distance * (v_L3_L3H_W_magSquared - separation_speed_squared);
  EXPECT_NEAR(separation_acceleration1, separation_acceleration2, kTolerance);
}

GTEST_TEST(MultibodyPlantTest, FixedWorldKinematics) {
  MultibodyPlant<double> plant(0.0);
  test::AddFixedObjectsToPlant(&plant);
  plant.Finalize();
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  // The point of this test is that we can compute poses and spatial velocities
  // even for a model with zero dofs.
  ASSERT_EQ(plant.num_positions(), 0);
  ASSERT_EQ(plant.num_velocities(), 0);
  // However the world is non-empty.
  ASSERT_NE(plant.num_bodies(), 0);

  const RigidBody<double>& mug = plant.GetBodyByName("simple_mug");

  // The objects frame O is affixed to a robot table defined by
  // test::AddFixedObjectsToPlant().
  const Frame<double>& objects_frame = plant.GetFrameByName("objects_frame");

  // This will trigger the computation of position kinematics.
  const RigidTransformd& X_WM = mug.EvalPoseInWorld(*context);

  // From test::AddFixedObjectsToPlant() we know the fixed pose of the mug frame
  // M in the objects frame O.
  const RigidTransformd X_OM = Translation3d(0.0, 0.0, 0.05);
  // Therefore we expect the pose of the mug to be:
  const RigidTransformd& X_WM_expected =
      objects_frame.CalcPoseInWorld(*context) * X_OM;

  // We verify the results.
  EXPECT_TRUE(CompareMatrices(X_WM.GetAsMatrix34(),
                              X_WM_expected.GetAsMatrix34(), kTolerance));

  // Now we evaluate some velocity kinematics.
  const SpatialVelocity<double>& V_WM =
      mug.EvalSpatialVelocityInWorld(*context);
  // Since all bodies are anchored, they all have zero spatial velocity.
  EXPECT_EQ(V_WM.get_coeffs(), Vector6<double>::Zero());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
