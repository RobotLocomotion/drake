#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/test/kuka_iiwa_model_tests.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/frame.h"

namespace drake {

namespace multibody {

using test::KukaIiwaModelTests;

namespace {

TEST_F(KukaIiwaModelTests, FramesKinematics) {
  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  SetArbitraryConfiguration();

  const RigidTransform<double>& X_WE =
      end_effector_link_->EvalPoseInWorld(*context_);
  const RigidTransform<double> X_WH = frame_H_->CalcPoseInWorld(*context_);
  const RigidTransform<double> X_WH_expected = X_WE * X_EH_;
  EXPECT_TRUE(CompareMatrices(
      X_WH.GetAsMatrix34(), X_WH_expected.GetAsMatrix34(),
      kTolerance, MatrixCompareType::relative));

  // Alternative, we can get the pose X_WE using the plant's output port for
  // poses.
  const auto& X_WB_all =
      plant_->get_body_poses_output_port()
          .Eval<std::vector<RigidTransform<double>>>(*context_);
  ASSERT_EQ(X_WB_all.size(), plant_->num_bodies());
  const RigidTransform<double>& X_WE_from_port =
      X_WB_all[end_effector_link_->index()];
  EXPECT_TRUE(CompareMatrices(
      X_WE.GetAsMatrix34(), X_WE_from_port.GetAsMatrix34(),
      kTolerance, MatrixCompareType::relative));

  // Verify the invariance X_WB_all[0] = RigidTransform<double>::Identity().
  EXPECT_TRUE(
      CompareMatrices(X_WB_all[0].GetAsMatrix34(),
                      RigidTransform<double>::Identity().GetAsMatrix34(),
                      kTolerance, MatrixCompareType::relative));

  const RotationMatrix<double> R_WH =
      frame_H_->CalcRotationMatrixInWorld(*context_);
  const RotationMatrix<double> R_WH_expected = X_WH_expected.rotation();
  EXPECT_TRUE(CompareMatrices(R_WH.matrix(), R_WH_expected.matrix(), kTolerance,
                              MatrixCompareType::relative));

  const Body<double>& link3 = plant_->GetBodyByName("iiwa_link_3");
  const RigidTransform<double> X_HL3 =
      link3.body_frame().CalcPose(*context_, *frame_H_);
  const RigidTransform<double> X_WL3 =
      link3.body_frame().CalcPoseInWorld(*context_);
  const RigidTransform<double> X_HL3_expected = X_WH.inverse() * X_WL3;
  EXPECT_TRUE(CompareMatrices(
      X_HL3.GetAsMatrix34(), X_HL3_expected.GetAsMatrix34(),
      kTolerance, MatrixCompareType::relative));

  const RotationMatrix<double> R_HL3 =
      link3.body_frame().CalcRotationMatrix(*context_, *frame_H_);
  const RotationMatrix<double> R_WL3 =
      link3.body_frame().CalcRotationMatrixInWorld(*context_);
  const RotationMatrix<double> R_HL3_expected = R_WH.inverse() * R_WL3;
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
  EXPECT_TRUE(CompareMatrices(
      V_WH.get_coeffs(), V_WH_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));

  // Spatial velocity of link 3 measured in the H frame and expressed in the
  // end-effector frame E.
  const SpatialVelocity<double> V_HL3_E =
      link3.body_frame().CalcSpatialVelocity(
          *context_, *frame_H_, end_effector_link_->body_frame());
  // Compute V_HL3_E_expected.
  const SpatialVelocity<double> V_WH_E = R_WE.transpose() * V_WH;
  const math::RotationMatrix<double> R_EH =
      frame_H_->GetFixedPoseInBodyFrame().rotation();
  const Vector3<double> p_HL3_E = R_EH * X_HL3.translation();
  const SpatialVelocity<double> V_WL3_E =
      R_WE.transpose() * link3.EvalSpatialVelocityInWorld(*context_);
  // V_WL3_E = V_WH_E.Shift(p_HL3_E) + V_HL3_E
  const SpatialVelocity<double> V_HL3_E_expected =
      V_WL3_E - V_WH_E.Shift(p_HL3_E);
  EXPECT_TRUE(CompareMatrices(
      V_HL3_E.get_coeffs(), V_HL3_E_expected.get_coeffs(),
      kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
