#pragma once

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace allegro_hand {

/// The class combines a set of methods to calculate target positions of the
/// Allegro hand's fingertips, so that it would matches the desired pose of the
/// mug object. By following the target positions, the hand could move the mug
/// to the corresponding pose. The class is only used for run_rotating_mug.cc.
class MugStateSet {
 public:
  MugStateSet();

  /// Returns the poses of the 4 fingertips' target when trying to grasp on
  /// the mug using the hand.
  /// @param X_WO: the pose of the object frame measured in the world frame
  /// @param X_WF_target: the target frames for the IK solver to produce
  /// the initial target
  /// @param X_WF_target_ForDiffIK: the target frames for the differentcial IK
  /// when updating the target position, due to the motion of the mug
  void CalcFingerPoseForGrasp(
      const Isometry3<double>& X_WO,
      std::vector<Isometry3<double>>* X_WF_target,
      std::vector<Isometry3<double>>* X_WF_target_ForDiffIK) const;

  /// Calculates the desired fingertip pose when aiming at rotating the mug
  /// along X axis (in the world frame) and the mug center for
  /// @param rotation_angle_rad.
  /// The rotation is based on the saved mug position X_WO_ref_
  void CalcFingerPoseWithMugXRotation(
      double rotation_angle_rad,
      std::vector<Isometry3<double>>* X_WF_target) const;
  void CalcFingerPoseWithMugXRotation(
      double rotation_angle_rad, std::vector<Isometry3<double>>* X_WF_target,
      const Isometry3<double>& X_WO) {
    UpdateReferenceMugPose(X_WO);
    CalcFingerPoseWithMugXRotation(rotation_angle_rad, X_WF_target);
  }

  /// Calculates the desired fingertip pose when aiming at rotating the mug
  /// along Y axis (in the world frame) and the mug center for
  /// @param rotation_angle_rad.
  /// The rotation is based on the saved mug position X_WO_ref_
  void CalcFingerPoseWithMugYRotation(
      double rotation_angle_rad,
      std::vector<Isometry3<double>>* X_WF_target) const;
  void CalcFingerPoseWithMugYRotation(
      double rotation_angle_rad, std::vector<Isometry3<double>>* X_WF_target,
      const Isometry3<double>& X_WO) {
    UpdateReferenceMugPose(X_WO);
    CalcFingerPoseWithMugYRotation(rotation_angle_rad, X_WF_target);
  }

  /// Calculates the desired fingertip pose when aiming at rotating the mug
  /// along Z axis (in the world frame) and the mug center for
  /// @param rotation_angle_rad.
  /// The rotation is based on the saved mug position X_WO_ref_
  void CalcFingerPoseWithMugZRotation(
      double rotation_angle_rad,
      std::vector<Isometry3<double>>* X_WF_target) const;
  void CalcFingerPoseWithMugZRotation(
      double rotation_angle_rad, std::vector<Isometry3<double>>* X_WF_target,
      const Isometry3<double>& X_WO) {
    UpdateReferenceMugPose(X_WO);
    CalcFingerPoseWithMugZRotation(rotation_angle_rad, X_WF_target);
  }

  /// Calculates the desired fingertip pose when aiming at moving the mug along
  /// @param v_W is the target translation of the mug in the world frame.
  /// The translation is based on the saved mug position X_WO_ref_
  void CalcFingerPoseWithMugTranslation(
      const Vector3<double>& v_W,
      std::vector<Isometry3<double>>* X_WF_target) const;
  void CalcFingerPoseWithMugTranslation(
      const Vector3<double>& v_W, std::vector<Isometry3<double>>* X_WF_target,
      const Isometry3<double>& X_WO) {
    UpdateReferenceMugPose(X_WO);
    CalcFingerPoseWithMugTranslation(v_W, X_WF_target);
  }

  void UpdateReferenceMugPose(const Isometry3<double>& X_WO) {
    X_WO_ref_ = X_WO;
  }

  /// Publish the target pose of the fingers to LCM, so that the drake
  /// visualizer could display the frames. This is used only for test
  /// purpose.
  /// @param X_WO the pose of the mug measured in the world frame
  void PublishTargetFingerPoseToLcm(const Isometry3<double>& X_WO) const;

 private:
  /// frames of the selected fingertip frame on the cup. F represents the
  /// fingertip frames
  std::vector<Isometry3<double>> X_OF_contact_;

  /// Saved mug position, used as a reference for rotation.
  Isometry3<double> X_WO_ref_;

  /// dimension of the mug
  const double MugHeight_ = 0.14;
  const double MugRadius_ = 0.04;
  /// parameters about the grasp points on the mug, along the Z direction,
  /// which corresponds to the height direction of the mug. In the SDF file,
  /// the origin of the mug frame is defined at the bottom of the mug
  const double central_point_ = MugHeight_ / 2;
  const double p_OIndex_z_ = 0.045;
  /// desired position of the thumb on the mug frame, deviated from the center
  /// point on the height direction
  const double p_OThumb_z_ = 0.012;
};

void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<Eigen::Isometry3d>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm);

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
