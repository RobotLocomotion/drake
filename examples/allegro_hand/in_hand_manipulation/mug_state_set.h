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
class SetMugStateControl {
 public:
  SetMugStateControl();

  /// Returns the frames of the 4 fingertips' target when trying to grasp on
  /// the mug using the hand.
  /// @param finger_frames: the target frames for the IK solver to produce
  /// the initial target
  /// @param relative_finger_pose: the target frames for the differentcial IK
  /// when updating the target position, due to the motion of the mug
  void GetGraspTargetFrames(
      const Isometry3<double>& obj_frame,
      std::vector<Isometry3<double>>* finger_frames,
      std::vector<Isometry3<double>>* relative_finger_pose);

  /// Calculates the desired fingertip pose when aiming at rotate the mug
  /// along X axis (in the world frame) and the mug center for
  /// @param rotation_angle degrees. @param rotation angle is in radian
  /// The rotation is based on the saved mug position ref_mug_pose_
  void GetXRotatedTargetFrame(const double rotation_angle,
                              std::vector<Isometry3<double>>* finger_frames);
  /// Calculates the desired fingertip pose when aiming at rotate the mug
  /// along Y axis (in the world frame) and the mug center for
  /// @param rotation_angle degrees. @param rotation angle is in radian
  /// The rotation is based on the saved mug position ref_mug_pose_
  void GetYRotatedTargetFrame(const double rotation_angle,
                              std::vector<Isometry3<double>>* finger_frames);
  /// Calculates the desired fingertip pose when aiming at rotate the mug
  /// along Z axis (in the world frame) and the mug center for
  /// @param rotation_angle degrees. @param rotation angle is in radian
  /// The rotation is based on the saved mug position ref_mug_pose_
  void GetZRotatedTargetFrame(const double rotation_angle,
                              std::vector<Isometry3<double>>* finger_frames);
  /// Calculates the desired fingertip pose when aiming moving the mug along
  /// the vector @param translation_vector in the world frame.
  /// The translation is based on the saved mug position ref_mug_pose_
  void GetTransTargetFrame(const Vector3<double> translation_vector,
                           std::vector<Isometry3<double>>* finger_frames);

  void UpdateMugPose(const Isometry3<double>& mug_frame) {
    ref_mug_pose_ = mug_frame;
  }

  /// Publish the target pose of the fingers to LCM, so that the drake
  /// visualizer could display the frames. This is used only for test
  /// purpose.
  void PublishTargetFrametoLcm(const Isometry3<double>& mug_frame);

 private:
  /// frames of the selected fingertip frame on the cup
  std::vector<Isometry3<double>> contact_mug_frames_;

  /// Saved mug position, used as a reference for rotation.
  Isometry3<double> ref_mug_pose_;

  /// dimension of the mug
  const double MugHeight_ = 0.14;
  const double MugRadius_ = 0.04;
  /// parameters about the grasp points on the mug
  const double central_point_ = MugHeight_ / 2;
  const double index_finger_interval_ = 0.045;
  const double thumb_partial_ = 0.012;
};

void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<Eigen::Isometry3d>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm);

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
