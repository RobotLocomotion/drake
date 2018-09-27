#include "drake/examples/allegro_hand/in_hand_manipulation/mug_state_set.h"
#include "drake/lcmt_viewer_draw.hpp"

namespace drake {
namespace examples {
namespace allegro_hand {

MugStateSet::MugStateSet() {
  // Ini the target fingertip positions on the mug. F is the fingertip frame.
  // Each col corresponds to a finger, in the order of thumb-index-middle-ring
  Eigen::Matrix<double, 3, 4> p_OF;
  p_OF.col(2) << 0, MugRadius_, central_point_;
  p_OF.col(1) << 0, MugRadius_, central_point_ - index_finger_interval_;
  p_OF.col(3) << 0, MugRadius_, central_point_ + index_finger_interval_;
  p_OF.col(0) << 0, -MugRadius_, central_point_ - thumb_partial_;
  Eigen::Vector4d TargetRotAngle(M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2);

  Eigen::Isometry3d X_OF;
  X_OF.matrix().setIdentity();
  for (int i = 0; i < 4; i++) {
    X_OF.translation() = p_OF.col(i);
    X_OF.linear() =
        math::RotationMatrix<double>(math::RollPitchYaw<double>(Eigen::Vector3d(
                                         TargetRotAngle(i), 0, 0)))
            .matrix();
    X_OF_contact_.push_back(X_OF);
  }
}

void MugStateSet::GetGraspTargetFrames(
    const Isometry3<double>& X_WO, std::vector<Isometry3<double>>* frame_poses,
    std::vector<Isometry3<double>>* relative_finger_pose) const {
  if (frame_poses->size() < 4)
    *frame_poses = std::vector<drake::Isometry3<double>>(4);
  if (relative_finger_pose->size() < 4)
    *relative_finger_pose = std::vector<drake::Isometry3<double>>(4);

  // setting the target position of the fingertips to be a position in the
  // minus Z direction, so that the fingers can exerting some force on the mug
  // after reaching the target surface.
  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.007);

  for (int i = 0; i < 4; i++) {
    (*relative_finger_pose)[i] = X_OF_contact_[i] * grasp_offset;
    (*frame_poses)[i] = X_WO * X_OF_contact_[i] * grasp_offset;
  }
  // For the initial IK calculation, the target of the thumb is set to be lower
  // than the actual target, so that to prevent collision with the mug.
  grasp_offset.translation() = Eigen::Vector3d(0, 0, 0.017);
  (*frame_poses)[0] = (*frame_poses)[0] * grasp_offset;
}

void MugStateSet::CalcFingerPoseWithMugXRotation(
    double rotation_angle_rad,
    std::vector<Isometry3<double>>* frame_poses) const {
  if (frame_poses->size() < 4)
    *frame_poses = std::vector<drake::Isometry3<double>>(4);

  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.002);

  Isometry3<double> temp;
  temp.matrix().setIdentity();
  Isometry3<double> tar_mug_frame;
  tar_mug_frame.matrix().setIdentity();
  tar_mug_frame.translation() << 0, 0, -MugHeight_ * 0.5;
  temp.rotate(
      Eigen::AngleAxis<double>(rotation_angle_rad, Eigen::Vector3d::UnitX()));
  tar_mug_frame = temp * tar_mug_frame;
  temp.matrix().setIdentity();
  temp.translation() << 0, 0, MugHeight_ * 0.5;
  tar_mug_frame = temp * tar_mug_frame;
  tar_mug_frame = X_WO_ref_ * tar_mug_frame;

  for (int i = 0; i < 4; i++) {
    (*frame_poses)[i] = tar_mug_frame * X_OF_contact_[i] * grasp_offset;
  }
}

void MugStateSet::CalcFingerPoseWithMugYRotation(
    double rotation_angle_rad,
    std::vector<Isometry3<double>>* frame_poses) const {
  if (frame_poses->size() < 4)
    *frame_poses = std::vector<drake::Isometry3<double>>(4);

  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.002);

  Isometry3<double> temp;
  temp.matrix().setIdentity();
  Isometry3<double> tar_mug_frame;
  tar_mug_frame.matrix().setIdentity();
  tar_mug_frame.translation() << 0, 0, -MugHeight_ * 0.5;
  temp.rotate(
      Eigen::AngleAxis<double>(rotation_angle_rad, Eigen::Vector3d::UnitY()));
  tar_mug_frame = temp * tar_mug_frame;
  temp.matrix().setIdentity();
  temp.translation() << 0, 0, MugHeight_ * 0.5;
  tar_mug_frame = temp * tar_mug_frame;
  tar_mug_frame = X_WO_ref_ * tar_mug_frame;

  for (int i = 0; i < 4; i++) {
    (*frame_poses)[i] = tar_mug_frame * X_OF_contact_[i] * grasp_offset;
  }
}

void MugStateSet::CalcFingerPoseWithMugZRotation(
    double rotation_angle_rad,
    std::vector<Isometry3<double>>* frame_poses) const {
  if (frame_poses->size() < 4)
    *frame_poses = std::vector<drake::Isometry3<double>>(4);

  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.002);

  Isometry3<double> tar_mug_frame;
  tar_mug_frame.matrix().setIdentity();
  tar_mug_frame.rotate(
      Eigen::AngleAxis<double>(rotation_angle_rad, Eigen::Vector3d::UnitZ()));
  tar_mug_frame = X_WO_ref_ * tar_mug_frame;

  for (int i = 0; i < 4; i++) {
    (*frame_poses)[i] = tar_mug_frame * X_OF_contact_[i] * grasp_offset;
  }
}

void MugStateSet::CalcFingerPoseWithMugTranslation(
    const Vector3<double>& v_W,
    std::vector<Isometry3<double>>* frame_poses) const {
  Isometry3<double> tar_mug_frame = X_WO_ref_;
  tar_mug_frame.translation() += v_W;

  Isometry3<double> grasp_offset;
  grasp_offset.matrix().setIdentity();
  grasp_offset.translation() = Eigen::Vector3d(0, 0, -0.001);
  for (int i = 0; i < 4; i++) {
    (*frame_poses)[i] = tar_mug_frame * X_OF_contact_[i] * grasp_offset;
  }
}

void MugStateSet::PublishTargetFingerPoseToLcm(
    const Isometry3<double>& X_WO) const {
  lcm::DrakeLcm lcm;
  std::vector<std::string> frame_names;
  std::vector<Isometry3<double>> frame_poses;
  for (int i = 0; i < 4; i++) {
    frame_names.push_back("FingerTargetFrame" + std::to_string(i));
    frame_poses.push_back(X_WO * X_OF_contact_[i]);
  }
  PublishFramesToLcm("FingerTargetFrame", frame_poses, frame_names, &lcm);
}

void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<Eigen::Isometry3d>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm) {
  DRAKE_DEMAND(poses.size() == names.size());
  lcmt_viewer_draw frame_msg{};
  frame_msg.timestamp = 0;
  int32_t vsize = poses.size();
  frame_msg.num_links = vsize;
  frame_msg.link_name.resize(vsize);
  frame_msg.robot_num.resize(vsize, 0);

  for (size_t i = 0; i < poses.size(); i++) {
    Eigen::Isometry3f pose = poses[i].cast<float>();
    // Create a frame publisher
    Eigen::Vector3f goal_pos = pose.translation();
    Eigen::Quaternion<float> goal_quat =
        Eigen::Quaternion<float>(pose.linear());
    frame_msg.link_name[i] = names[i];
    frame_msg.position.push_back({goal_pos(0), goal_pos(1), goal_pos(2)});
    frame_msg.quaternion.push_back(
        {goal_quat.w(), goal_quat.x(), goal_quat.y(), goal_quat.z()});
  }
  const int num_bytes = frame_msg.getEncodedSize();
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  frame_msg.encode(bytes.data(), 0, num_bytes);
  dlcm->Publish("DRAKE_DRAW_FRAMES_" + channel_name, bytes.data(), num_bytes,
                {});
}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
