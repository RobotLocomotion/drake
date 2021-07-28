#include "drake/examples/planar_gripper/planar_gripper_common.h"

#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace drake {
namespace examples {
namespace planar_gripper {

using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::MultibodyPlant;
using Eigen::Vector3d;

template <typename T>
void WeldGripperFrames(MultibodyPlant<T>* plant) {
  // The finger base links are all welded a fixed distance from the gripper
  // frame's origin (Go), lying on the gripper frame's Y-Z plane. We denote the
  // gripper frame's Y and Z axes as Gy and Gz.
  const double kGripperOriginToBaseDistance = 0.201;
  const double kFinger1Angle = M_PI / 3.0;
  const double kFinger2Angle = -M_PI / 3.0;
  const double kFinger3Angle = M_PI;

  // Note: Before welding and with all finger joint angles being zero, all
  // finger base links sit at the world origin with the finger pointing along
  // the world -Z axis.

  // We align the planar gripper coordinate frame G with the world frame W.
  const RigidTransformd X_WG = X_WGripper();

  // Weld the first finger. Finger base links are arranged equidistant along the
  // perimeter of a circle. The first finger is welded kFinger1Angle radians
  // from the +Gz-axis. Frames F1, F2, F3 correspond to the base link finger
  // frames.
  RigidTransformd X_GF1 =
      RigidTransformd(Eigen::AngleAxisd(kFinger1Angle, Vector3d::UnitX()),
                      Vector3d(0, 0, 0)) *
      RigidTransformd(math::RotationMatrixd(),
                      Vector3d(0, 0, kGripperOriginToBaseDistance));
  const multibody::Frame<T>& finger1_base_frame =
      plant->GetFrameByName("finger1_base");
  plant->WeldFrames(plant->world_frame(), finger1_base_frame, X_WG * X_GF1);

  // Weld the second finger. The second finger is welded kFinger2Angle radians
  // from the +Gz-axis.
  RigidTransformd X_GF2 =
      RigidTransformd(Eigen::AngleAxisd(kFinger2Angle, Vector3d::UnitX()),
                      Vector3d(0, 0, 0)) *
      RigidTransformd(math::RotationMatrixd(),
                      Vector3d(0, 0, kGripperOriginToBaseDistance));
  const multibody::Frame<T>& finger2_base_frame =
      plant->GetFrameByName("finger2_base");
  plant->WeldFrames(plant->world_frame(), finger2_base_frame, X_WG * X_GF2);

  // Weld the 3rd finger. The third finger is welded kFinger3Angle radians from
  // the +Gz-axis.
  RigidTransformd X_GF3 =
      RigidTransformd(Eigen::AngleAxisd(kFinger3Angle, Vector3d::UnitX()),
                      Vector3d(0, 0, 0)) *
      RigidTransformd(math::RotationMatrixd(),
                      Vector3d(0, 0, kGripperOriginToBaseDistance));
  const multibody::Frame<T>& finger3_base_frame =
      plant->GetFrameByName("finger3_base");
  plant->WeldFrames(plant->world_frame(), finger3_base_frame, X_WG * X_GF3);
}

// Explicit instantiations.
template void WeldGripperFrames(MultibodyPlant<double>* plant);

/// Build a keyframe matrix for joints in joint_ordering by extracting the
/// appropriate columns from all_keyframes. The interpretation of columns in
/// joint_keyframes are ordered as in joint_ordering.
/// @pre There are as many strings in headers as there are columns in
/// all_keyframes.
/// @pre Every string in joint_ordering is expected to be unique.
/// @pre Every string in joint_ordering is expected to be found in headers.
MatrixX<double> MakeKeyframes(MatrixX<double> all_keyframes,
                              std::vector<std::string> joint_ordering,
                              std::vector<std::string> headers) {
  // First find the columns in the keyframe data for just the joints in
  // joint_ordering.
  std::map<std::string, int> joint_name_to_col_index_map;
  for (const auto& header_name : joint_ordering) {
    auto match_it = std::find(headers.begin(), headers.end(), header_name);
    DRAKE_DEMAND(match_it != headers.end());
    joint_name_to_col_index_map[header_name] = match_it - headers.begin();
  }
  // Now create the keyframe matrix.
  const int keyframe_count = all_keyframes.rows();
  const int kNumFingerJoints = joint_ordering.size();
  MatrixX<double> joint_keyframes(keyframe_count, kNumFingerJoints);
  for (int i = 0; i < kNumFingerJoints; ++i) {
    const std::string& joint_name = joint_ordering[i];
    const int all_keyframe_col_index = joint_name_to_col_index_map[joint_name];
    joint_keyframes.block(0, i, keyframe_count, 1) =
        all_keyframes.block(0, all_keyframe_col_index, keyframe_count, 1);
  }
  return joint_keyframes;
}

std::pair<MatrixX<double>, std::map<std::string, int>> ParseKeyframes(
    const std::string& name, EigenPtr<Vector3<double>> brick_initial_pose) {
  const std::string keyframe_path = FindResourceOrThrow(name);
  std::fstream file;
  file.open(keyframe_path, std::fstream::in);
  DRAKE_DEMAND(file.is_open());

  // Count the number of lines in the file.
  std::string line;
  int line_count = 0;
  while (!std::getline(file, line).eof()) {
    line_count++;
  }
  const int keyframe_count = line_count - 1;
  drake::log()->info("Found {} keyframes", keyframe_count);

  // Get the file headers.
  file.clear();
  file.seekg(0);
  std::getline(file, line);
  std::stringstream sstream(line);
  std::vector<std::string> headers;
  std::string token;
  while (sstream >> token) {
    headers.push_back(token);
  }

  // Make sure we read the correct number of headers.
  const int kNumHeaders = 9;
  if (headers.size() != kNumHeaders) {
    throw std::runtime_error(
        "Unexpected number of headers found in keyframe input file.");
  }

  // Extract all keyframes (finger and brick)
  MatrixX<double> all_keyframes(keyframe_count, headers.size());
  for (int i = 0; i < all_keyframes.rows(); ++i) {
    for (int j = 0; j < all_keyframes.cols(); ++j) {
      file >> all_keyframes(i, j);
    }
  }

  // Find the columns in the keyframe data for just the finger joints and
  // create the corresponding keyframe matrix.
  std::vector<std::string> finger_joint_ordering = {
      "finger1_BaseJoint", "finger2_BaseJoint", "finger3_BaseJoint",
      "finger1_MidJoint",  "finger2_MidJoint",  "finger3_MidJoint"};
  MatrixX<double> finger_joint_keyframes =
      MakeKeyframes(all_keyframes, finger_joint_ordering, headers);

  // Find the columns in the keyframe data for just the brick joints and create
  // the corresponding keyframe matrix. Note: Only the first keyframe is used to
  // set the brick's initial position. All other brick keyframe data is unused.
  std::vector<std::string> brick_joint_ordering = {"brick_translate_y_joint",
                                                   "brick_translate_z_joint",
                                                   "brick_revolute_x_joint"};
  std::map<std::string, int> brick_joint_name_to_col_index_map;
  MatrixX<double> brick_joint_keyframes =
      MakeKeyframes(all_keyframes, brick_joint_ordering, headers);

  // Set the brick's initial pose (expressed in the gripper frame G).
  if (brick_initial_pose != EigenPtr<Vector3<double>>(nullptr)) {
    (*brick_initial_pose)(0) = brick_joint_keyframes(0, 0);  // y-translate
    (*brick_initial_pose)(1) = brick_joint_keyframes(0, 1);  // z-translate
    (*brick_initial_pose)(2) = brick_joint_keyframes(0, 2);  // x-revolute
  }

  finger_joint_keyframes.transposeInPlace();

  // Create the finger joint name to row index map.
  std::map<std::string, int> finger_joint_name_to_row_index_map;
  for (size_t i = 0; i < finger_joint_ordering.size(); i++) {
    finger_joint_name_to_row_index_map[finger_joint_ordering[i]] = i;
  }

  return std::make_pair(finger_joint_keyframes,
                        finger_joint_name_to_row_index_map);
}

MatrixX<double> ReorderKeyframesForPlant(
    const MultibodyPlant<double>& plant, const MatrixX<double> keyframes,
    std::map<std::string, int>* finger_joint_name_to_row_index_map) {
  DRAKE_DEMAND(finger_joint_name_to_row_index_map != nullptr);
  if (static_cast<int>(finger_joint_name_to_row_index_map->size()) !=
      keyframes.rows()) {
    throw std::runtime_error(
        "The number of keyframe rows must match the size of "
        "finger_joint_name_to_row_index_map.");
  }
  if (keyframes.rows() != kNumJoints) {
    throw std::runtime_error(
        "The number of keyframe rows must match the number of planar-gripper "
        "joints");
  }
  if (plant.num_positions() != kNumJoints) {
    throw std::runtime_error(
        "The number of plant positions must exactly match the number of "
        "planar-gripper joints.");
  }
  std::map<std::string, int> original_map = *finger_joint_name_to_row_index_map;
  MatrixX<double> reordered_keyframes(keyframes);
  for (auto iter = original_map.begin(); iter != original_map.end(); ++iter) {
    auto joint_vel_start_index =
        plant.GetJointByName(iter->first).velocity_start();
    reordered_keyframes.row(joint_vel_start_index) =
        keyframes.row(iter->second);
    (*finger_joint_name_to_row_index_map)[iter->first] = joint_vel_start_index;
  }
  return reordered_keyframes;
}

const RigidTransformd X_WGripper() {
  return math::RigidTransformd::Identity();
}

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
