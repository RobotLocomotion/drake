#pragma once

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace drake {
namespace examples {
namespace planar_gripper {

using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::multibody::MultibodyPlant;

template <typename T>
void WeldGripperFrames(multibody::MultibodyPlant<T>* plant) {
  const double outer_radius = 0.19;
  const double f1_angle = M_PI / 3;
  const math::RigidTransformd XT(math::RollPitchYaw<double>(0, 0, 0),
                                 Eigen::Vector3d(0, 0, outer_radius));

  // Weld the first finger.
  math::RigidTransformd X_PC1(math::RollPitchYaw<double>(f1_angle, 0, 0),
                              Eigen::Vector3d::Zero());
  X_PC1 = X_PC1 * XT;
  const multibody::Frame<T>& finger1_base_frame =
      plant->GetFrameByName("finger1_base");
  plant->WeldFrames(plant->world_frame(), finger1_base_frame, X_PC1);

  // Weld the second finger.
  const math::RigidTransformd X_PC2 =
      math::RigidTransformd(math::RollPitchYawd(M_PI / 3 * 2, 0, 0),
                            Eigen::Vector3d::Zero()) *
      X_PC1;
  const multibody::Frame<T>& finger2_base_frame =
      plant->GetFrameByName("finger2_base");
  plant->WeldFrames(plant->world_frame(), finger2_base_frame, X_PC2);

  // Weld the 3rd finger.
  const math::RigidTransformd X_PC3 =
      math::RigidTransformd(math::RollPitchYawd(M_PI / 3 * 2, 0, 0),
                            Eigen::Vector3d::Zero()) *
      X_PC2;
  const multibody::Frame<T>& finger3_base_frame =
      plant->GetFrameByName("finger3_base");
  plant->WeldFrames(plant->world_frame(), finger3_base_frame, X_PC3);
}

void PublishRobotPlan(const robotlocomotion::robot_plan_t& plan) {
  // Publish the plan for inspection
  drake::systems::lcm::LcmInterfaceSystem lcm;
  const int num_bytes = plan.getEncodedSize();
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  plan.encode(bytes.data(), 0, num_bytes);
  lcm.Publish("ROBOT_PLAN", bytes.data(), num_bytes, {});
}

MatrixX<double> ParseKeyframes(const std::string& name,
                               EigenPtr<Vector3<double>> brick_ics) {
  const std::string keyframe_path = FindResourceOrThrow(name);
  std::fstream fs;
  fs.open(keyframe_path, std::fstream::in);
  DRAKE_DEMAND(fs.is_open());

  // Count the number of keyframes.
  std::string line;
  int line_count = 0;
  while (std::getline(fs, line)) {
    if (line.size() == 0) break;
    line_count++;
  }
  const int keyframe_count = line_count - 1;
  drake::log()->info("Found {} keyframes", keyframe_count);

  // Get the file headers.
  fs.seekg(0);
  std::getline(fs, line);
  std::stringstream sstream(line);
  std::vector<std::string> headers;
  std::string token;
  while (sstream >> token) {
    headers.push_back(token);
  }

  // Extract all keyframes (finger and brick)
  MatrixX<double> all_keyframes(keyframe_count, headers.size());
  for (int i = 0; i < all_keyframes.rows(); ++i) {
    for (int j = 0; j < all_keyframes.cols(); ++j) {
      fs >> all_keyframes(i, j);
    }
  }

  // Find the columns for just the finger joints.
  std::vector<std::string> finger_joint_name_ordering = {
      "finger1_ShoulderJoint", "finger2_ShoulderJoint", "finger3_ShoulderJoint",
      "finger1_ElbowJoint",    "finger2_ElbowJoint",    "finger3_ElbowJoint"};
  std::map<std::string, int> finger_joint_name_to_col_index_map;
  for (auto it = finger_joint_name_ordering.begin();
       it != finger_joint_name_ordering.end(); it++) {
    auto name_it = std::find(headers.begin(), headers.end(), *it);
    DRAKE_DEMAND(name_it != headers.end());
    finger_joint_name_to_col_index_map[*it] = name_it - headers.begin();
  }

  // Create the matrix for just the finger joint keyframes.
  const int kNumFingerJoints = 6;
  MatrixX<double> finger_joint_keyframes(keyframe_count, kNumFingerJoints);
  for (auto it = finger_joint_name_ordering.begin();
       it != finger_joint_name_ordering.end(); ++it) {
    // Create the indexes.
    int all_keyframe_col_index = finger_joint_name_to_col_index_map[*it];
    auto joint_only_col_index = it - finger_joint_name_ordering.begin();

    finger_joint_keyframes.block(0, joint_only_col_index, keyframe_count, 1) =
        all_keyframes.block(0, all_keyframe_col_index, keyframe_count, 1);
  }

  // Create a matrix for the just the brick's keyframes;
  std::vector<std::string> brick_joint_name_ordering = {
      "brick_translate_y_joint", "brick_translate_z_joint",
      "brick_revolute_x_joint"};
  std::map<std::string, int> brick_joint_name_to_col_index_map;
  for (auto it = brick_joint_name_ordering.begin();
       it != brick_joint_name_ordering.end(); it++) {
    auto name_it = std::find(headers.begin(), headers.end(), *it);
    DRAKE_DEMAND(name_it != headers.end());
    brick_joint_name_to_col_index_map[*it] = name_it - headers.begin();
  }

  const int kNumBrickJoints = 3;
  MatrixX<double> brick_joint_keyframes(keyframe_count, kNumBrickJoints);
  for (auto it = brick_joint_name_ordering.begin();
       it != brick_joint_name_ordering.end(); ++it) {
    // Create the indexes.
    int all_keyframe_col_index = brick_joint_name_to_col_index_map[*it];
    auto joint_only_col_index = it - brick_joint_name_ordering.begin();

    brick_joint_keyframes.block(0, joint_only_col_index, keyframe_count, 1) =
        all_keyframes.block(0, all_keyframe_col_index, keyframe_count, 1);
  }

  // Find the brick's initial conditions.
  (*brick_ics)(0) = brick_joint_keyframes(0, 0);  // y-translate
  (*brick_ics)(1) = brick_joint_keyframes(0, 1);  // z-translate
  (*brick_ics)(2) = brick_joint_keyframes(0, 2);  // x-revolute

  return finger_joint_keyframes;
}

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
