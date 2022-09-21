#pragma once

#include <array>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace c_iris {

std::unique_ptr<drake::multibody::MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& iiwa_sdf_name, bool finalize);

Eigen::Matrix<double, 3, 8> GenerateBoxVertices(
    const Eigen::Vector3d& size, const drake::math::RigidTransformd& pose);

std::unique_ptr<drake::multibody::MultibodyPlant<double>>
ConstructDualArmIiwaPlant(
    const std::string& iiwa_sdf_name, const drake::math::RigidTransformd& X_WL,
    const drake::math::RigidTransformd& X_WR,
    drake::multibody::ModelInstanceIndex* left_iiwa_instance,
    drake::multibody::ModelInstanceIndex* right_iiwa_instance);

class IiwaTest : public ::testing::Test {
 public:
  IiwaTest();

 protected:
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> iiwa_;
  std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_;
  const drake::multibody::internal::MultibodyTree<double>& iiwa_tree_;
  const drake::multibody::BodyIndex world_;
  std::array<drake::multibody::BodyIndex, 8> iiwa_link_;
  std::array<drake::multibody::internal::MobilizerIndex, 8> iiwa_joint_;
};

/**
 * The iiwa plant is finalized at the test construction.
 */
class FinalizedIiwaTest : public ::testing::Test {
 public:
  FinalizedIiwaTest();

 protected:
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> iiwa_;
  const drake::multibody::internal::MultibodyTree<double>& iiwa_tree_;
  const drake::multibody::BodyIndex world_;
  std::array<drake::multibody::BodyIndex, 8> iiwa_link_;
  std::array<drake::multibody::internal::MobilizerIndex, 8> iiwa_joint_;
};

/**
 * @param X_7S The transformation from schunk frame to iiwa link 7.
 * @note the plant is not finalized.
 */
void AddIiwaWithSchunk(const drake::math::RigidTransformd& X_7S,
                       drake::multibody::MultibodyPlant<double>* plant);

/**
 * @param X_WL the pose of the left IIWA base in the world frame.
 * @param X_WR the pose of the right IIWA base in the world frame.
 * @param X_7S The weld pose of Schunk gripper in IIWA link 7 frame.
 * @note `plant` is not finalized at the end of this function.
 */
void AddDualArmIiwa(const drake::math::RigidTransformd& X_WL,
                    const drake::math::RigidTransformd& X_WR,
                    const drake::math::RigidTransformd& X_7S,
                    drake::multibody::MultibodyPlant<double>* plant,
                    drake::multibody::ModelInstanceIndex* left_iiwa_instance,
                    drake::multibody::ModelInstanceIndex* right_iiwa_instance);
}  // namespace c_iris
}  // namespace multibody
}  // namespace drake
