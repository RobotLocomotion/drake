// TODO(hongkai.dai): remove any of the helpers here that we don't end up using
// in the completed version of rational kinematics
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

/**
 * Constructs an IIWA plant with the base welded to the ground.
 */
std::unique_ptr<MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& iiwa_sdf_name, bool finalize);

/*
 * Each row of the returned matrix is a vertex of the box.
 */
Eigen::Matrix<double, 3, 8> GenerateBoxVertices(
    const Eigen::Vector3d& size, const drake::math::RigidTransformd& pose);

/*
 * Constructs a dual arm IIWA plant with the base of each IIWA welded to the
 * ground at the given pose.
 */
std::unique_ptr<MultibodyPlant<double>> ConstructDualArmIiwaPlant(
    const std::string& iiwa_sdf_name, const drake::math::RigidTransformd& X_WL,
    const drake::math::RigidTransformd& X_WR,
    ModelInstanceIndex* left_iiwa_instance,
    ModelInstanceIndex* right_iiwa_instance);

class IiwaTest : public ::testing::Test {
 public:
  IiwaTest();

 protected:
  std::unique_ptr<MultibodyPlant<double>> iiwa_;
  std::unique_ptr<geometry::SceneGraph<double>> scene_graph_;
  const internal::MultibodyTree<double>& iiwa_tree_;
  const BodyIndex world_;
  std::array<BodyIndex, 8> iiwa_link_;
  std::array<internal::MobodIndex, 8> iiwa_joint_;
};

/*
 * The iiwa plant is finalized at the test construction.
 */
class FinalizedIiwaTest : public ::testing::Test {
 public:
  FinalizedIiwaTest();

 protected:
  std::unique_ptr<MultibodyPlant<double>> iiwa_;
  const internal::MultibodyTree<double>& iiwa_tree_;
  const BodyIndex world_;
  std::array<BodyIndex, 8> iiwa_link_;
  std::array<internal::MobodIndex, 8> iiwa_joint_;
};

/*
 * @param X_7S The transformation from schunk frame to iiwa link 7.
 * @note the plant is not finalized.
 */
void AddIiwaWithSchunk(const math::RigidTransformd& X_7S,
                       MultibodyPlant<double>* plant);

}  // namespace multibody
}  // namespace drake
