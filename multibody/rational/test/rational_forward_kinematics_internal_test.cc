#include "drake/multibody/rational/rational_forward_kinematics_internal.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/rational/test/rational_forward_kinematics_test_utilities.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/unit_inertia.h"

using testing::ElementsAre;

namespace drake {
namespace multibody {
namespace internal {
/**
 * Construct a MultibodyPlant with a tree topology
 * revolute -> link6
 *   |
 * world -> revolute -> link0 -> prismatic -> link1 -> revolute -> link2
 *                                              |
 * link5 <- weld <- link4 <- weld <- link3 <- revolute
 */
class KinematicTreeTest : public ::testing::Test {
 public:
  KinematicTreeTest() : plant_{new MultibodyPlant<double>{0.}} {
    AddBodyWithJoint<RevoluteJoint>("link0", plant_->world_body().index(),
                                    "joint0", Eigen::Vector3d::UnitZ());
    AddBodyWithJoint<PrismaticJoint>("link1", body_indices_[0], "joint1",
                                     Eigen::Vector3d::UnitZ());
    AddBodyWithJoint<RevoluteJoint>("link2", body_indices_[1], "joint2",
                                    Eigen::Vector3d::UnitZ());
    AddBodyWithJoint<RevoluteJoint>("link3", body_indices_[1], "joint3",
                                    Eigen::Vector3d::UnitZ());
    AddBodyWithJoint<WeldJoint>("link4", body_indices_[3], "joint4",
                                math::RigidTransformd());
    AddBodyWithJoint<WeldJoint>("link5", body_indices_[4], "joint5",
                                math::RigidTransformd());
    AddBodyWithJoint<RevoluteJoint>("link6", plant_->world_body().index(),
                                    "joint6", Eigen::Vector3d::UnitZ());
    plant_->Finalize();
  }

 protected:
  template <template <typename> class JointType, typename... Args>
  void AddBodyWithJoint(const std::string& body_name,
                        const BodyIndex parent_index,
                        const std::string& joint_name, Args&&... args) {
    const SpatialInertia<double> spatial_inertia(
        1, Eigen::Vector3d::Zero(),
        UnitInertia<double>(0.01, 0.01, 0.01, 0, 0, 0));
    body_indices_.push_back(
        plant_->AddRigidBody(body_name, spatial_inertia).index());
    plant_->AddJoint<JointType>(joint_name, plant_->get_body(parent_index),
                                std::nullopt,
                                plant_->get_body(body_indices_.back()),
                                std::nullopt, std::forward<Args>(args)...);
  }
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::vector<BodyIndex> body_indices_;
};

TEST_F(KinematicTreeTest, FindPath) {
  // Simple path to the leaf of the tree.
  EXPECT_THAT(FindPath(*plant_, world_index(), body_indices_[2]),
              ElementsAre(world_index(), body_indices_[0], body_indices_[1],
                          body_indices_[2]));
  // Path goes through the world in the middle.
  EXPECT_THAT(FindPath(*plant_, body_indices_[6], body_indices_[1]),
              ElementsAre(body_indices_[6], world_index(), body_indices_[0],
                          body_indices_[1]));
}

TEST_F(FinalizedIiwaTest, FindPath) {
  auto path = FindPath(*iiwa_, world_, iiwa_link_[7]);
  EXPECT_THAT(path, ElementsAre(world_, iiwa_link_[0], iiwa_link_[1],
                                iiwa_link_[2], iiwa_link_[3], iiwa_link_[4],
                                iiwa_link_[5], iiwa_link_[6], iiwa_link_[7]));

  // Flip the start and the end.
  path = FindPath(*iiwa_, iiwa_link_[7], world_);
  EXPECT_THAT(path, ElementsAre(iiwa_link_[7], iiwa_link_[6], iiwa_link_[5],
                                iiwa_link_[4], iiwa_link_[3], iiwa_link_[2],
                                iiwa_link_[1], iiwa_link_[0], world_));

  // start == end
  path = FindPath(*iiwa_, iiwa_link_[4], iiwa_link_[4]);
  EXPECT_THAT(path, ElementsAre(iiwa_link_[4]));

  path = FindPath(*iiwa_, iiwa_link_[3], iiwa_link_[7]);
  EXPECT_THAT(path, ElementsAre(iiwa_link_[3], iiwa_link_[4], iiwa_link_[5],
                                iiwa_link_[6], iiwa_link_[7]));

  path = FindPath(*iiwa_, iiwa_link_[3], world_);
  EXPECT_THAT(path, ElementsAre(iiwa_link_[3], iiwa_link_[2], iiwa_link_[1],
                                iiwa_link_[0], world_));
}

TEST_F(FinalizedIiwaTest, FindBodyInTheMiddleOfChain) {
  EXPECT_EQ(FindBodyInTheMiddleOfChain(*iiwa_, iiwa_link_[0], iiwa_link_[1]),
            iiwa_link_[1]);
  EXPECT_EQ(FindBodyInTheMiddleOfChain(*iiwa_, iiwa_link_[1], iiwa_link_[0]),
            iiwa_link_[0]);
  EXPECT_EQ(FindBodyInTheMiddleOfChain(*iiwa_, iiwa_link_[0], iiwa_link_[2]),
            iiwa_link_[1]);
  EXPECT_EQ(FindBodyInTheMiddleOfChain(*iiwa_, iiwa_link_[2], iiwa_link_[0]),
            iiwa_link_[1]);
  EXPECT_EQ(FindBodyInTheMiddleOfChain(*iiwa_, world_, iiwa_link_[1]),
            iiwa_link_[1]);
  EXPECT_EQ(FindBodyInTheMiddleOfChain(*iiwa_, world_, iiwa_link_[2]),
            iiwa_link_[1]);
  EXPECT_EQ(FindBodyInTheMiddleOfChain(*iiwa_, world_, iiwa_link_[3]),
            iiwa_link_[2]);
  EXPECT_EQ(FindBodyInTheMiddleOfChain(*iiwa_, world_, iiwa_link_[7]),
            iiwa_link_[4]);
  EXPECT_EQ(FindBodyInTheMiddleOfChain(*iiwa_, iiwa_link_[7], world_),
            iiwa_link_[3]);
}

TEST_F(KinematicTreeTest, FindBodyInTheMiddleOfChain) {
  // With revolute joints on the path.
  EXPECT_EQ(
      FindBodyInTheMiddleOfChain(*plant_, world_index(), body_indices_[1]),
      body_indices_[0]);
  // With revolute and prismatic joints on the path.
  EXPECT_EQ(
      FindBodyInTheMiddleOfChain(*plant_, body_indices_[6], body_indices_[2]),
      body_indices_[0]);
  // With revolute and weld joints on the path.
  EXPECT_EQ(
      FindBodyInTheMiddleOfChain(*plant_, body_indices_[5], world_index()),
      body_indices_[0]);
}

TEST_F(FinalizedIiwaTest, FindMobilizersOnPath) {
  EXPECT_THAT(FindMobilizersOnPath(*iiwa_, iiwa_link_[0], iiwa_link_[1]),
              ElementsAre(iiwa_joint_[1]));

  EXPECT_THAT(FindMobilizersOnPath(*iiwa_, iiwa_link_[1], iiwa_link_[0]),
              ElementsAre(iiwa_joint_[1]));

  EXPECT_THAT(FindMobilizersOnPath(*iiwa_, iiwa_link_[0], iiwa_link_[5]),
              ElementsAre(iiwa_joint_[1], iiwa_joint_[2], iiwa_joint_[3],
                          iiwa_joint_[4], iiwa_joint_[5]));
  EXPECT_THAT(FindMobilizersOnPath(*iiwa_, world_, iiwa_link_[3]),
              ElementsAre(iiwa_joint_[0], iiwa_joint_[1], iiwa_joint_[2],
                          iiwa_joint_[3]));

  EXPECT_THAT(FindMobilizersOnPath(*iiwa_, iiwa_link_[2], world_),
              ElementsAre(iiwa_joint_[2], iiwa_joint_[1], iiwa_joint_[0]));
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
