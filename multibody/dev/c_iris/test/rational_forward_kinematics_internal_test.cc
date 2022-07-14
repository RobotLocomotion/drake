#include "drake/multibody/dev/c_iris/rational_forward_kinematics_internal.h"

#include <gtest/gtest.h>

#include "drake/multibody/dev/c_iris/test/rational_forward_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace c_iris {
namespace internal {
using drake::multibody::BodyIndex;
using drake::multibody::internal::Mobilizer;
using drake::multibody::internal::MobilizerIndex;
using drake::multibody::internal::MultibodyTree;

const Mobilizer<double>* GetInboardMobilizer(const MultibodyTree<double>& tree,
                                             BodyIndex body_index) {
  return &(tree.get_mobilizer(
      tree.get_topology().get_body(body_index).inboard_mobilizer));
}

// @param was_parent if this child was the parent before reshuffling. This
// determines if the mobilizer between the child and the parent was the inboard
// mobilizer of the child or the parent.
void CheckChildInReshuffledBody(const MultibodyTree<double>& tree,
                                const ReshuffledBody& reshuffled_body,
                                int child_index,
                                BodyIndex child_body_index_expected) {
  EXPECT_EQ(reshuffled_body.children[child_index]->body_index,
            child_body_index_expected);
  EXPECT_EQ(reshuffled_body.children[child_index]->parent->body_index,
            reshuffled_body.body_index);
  BodyIndex inboard_mobilizer_body;
  const BodyIndex reshuffled_body_parent =
      tree.get_topology().get_body(reshuffled_body.body_index).parent_body;
  if (reshuffled_body_parent.is_valid() &&
      reshuffled_body_parent == child_body_index_expected) {
    inboard_mobilizer_body = reshuffled_body.body_index;
  } else {
    const BodyIndex child_body_parent =
        tree.get_topology().get_body(child_body_index_expected).parent_body;
    if (child_body_parent.is_valid() &&
        child_body_parent == reshuffled_body.body_index) {
      inboard_mobilizer_body = child_body_index_expected;
    }
  }

  EXPECT_EQ(reshuffled_body.children[child_index]->mobilizer,
            GetInboardMobilizer(tree, inboard_mobilizer_body));
}

TEST_F(FinalizedIiwaTest, TestAddChildrenToReshuffledBodyWithWorldAsRoot) {
  ReshuffledBody reshuffled_world(world_, nullptr, nullptr);
  std::unordered_set<BodyIndex> visited;
  visited.emplace(world_);
  // Add children of the world.
  AddChildrenToReshuffledBody(*iiwa_, &reshuffled_world, &visited);
  EXPECT_EQ(reshuffled_world.children.size(), 1);
  CheckChildInReshuffledBody(iiwa_tree_, reshuffled_world, 0, iiwa_link_[0]);
  EXPECT_EQ(visited.size(), 2);
  EXPECT_EQ(visited, std::unordered_set<BodyIndex>({world_, iiwa_link_[0]}));

  ReshuffledBody* reshuffled_link_0 = reshuffled_world.children[0].get();

  // Continue to add children of link 0
  AddChildrenToReshuffledBody(*iiwa_, reshuffled_link_0, &visited);
  EXPECT_EQ(reshuffled_world.children.size(), 1);
  EXPECT_EQ(reshuffled_link_0->children.size(), 1);
  CheckChildInReshuffledBody(iiwa_tree_, *reshuffled_link_0, 0, iiwa_link_[1]);
  EXPECT_EQ(visited.size(), 3);
  EXPECT_EQ(visited, std::unordered_set<BodyIndex>(
                         {world_, iiwa_link_[0], iiwa_link_[1]}));

  // Continue to add children of link 1
  ReshuffledBody* reshuffled_link_1 = reshuffled_link_0->children[0].get();
  AddChildrenToReshuffledBody(*iiwa_, reshuffled_link_1, &visited);
  EXPECT_EQ(reshuffled_link_1->children.size(), 1);
  CheckChildInReshuffledBody(iiwa_tree_, *reshuffled_link_1, 0, iiwa_link_[2]);
  EXPECT_EQ(reshuffled_link_1->children[0]->body_index, iiwa_link_[2]);
  EXPECT_EQ(reshuffled_link_1->children[0]->parent, reshuffled_link_1);
  EXPECT_EQ(reshuffled_link_1->children[0]->mobilizer,
            GetInboardMobilizer(iiwa_tree_, iiwa_link_[2]));
  EXPECT_EQ(visited.size(), 4);
  EXPECT_EQ(visited,
            std::unordered_set<BodyIndex>(
                {world_, iiwa_link_[0], iiwa_link_[1], iiwa_link_[2]}));
}

TEST_F(FinalizedIiwaTest, TestAddChildrenToReshuffledBodyWithLink3AsRoot) {
  ReshuffledBody reshuffled_link_3(iiwa_link_[3], nullptr, nullptr);
  std::unordered_set<BodyIndex> visited;
  visited.insert(iiwa_link_[3]);
  AddChildrenToReshuffledBody(*iiwa_, &reshuffled_link_3, &visited);
  EXPECT_EQ(reshuffled_link_3.children.size(), 2);
  CheckChildInReshuffledBody(iiwa_tree_, reshuffled_link_3, 0, iiwa_link_[2]);
  CheckChildInReshuffledBody(iiwa_tree_, reshuffled_link_3, 1, iiwa_link_[4]);
  EXPECT_EQ(visited.size(), 3);
  EXPECT_EQ(visited, std::unordered_set<BodyIndex>(
                         {iiwa_link_[2], iiwa_link_[3], iiwa_link_[4]}));

  // Continue to add children of iiwa_link_2
  ReshuffledBody* reshuffled_link_2 = reshuffled_link_3.children[0].get();
  AddChildrenToReshuffledBody(*iiwa_, reshuffled_link_2, &visited);
  EXPECT_EQ(reshuffled_link_2->children.size(), 1);
  EXPECT_EQ(reshuffled_link_2->parent->body_index, iiwa_link_[3]);
  CheckChildInReshuffledBody(iiwa_tree_, *reshuffled_link_2, 0, iiwa_link_[1]);
  EXPECT_EQ(visited.size(), 4);
  EXPECT_EQ(visited,
            std::unordered_set<BodyIndex>(
                {iiwa_link_[1], iiwa_link_[2], iiwa_link_[3], iiwa_link_[4]}));

  // Continue to add children of iiwa_link_1
  ReshuffledBody* reshuffled_link_1 = reshuffled_link_2->children[0].get();
  AddChildrenToReshuffledBody(*iiwa_, reshuffled_link_1, &visited);
  EXPECT_EQ(reshuffled_link_1->children.size(), 1);
  CheckChildInReshuffledBody(iiwa_tree_, *reshuffled_link_1, 0, iiwa_link_[0]);
  EXPECT_EQ(visited.size(), 5);
  EXPECT_EQ(visited, std::unordered_set<BodyIndex>(
                         {iiwa_link_[0], iiwa_link_[1], iiwa_link_[2],
                          iiwa_link_[3], iiwa_link_[4]}));

  // Continue to add children of iiwa_link_0
  ReshuffledBody* reshuffled_link_0 = reshuffled_link_1->children[0].get();
  AddChildrenToReshuffledBody(*iiwa_, reshuffled_link_0, &visited);
  EXPECT_EQ(reshuffled_link_0->children.size(), 1);
  CheckChildInReshuffledBody(iiwa_tree_, *reshuffled_link_0, 0, world_);
  EXPECT_EQ(visited.size(), 6);
  EXPECT_EQ(visited, std::unordered_set<BodyIndex>(
                         {world_, iiwa_link_[0], iiwa_link_[1], iiwa_link_[2],
                          iiwa_link_[3], iiwa_link_[4]}));

  // Continue to add children of world.
  ReshuffledBody* reshuffled_world = reshuffled_link_0->children[0].get();
  AddChildrenToReshuffledBody(*iiwa_, reshuffled_world, &visited);
  EXPECT_TRUE(reshuffled_world->children.empty());
  EXPECT_EQ(visited.size(), 6);
  EXPECT_EQ(visited, std::unordered_set<BodyIndex>(
                         {world_, iiwa_link_[0], iiwa_link_[1], iiwa_link_[2],
                          iiwa_link_[3], iiwa_link_[4]}));
}

TEST_F(FinalizedIiwaTest, TestReshuffleKinematicsTreeWithWorldAsRoot) {
  ReshuffledBody reshuffled_world(world_, nullptr, nullptr);
  ReshuffleKinematicsTree(*iiwa_, &reshuffled_world);

  // Check world after reshuffling.
  EXPECT_EQ(reshuffled_world.parent, nullptr);
  EXPECT_EQ(reshuffled_world.body_index, world_);
  EXPECT_EQ(reshuffled_world.mobilizer, nullptr);

  // Check the children of world.
  EXPECT_EQ(reshuffled_world.children.size(), 1);
  CheckChildInReshuffledBody(iiwa_tree_, reshuffled_world, 0, iiwa_link_[0]);
  ReshuffledBody* reshuffled_link_0 = reshuffled_world.children[0].get();

  ReshuffledBody* reshuffled_link_i = reshuffled_link_0;
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(reshuffled_link_i->children.size(), 1);
    CheckChildInReshuffledBody(iiwa_tree_, *reshuffled_link_i, 0,
                               iiwa_link_[i + 1]);
    reshuffled_link_i = reshuffled_link_i->children[0].get();
  }

  // Now check the children of reshuffled_link_7
  EXPECT_EQ(reshuffled_link_i->children.size(), 0);
}

TEST_F(FinalizedIiwaTest, TestReshuffleKinematicsTreeWithLink3AsRoot) {
  ReshuffledBody reshuffled_link_3(iiwa_link_[3], nullptr, nullptr);
  ReshuffleKinematicsTree(*iiwa_, &reshuffled_link_3);
  EXPECT_EQ(reshuffled_link_3.body_index, iiwa_link_[3]);
  EXPECT_EQ(reshuffled_link_3.parent, nullptr);
  EXPECT_EQ(reshuffled_link_3.mobilizer, nullptr);
  EXPECT_EQ(reshuffled_link_3.children.size(), 2);
  CheckChildInReshuffledBody(iiwa_tree_, reshuffled_link_3, 0, iiwa_link_[2]);
  CheckChildInReshuffledBody(iiwa_tree_, reshuffled_link_3, 1, iiwa_link_[4]);

  ReshuffledBody* reshuffled_link_i = reshuffled_link_3.children[0].get();
  for (int i = 2; i >= 0; --i) {
    EXPECT_EQ(reshuffled_link_i->children.size(), 1);
    const BodyIndex child_index = i > 0 ? iiwa_link_[i - 1] : world_;
    CheckChildInReshuffledBody(iiwa_tree_, *reshuffled_link_i, 0, child_index);
    reshuffled_link_i = reshuffled_link_i->children[0].get();
  }
  // Now the reshuffled world.
  EXPECT_EQ(reshuffled_link_i->children.size(), 0);

  reshuffled_link_i = reshuffled_link_3.children[1].get();
  for (int i = 4; i < 7; ++i) {
    EXPECT_EQ(reshuffled_link_i->children.size(), 1);
    CheckChildInReshuffledBody(iiwa_tree_, *reshuffled_link_i, 0,
                               iiwa_link_[i + 1]);
    reshuffled_link_i = reshuffled_link_i->children[0].get();
  }
  // Now check the reshuffled_link_7.
  EXPECT_EQ(reshuffled_link_i->children.size(), 0);
}

TEST_F(FinalizedIiwaTest, FindShortestPath) {
  auto path = FindShortestPath(*iiwa_, world_, iiwa_link_[7]);
  EXPECT_EQ(path, std::vector<BodyIndex>({world_, iiwa_link_[0], iiwa_link_[1],
                                          iiwa_link_[2], iiwa_link_[3],
                                          iiwa_link_[4], iiwa_link_[5],
                                          iiwa_link_[6], iiwa_link_[7]}));

  path = FindShortestPath(*iiwa_, iiwa_link_[7], world_);
  EXPECT_EQ(path,
            std::vector<BodyIndex>({iiwa_link_[7], iiwa_link_[6], iiwa_link_[5],
                                    iiwa_link_[4], iiwa_link_[3], iiwa_link_[2],
                                    iiwa_link_[1], iiwa_link_[0], world_}));

  path = FindShortestPath(*iiwa_, iiwa_link_[3], iiwa_link_[7]);
  EXPECT_EQ(path,
            std::vector<BodyIndex>({iiwa_link_[3], iiwa_link_[4], iiwa_link_[5],
                                    iiwa_link_[6], iiwa_link_[7]}));

  path = FindShortestPath(*iiwa_, iiwa_link_[3], world_);
  EXPECT_EQ(path,
            std::vector<BodyIndex>({iiwa_link_[3], iiwa_link_[2], iiwa_link_[1],
                                    iiwa_link_[0], world_}));
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

TEST_F(FinalizedIiwaTest, FindMobilizersOnShortestPath) {
  EXPECT_EQ(FindMobilizersOnShortestPath(*iiwa_, iiwa_link_[0], iiwa_link_[1]),
            std::vector<MobilizerIndex>({iiwa_joint_[1]}));

  EXPECT_EQ(FindMobilizersOnShortestPath(*iiwa_, iiwa_link_[1], iiwa_link_[0]),
            std::vector<MobilizerIndex>({iiwa_joint_[1]}));

  EXPECT_EQ(FindMobilizersOnShortestPath(*iiwa_, iiwa_link_[0], iiwa_link_[5]),
            std::vector<MobilizerIndex>({iiwa_joint_[1], iiwa_joint_[2],
                                         iiwa_joint_[3], iiwa_joint_[4],
                                         iiwa_joint_[5]}));
  EXPECT_EQ(FindMobilizersOnShortestPath(*iiwa_, world_, iiwa_link_[3]),
            std::vector<MobilizerIndex>({iiwa_joint_[0], iiwa_joint_[1],
                                         iiwa_joint_[2], iiwa_joint_[3]}));

  EXPECT_EQ(FindMobilizersOnShortestPath(*iiwa_, iiwa_link_[2], world_),
            std::vector<MobilizerIndex>(
                {iiwa_joint_[2], iiwa_joint_[1], iiwa_joint_[0]}));
}

}  // namespace internal
}  // namespace c_iris
}  // namespace multibody
}  // namespace drake
