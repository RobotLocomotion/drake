#include "drake/multibody/rational/rational_forward_kinematics_internal.h"

#include <gtest/gtest.h>

#include "drake/multibody/rational/test/rational_forward_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
TEST_F(FinalizedIiwaTest, FindPath) {
  auto path = FindPath(*iiwa_, world_, iiwa_link_[7]);
  EXPECT_EQ(path, std::vector<BodyIndex>({world_, iiwa_link_[0], iiwa_link_[1],
                                          iiwa_link_[2], iiwa_link_[3],
                                          iiwa_link_[4], iiwa_link_[5],
                                          iiwa_link_[6], iiwa_link_[7]}));

  path = FindPath(*iiwa_, iiwa_link_[7], world_);
  EXPECT_EQ(path,
            std::vector<BodyIndex>({iiwa_link_[7], iiwa_link_[6], iiwa_link_[5],
                                    iiwa_link_[4], iiwa_link_[3], iiwa_link_[2],
                                    iiwa_link_[1], iiwa_link_[0], world_}));

  path = FindPath(*iiwa_, iiwa_link_[3], iiwa_link_[7]);
  EXPECT_EQ(path,
            std::vector<BodyIndex>({iiwa_link_[3], iiwa_link_[4], iiwa_link_[5],
                                    iiwa_link_[6], iiwa_link_[7]}));

  path = FindPath(*iiwa_, iiwa_link_[3], world_);
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

TEST_F(FinalizedIiwaTest, FindMobilizersOnPath) {
  EXPECT_EQ(FindMobilizersOnPath(*iiwa_, iiwa_link_[0], iiwa_link_[1]),
            std::vector<MobilizerIndex>({iiwa_joint_[1]}));

  EXPECT_EQ(FindMobilizersOnPath(*iiwa_, iiwa_link_[1], iiwa_link_[0]),
            std::vector<MobilizerIndex>({iiwa_joint_[1]}));

  EXPECT_EQ(FindMobilizersOnPath(*iiwa_, iiwa_link_[0], iiwa_link_[5]),
            std::vector<MobilizerIndex>({iiwa_joint_[1], iiwa_joint_[2],
                                         iiwa_joint_[3], iiwa_joint_[4],
                                         iiwa_joint_[5]}));
  EXPECT_EQ(FindMobilizersOnPath(*iiwa_, world_, iiwa_link_[3]),
            std::vector<MobilizerIndex>({iiwa_joint_[0], iiwa_joint_[1],
                                         iiwa_joint_[2], iiwa_joint_[3]}));

  EXPECT_EQ(FindMobilizersOnPath(*iiwa_, iiwa_link_[2], world_),
            std::vector<MobilizerIndex>(
                {iiwa_joint_[2], iiwa_joint_[1], iiwa_joint_[0]}));
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
