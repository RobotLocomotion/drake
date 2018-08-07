#include "drake/manipulation/util/model_tree/model_tree_node.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/joints/floating_base_types.h"

using drake::nullopt;
using drake::math::RotationMatrix;
using drake::math::Transform;
using drake::multibody::joints::FloatingBaseType;

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {
namespace {

constexpr char kModelName[] = "my_model";
constexpr char kModelPath[] = "my/model/path";
constexpr ModelFileType kModelFileType = ModelFileType::kUrdf;
constexpr char kParentName[] = "my_parent";
constexpr char kParentBodyName[] = "my_predecessor_body";
constexpr FloatingBaseType kBaseJointType = FloatingBaseType::kQuaternion;

Transform<double> MakeTransform() {
  return {RotationMatrix<double>::MakeSpaceXYZRotation({1., 2., 3.}),
          {1., 2., 3}};
}

std::vector<ModelTreeNode> MakeModelTreeNodes() {
  std::vector<ModelTreeNode> nodes{};
  std::string model_name = "node_with_predecessor_info";
  PredecessorInfo predecessor_info{kModelName, "my_model_body_0", false};
  ModelFile model_file{"/path/to/" + model_name, kModelFileType};
  nodes.emplace_back(model_name, model_file, predecessor_info, MakeTransform(),
                     FloatingBaseType::kFixed, std::vector<ModelTreeNode>());
  model_name = "node_without_predecessor_info";
  nodes.emplace_back(model_name, model_file, nullopt, MakeTransform(),
                     FloatingBaseType::kFixed, std::vector<ModelTreeNode>());
  return nodes;
}

class ModelTreeNodeTest : public ::testing::Test {
 public:
  ModelTreeNodeTest()
      : model_(kModelName, ModelFile(kModelPath, kModelFileType),
               PredecessorInfo(kParentName, kParentBodyName, false),
               MakeTransform(), kBaseJointType, MakeModelTreeNodes()) {}

 protected:
  ModelTreeNode model_;
};

GTEST_TEST(ModelTreeNodeNegativeTests, DuplicateChildNamesTest) {
  std::vector<ModelTreeNode> nodes_with_duplicated_names{
      2, MakeModelTreeNodes()[0]};
  DRAKE_EXPECT_THROWS_MESSAGE(
      ModelTreeNode(kModelName, {}, {}, {}, kBaseJointType,
                    nodes_with_duplicated_names),
      std::runtime_error,
      "Duplicate child node name: " + nodes_with_duplicated_names[0].name());
}

TEST_F(ModelTreeNodeTest, ConstructorTest) {
  EXPECT_EQ(model_.name(), kModelName);
  EXPECT_EQ(model_.model_file(), ModelFile(kModelPath, kModelFileType));
  EXPECT_EQ(model_.predecessor_info(),
            PredecessorInfo(kParentName, kParentBodyName, false));
  EXPECT_TRUE(model_.X_PB().IsNearlyEqualTo(MakeTransform(), 0.0));
  EXPECT_EQ(model_.base_joint_type(), kBaseJointType);

  // Check the relationship between "free" nodes and the ones stored in
  // model_. These should not match because the ones stored in model_ have a
  // parent now.
  std::vector<ModelTreeNode> free_nodes = MakeModelTreeNodes();
  EXPECT_NE(model_.children(), free_nodes);
  ASSERT_EQ(model_.children().size(), free_nodes.size());
  ASSERT_EQ(free_nodes.size(), 2);
  for (int i = 0; i < static_cast<int>(model_.children().size()); ++i) {
    EXPECT_EQ(model_.children()[i].name(),
              model_.name() + "/" + free_nodes[i].name());
    EXPECT_EQ(model_.children()[i].model_file(), free_nodes[i].model_file());
  }

  // Check the child constructed with predecessor info.
  ASSERT_TRUE(free_nodes[0].predecessor_info());
  ASSERT_TRUE(model_.children()[0].predecessor_info());
  PredecessorInfo expected_predecessor_info = *free_nodes[0].predecessor_info();
  expected_predecessor_info.model_instance_name =
      model_.name() + "/" + expected_predecessor_info.model_instance_name;
  EXPECT_EQ(*model_.children()[0].predecessor_info(),
            expected_predecessor_info);
  EXPECT_TRUE(
      model_.children()[0].X_PB().IsNearlyEqualTo(free_nodes[0].X_PB(), 0.0));
  EXPECT_EQ(model_.children()[0].base_joint_type(),
            free_nodes[0].base_joint_type());

  // Check the child constructed without predecessor info.
  ASSERT_FALSE(free_nodes[1].predecessor_info());
  ASSERT_TRUE(model_.children()[1].predecessor_info());
  expected_predecessor_info = *model_.predecessor_info();
  EXPECT_EQ(*model_.children()[1].predecessor_info(),
            expected_predecessor_info);
  EXPECT_TRUE(model_.children()[1].X_PB().IsNearlyEqualTo(
      model_.X_PB() * free_nodes[1].X_PB(), 0.0));
  EXPECT_EQ(model_.children()[1].base_joint_type(), model_.base_joint_type());
}

TEST_F(ModelTreeNodeTest, OperatorEqualsTest) {
  // Add parent.
  // Construct comparison models.
  ModelTreeNode same_model{kModelName,
                           ModelFile(kModelPath, kModelFileType),
                           PredecessorInfo(kParentName, kParentBodyName, false),
                           MakeTransform(),
                           kBaseJointType,
                           MakeModelTreeNodes()};

  ModelTreeNode different_name{
      "my_other_model",
      ModelFile(kModelPath, kModelFileType),
      PredecessorInfo(kParentName, kParentBodyName, false),
      MakeTransform(),
      kBaseJointType,
      MakeModelTreeNodes()};

  ModelTreeNode different_model_file{
      kModelName,
      ModelFile(kModelPath, ModelFileType::kSdf),
      PredecessorInfo(kParentName, kParentBodyName, false),
      MakeTransform(),
      kBaseJointType,
      MakeModelTreeNodes()};

  ModelTreeNode different_predecessor_info{
      kModelName,
      ModelFile(kModelPath, kModelFileType),
      PredecessorInfo("foo", kParentBodyName, false),
      MakeTransform(),
      kBaseJointType,
      MakeModelTreeNodes()};

  ModelTreeNode different_X_PM{
      kModelName,
      ModelFile(kModelPath, kModelFileType),
      PredecessorInfo(kParentName, kParentBodyName, false),
      {},
      kBaseJointType,
      MakeModelTreeNodes()};

  ModelTreeNode different_base_joint_type{
      kModelName,
      ModelFile(kModelPath, kModelFileType),
      PredecessorInfo(kParentName, kParentBodyName, false),
      MakeTransform(),
      FloatingBaseType::kFixed,
      MakeModelTreeNodes()};

  ModelTreeNode different_children{
      kModelName,
      ModelFile(kModelPath, ModelFileType::kSdf),
      PredecessorInfo(kParentName, kParentBodyName, false),
      MakeTransform(),
      kBaseJointType,
      std::vector<ModelTreeNode>(1,
                                 ModelTreeNode("different_child", {}, {}, {},
                                               FloatingBaseType::kFixed, {}))};

  EXPECT_TRUE(model_ == same_model);
  EXPECT_FALSE(model_ != same_model);

  EXPECT_FALSE(model_ == different_name);
  EXPECT_TRUE(model_ != different_name);

  EXPECT_FALSE(model_ == different_model_file);
  EXPECT_TRUE(model_ != different_model_file);

  EXPECT_FALSE(model_ == different_predecessor_info);
  EXPECT_TRUE(model_ != different_predecessor_info);

  EXPECT_FALSE(model_ == different_X_PM);
  EXPECT_TRUE(model_ != different_X_PM);

  EXPECT_FALSE(model_ == different_base_joint_type);
  EXPECT_TRUE(model_ != different_base_joint_type);

  EXPECT_FALSE(model_ == different_children);
  EXPECT_TRUE(model_ != different_children);
}

}  // namespace
}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
