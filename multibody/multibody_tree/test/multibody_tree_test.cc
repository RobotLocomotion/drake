#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <functional>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"

namespace drake {
namespace multibody {
namespace multibody_model {
namespace {

using multibody::benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel;

// This test creates a model for a KUKA Iiiwa arm and verifies we can retrieve
// multibody elements by name or get exceptions accordingly.
GTEST_TEST(MultibodyTree, RetrieveNamedElements) {
  const std::string kInvalidName = "InvalidName";
  const std::vector<std::string> kLinkNames = {
      "iiwa_link_1",
      "iiwa_link_2",
      "iiwa_link_3",
      "iiwa_link_4",
      "iiwa_link_5",
      "iiwa_link_6",
      "iiwa_link_7"};
  const std::vector<std::string> kJointNames = {
      "iiwa_joint_1",
      "iiwa_joint_2",
      "iiwa_joint_3",
      "iiwa_joint_4",
      "iiwa_joint_5",
      "iiwa_joint_6",
      "iiwa_joint_7"};

  std::unique_ptr<MultibodyTree<double>> model =
      MakeKukaIiwaModel<double>(9.81);

  // MakeKukaIiwaModel() has already called Finalize() on the new kuka model.
  // Therefore its topology is valid.
  EXPECT_TRUE(model->topology_is_valid());

  // Another call to Finalize() is not allowed.
  EXPECT_THROW(model->Finalize(), std::logic_error);

  // Model Size. Counting the world body, there should be three bodies.
  EXPECT_EQ(model->get_num_bodies(), 8);  // It includes the "world" body.
  EXPECT_EQ(model->get_num_joints(), 7);

  // State size.
  EXPECT_EQ(model->get_num_positions(), 7);
  EXPECT_EQ(model->get_num_velocities(), 7);
  EXPECT_EQ(model->get_num_states(), 14);

  // Query if elements exist in the model.
  EXPECT_TRUE(model->HasBodyNamed("iiwa_link_1"));
  EXPECT_TRUE(model->HasBodyNamed("iiwa_link_2"));
  EXPECT_TRUE(model->HasBodyNamed("iiwa_link_3"));
  EXPECT_TRUE(model->HasBodyNamed("iiwa_link_4"));
  EXPECT_TRUE(model->HasBodyNamed("iiwa_link_5"));
  EXPECT_TRUE(model->HasBodyNamed("iiwa_link_6"));
  EXPECT_TRUE(model->HasBodyNamed("iiwa_link_7"));
  EXPECT_FALSE(model->HasBodyNamed(kInvalidName));

  EXPECT_TRUE(model->HasJointNamed("iiwa_joint_1"));
  EXPECT_TRUE(model->HasJointNamed("iiwa_joint_2"));
  EXPECT_TRUE(model->HasJointNamed("iiwa_joint_3"));
  EXPECT_TRUE(model->HasJointNamed("iiwa_joint_4"));
  EXPECT_TRUE(model->HasJointNamed("iiwa_joint_5"));
  EXPECT_TRUE(model->HasJointNamed("iiwa_joint_6"));
  EXPECT_TRUE(model->HasJointNamed("iiwa_joint_7"));
  EXPECT_FALSE(model->HasJointNamed(kInvalidName));

  // Get links by name.
  for (const std::string link_name : kLinkNames) {
    const Body<double>& link = model->GetBodyByName(link_name);
    EXPECT_EQ(link.get_name(), link_name);
  }

  // Attempting to retrieve a link that is not part of the model should throw
  // an exception.
  EXPECT_THROW(model->GetBodyByName(kInvalidName), std::logic_error);

  // Get joints by name.
  for (const std::string joint_name : kJointNames) {
    const Joint<double>& joint = model->GetJointByName(joint_name);
    EXPECT_EQ(joint.get_name(), joint_name);
  }
  EXPECT_THROW(model->GetJointByName(kInvalidName), std::logic_error);

  // Templatized version to obtain retrieve a particular known type of joint.
  for (const std::string joint_name : kJointNames) {
    const RevoluteJoint<double>& joint =
        model->GetJointByName<RevoluteJoint>(joint_name);
    EXPECT_EQ(joint.get_name(), joint_name);
  }
  EXPECT_THROW(model->GetJointByName<RevoluteJoint>(kInvalidName),
               std::logic_error);
}

}  // namespace
}  // namespace multibody_model
}  // namespace multibody
}  // namespace drake

