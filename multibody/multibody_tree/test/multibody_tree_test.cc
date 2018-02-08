#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <functional>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/test_utilities/expect_error_message.h"

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

  // Create a non-finalized model of the arm so that we can test adding more
  // elements to it.
  std::unique_ptr<MultibodyTree<double>> model =
      MakeKukaIiwaModel<double>(false /* non-finalized model. */);

  // Verify the model was not finalized.
  EXPECT_FALSE(model->topology_is_valid());

  // Attempt to add a body having the same name as a body already part of the
  // model. This is not allowed and an exception should be thrown.
  DRAKE_EXPECT_ERROR_MESSAGE(
      model->AddRigidBody("iiwa_link_5", SpatialInertia<double>()),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      "This model already contains a body named 'iiwa_link_5'. "
      "Body names must be unique within a given model.");

  // Attempt to add a joint having the same name as a joint already part of the
  // model. This is not allowed and an exception should be thrown.
  DRAKE_EXPECT_ERROR_MESSAGE(
      model->AddJoint<RevoluteJoint>(
          "iiwa_joint_4",
          /* Dummy frame definitions. Not relevant for this test. */
          model->get_world_body(), {},
          model->get_world_body(), {},
          Vector3<double>::UnitZ()),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      "This model already contains a joint named 'iiwa_joint_4'. "
      "Joint names must be unique within a given model.");

  // Now we tested we cannot add body or joints with an existing name, finalize
  // the model.
  EXPECT_NO_THROW(model->Finalize());

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
  for (const std::string link_name : kLinkNames) {
    EXPECT_TRUE(model->HasBodyNamed(link_name));
  }
  EXPECT_FALSE(model->HasBodyNamed(kInvalidName));

  for (const std::string joint_name : kJointNames) {
    EXPECT_TRUE(model->HasJointNamed(joint_name));
  }
  EXPECT_FALSE(model->HasJointNamed(kInvalidName));

  // Get links by name.
  for (const std::string link_name : kLinkNames) {
    const Body<double>& link = model->GetBodyByName(link_name);
    EXPECT_EQ(link.get_name(), link_name);
  }
  DRAKE_EXPECT_ERROR_MESSAGE(
      model->GetBodyByName(kInvalidName), std::logic_error,
      "There is no body named '.*' in the model.");

  // Get joints by name.
  for (const std::string joint_name : kJointNames) {
    const Joint<double>& joint = model->GetJointByName(joint_name);
    EXPECT_EQ(joint.get_name(), joint_name);
  }
  DRAKE_EXPECT_ERROR_MESSAGE(
      model->GetJointByName(kInvalidName), std::logic_error,
      "There is no joint named '.*' in the model.");

  // Templatized version to obtain retrieve a particular known type of joint.
  for (const std::string joint_name : kJointNames) {
    const RevoluteJoint<double>& joint =
        model->GetJointByName<RevoluteJoint>(joint_name);
    EXPECT_EQ(joint.get_name(), joint_name);
  }
  DRAKE_EXPECT_ERROR_MESSAGE(
      model->GetJointByName<RevoluteJoint>(kInvalidName), std::logic_error,
      "There is no joint named '.*' in the model.");
}

}  // namespace
}  // namespace multibody_model
}  // namespace multibody
}  // namespace drake

