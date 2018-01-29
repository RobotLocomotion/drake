#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace multibody_plant {
namespace {

using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;

GTEST_TEST(RigidBodyPlant, SimpleModelCreation) {
  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant = MakeAcrobotPlant(parameters);

  const std::string kLink1Name = "Link1";
  const std::string kLink2Name = "Link2";
  const std::string kShoulderJointName = "ShoulderJoint";
  const std::string kElbowJointName = "ElbowJoint";
  const std::string kInvalidName = "InvalidName";

  // Model Size. Counting the world body, there should be three bodies.
  EXPECT_EQ(plant->num_bodies(), 3);
  EXPECT_EQ(plant->num_joints(), 2);

  // State size.
  EXPECT_EQ(plant->num_positions(), 2);
  EXPECT_EQ(plant->num_velocities(), 2);
  EXPECT_EQ(plant->num_states(), 4);

  // Query if elements exist in the model.
  EXPECT_TRUE(plant->HasBodyNamed("Link1"));
  EXPECT_TRUE(plant->HasBodyNamed("Link2"));
  EXPECT_FALSE(plant->HasBodyNamed("InvalidLinkName"));

  EXPECT_TRUE(plant->HasJointNamed("ShoulderJoint"));
  EXPECT_TRUE(plant->HasJointNamed("ElbowJoint"));
  EXPECT_FALSE(plant->HasJointNamed("InvalidJointName"));

  // Get links by name.
  const Body<double>& link1 = plant->GetBodyByName(kLink1Name);
  EXPECT_EQ(link1.get_name(), kLink1Name);
  const Body<double>& link2 = plant->GetBodyByName(kLink2Name);
  EXPECT_EQ(link2.get_name(), kLink2Name);

  // Attempting to retrieve a link that is not part of the model should throw
  // an exception.
  EXPECT_THROW(plant->GetBodyByName("InvalidLinkName"), std::logic_error);

  // Get joints by name.
  const Joint<double>& shoulder_joint =
      plant->GetJointByName(kShoulderJointName);
  EXPECT_EQ(shoulder_joint.get_name(), kShoulderJointName);
  const Joint<double>& elbow_joint = plant->GetJointByName(kElbowJointName);
  EXPECT_EQ(elbow_joint.get_name(), kElbowJointName);
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // Templatized version to obtain retrieve a particular known type of joint.
  const RevoluteJoint<double>& shoulder =
      plant->GetJointByName<RevoluteJoint>(kShoulderJointName);
  EXPECT_EQ(shoulder.get_name(), kShoulderJointName);
  const RevoluteJoint<double>& elbow =
      plant->GetJointByName<RevoluteJoint>(kElbowJointName);
  EXPECT_EQ(elbow.get_name(), kElbowJointName);
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

