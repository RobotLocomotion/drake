#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {

using drake::multibody::RevoluteSpring;

namespace multibody {
namespace {

// We test that we can scalar convert a plant containing a revolute joint and
// spring. In particular, we verify that the spring element correctly references
// the joint both before and after scalar conversion.
GTEST_TEST(ScalarConversionTest, RevoluteJointAndSpring) {
  MultibodyPlant<double> plant(0.0);
  // For this test inertia values are irrelevant.
  const RigidBody<double>& body =
      plant.AddRigidBody("Body", SpatialInertia<double>());
  const RevoluteJoint<double>& pin = plant.AddJoint<RevoluteJoint>(
      "Pin", plant.world_body(), std::nullopt, body, std::nullopt,
      Vector3<double>::UnitZ());
  const auto& spring = plant.AddForceElement<RevoluteSpring>(pin, 0, 1500);

  // We verify the correct reference to the pin joint before conversion.
  EXPECT_EQ(&pin, &spring.joint());

  // We are done defining the model.
  plant.Finalize();

  // Sanity check for the model's size.
  DRAKE_DEMAND(plant.num_velocities() == 1);
  DRAKE_DEMAND(plant.num_positions() == 1);
  // The plant has a UniformGravityFieldElement by default plus the
  // RevoluteSpring.
  DRAKE_DEMAND(plant.num_force_elements() == 2);

  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad;
  EXPECT_NO_THROW(plant_ad =
                      drake::systems::System<double>::ToAutoDiffXd(plant));
  DRAKE_DEMAND(plant_ad->num_velocities() == 1);
  DRAKE_DEMAND(plant_ad->num_positions() == 1);
  DRAKE_DEMAND(plant_ad->num_force_elements() == 2);

  // We verify the correct reference to the pin joint after conversion.
  const auto& pin_ad = plant_ad->GetJointByName<RevoluteJoint>(pin.name());
  const auto& spring_ad =
      plant_ad->GetForceElement<RevoluteSpring>(spring.index());
  // Verify correct cross-referencing in the scalar converted model.
  EXPECT_EQ(&pin_ad, &spring_ad.joint());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
