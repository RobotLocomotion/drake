#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using multibody::RevoluteSpring;
using systems::DiagramBuilder;
using systems::Diagram;

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

// Verifies that two MultibodyPlants have the same port indices.
template <typename T, typename U>
void CompareMultibodyPlantPortIndices(const MultibodyPlant<T>& plant_t,
                                      const MultibodyPlant<U>& plant_u) {
  // Check input ports.
  // (Except actuation input ports because there is no actuation source.)
  EXPECT_EQ(plant_t.get_applied_generalized_force_input_port().get_index(),
            plant_u.get_applied_generalized_force_input_port().get_index());
  EXPECT_EQ(plant_t.get_applied_spatial_force_input_port().get_index(),
            plant_u.get_applied_spatial_force_input_port().get_index());
  EXPECT_EQ(plant_t.get_geometry_query_input_port().get_index(),
            plant_u.get_geometry_query_input_port().get_index());
  // Check output ports.
  EXPECT_EQ(plant_t.get_body_poses_output_port().get_index(),
            plant_u.get_body_poses_output_port().get_index());
  EXPECT_EQ(plant_t.get_body_spatial_velocities_output_port().get_index(),
            plant_u.get_body_spatial_velocities_output_port().get_index());
  EXPECT_EQ(plant_t.get_body_spatial_accelerations_output_port().get_index(),
            plant_u.get_body_spatial_accelerations_output_port().get_index());
  EXPECT_EQ(plant_t.get_state_output_port().get_index(),
            plant_u.get_state_output_port().get_index());
  EXPECT_EQ(plant_t.get_generalized_acceleration_output_port().get_index(),
            plant_u.get_generalized_acceleration_output_port().get_index());
  EXPECT_EQ(plant_t.get_reaction_forces_output_port().get_index(),
            plant_u.get_reaction_forces_output_port().get_index());
  EXPECT_EQ(plant_t.get_contact_results_output_port().get_index(),
            plant_u.get_contact_results_output_port().get_index());
  EXPECT_EQ(plant_t.get_geometry_poses_output_port().get_index(),
            plant_u.get_geometry_poses_output_port().get_index());
  EXPECT_EQ(
      plant_t.get_state_output_port(default_model_instance()).get_index(),
      plant_u.get_state_output_port(default_model_instance()).get_index());
  EXPECT_EQ(
      plant_t.get_generalized_acceleration_output_port(default_model_instance())
          .get_index(),
      plant_u.get_generalized_acceleration_output_port(default_model_instance())
          .get_index());
  EXPECT_EQ(
      plant_t
          .get_generalized_contact_forces_output_port(default_model_instance())
          .get_index(),
      plant_u
          .get_generalized_contact_forces_output_port(default_model_instance())
          .get_index());
}

// This test verifies that the port indices of the input/output ports of
// MultibodyPlant remain the same after scalar conversion.
GTEST_TEST(ScalarConversionTest, PortIndexOrdering) {
  systems::DiagramBuilder<double> builder;
  std::unique_ptr<MultibodyPlant<double>> plant_unique_ptr =
      std::make_unique<MultibodyPlant<double>>(0.0);
  plant_unique_ptr->Finalize();
  auto pair =
      AddMultibodyPlantSceneGraph(&builder, std::move(plant_unique_ptr));
  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  std::unique_ptr<Diagram<AutoDiffXd>> autodiff_diagram =
      systems::System<double>::ToAutoDiffXd<Diagram>(*diagram);

  // Making the assumption that the system at index 0 is the plant
  const MultibodyPlant<AutoDiffXd>* autodiff_plant =
      dynamic_cast<const MultibodyPlant<AutoDiffXd>*>(
          autodiff_diagram->GetSystems()[0]);
  DRAKE_DEMAND(autodiff_plant != nullptr);

  CompareMultibodyPlantPortIndices(pair.plant, *autodiff_plant);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
