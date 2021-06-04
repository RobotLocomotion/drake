#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/dummy_model.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using multibody::RevoluteSpring;
using systems::Diagram;
using systems::DiagramBuilder;

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
  EXPECT_EQ(plant.num_velocities(), plant_ad->num_velocities());
  EXPECT_EQ(plant.num_positions(), plant_ad->num_positions());
  EXPECT_EQ(plant.num_force_elements(), plant_ad->num_force_elements());

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

// A simple concrete DiscreteUpdateManager that does not support scalar
// conversion to either AutoDiffXd or symbolic::Expression.
template <typename T>
class DoubleOnlyDiscreteUpdateManager
    : public multibody::internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoubleOnlyDiscreteUpdateManager);
  DoubleOnlyDiscreteUpdateManager() = default;
  ~DoubleOnlyDiscreteUpdateManager() final = default;

  bool is_cloneable_to_double() const final { return true; }

 private:
  std::unique_ptr<multibody::internal::DiscreteUpdateManager<double>>
  CloneToDouble() const final {
    auto clone = std::make_unique<DoubleOnlyDiscreteUpdateManager<double>>();
    return clone;
  }

  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final {}

  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      internal::AccelerationKinematicsCache<T>*) const final {}

  void DoCalcDiscreteValues(const systems::Context<T>&,
                            systems::DiscreteValues<T>*) const final {}
};

// This test verifies that adding external components that do not support some
// scalar types removes MultibodyPlant's ability to scalar convert to those
// scalar types.
GTEST_TEST(ScalarConversionTest, ExternalComponent) {
  MultibodyPlant<double> plant(0.1);
  std::unique_ptr<internal::PhysicalModel<double>> dummy_physical_model =
      std::make_unique<internal::test::DummyModel<double>>();
  EXPECT_TRUE(dummy_physical_model->is_cloneable_to_double());
  EXPECT_TRUE(dummy_physical_model->is_cloneable_to_autodiff());
  EXPECT_FALSE(dummy_physical_model->is_cloneable_to_symbolic());
  plant.AddPhysicalModel(std::move(dummy_physical_model));
  plant.Finalize();

  // double -> AutoDiffXd
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff;
  EXPECT_NO_THROW(plant_autodiff =
                      drake::systems::System<double>::ToAutoDiffXd(plant));
  // AutoDiffXd -> double
  EXPECT_NO_THROW(plant_autodiff->ToScalarType<double>());
  // double -> symbolic::Expression
  std::unique_ptr<MultibodyPlant<symbolic::Expression>>
      plant_double_to_symbolic;
  EXPECT_THROW(plant_double_to_symbolic =
                   drake::systems::System<double>::ToSymbolic(plant),
               std::exception);
  // double -> symbolic::Expression
  std::unique_ptr<MultibodyPlant<symbolic::Expression>>
      plant_autodiff_to_symbolic;
  EXPECT_THROW(
      plant_autodiff_to_symbolic =
          drake::systems::System<AutoDiffXd>::ToSymbolic(*plant_autodiff),
      std::exception);

  // Verify that adding a component that doesn't allow scalar conversion to
  // autodiff does not prevent scalar conversion to double.
  auto discrete_update_manager =
      std::make_unique<DoubleOnlyDiscreteUpdateManager<AutoDiffXd>>();
  EXPECT_TRUE(discrete_update_manager->is_cloneable_to_double());
  EXPECT_FALSE(discrete_update_manager->is_cloneable_to_autodiff());
  EXPECT_FALSE(discrete_update_manager->is_cloneable_to_symbolic());
  plant_autodiff->SetDiscreteUpdateManager(std::move(discrete_update_manager));
  EXPECT_NO_THROW(plant_autodiff->ToScalarType<double>());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
