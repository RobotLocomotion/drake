#include "drake/multibody/plant/physical_model_collection.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/dummy_physical_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

GTEST_TEST(PhysicalModelCollectionTest, EmptyCollection) {
  MultibodyPlant<double> plant(0.01);
  PhysicalModelCollection<double> model_collection;
  model_collection.DeclareSystemResources();

  EXPECT_EQ(model_collection.dummy_model(), nullptr);
  EXPECT_EQ(model_collection.mutable_dummy_model(), nullptr);
  EXPECT_EQ(model_collection.deformable_model(), nullptr);
  EXPECT_EQ(model_collection.mutable_deformable_model(), nullptr);

  EXPECT_TRUE(model_collection.is_cloneable_to_double());
  EXPECT_TRUE(model_collection.is_cloneable_to_autodiff());
  EXPECT_TRUE(model_collection.is_cloneable_to_symbolic());

  EXPECT_EQ(model_collection.owned_models().size(), 0);

  /* Target owning plants. */
  MultibodyPlant<double> double_plant(0.01);
  MultibodyPlant<AutoDiffXd> autodiff_plant(0.01);
  MultibodyPlant<symbolic::Expression> symbolic_plant(0.01);

  EXPECT_NO_THROW(model_collection.CloneToScalar<double>(&double_plant));
  EXPECT_NO_THROW(model_collection.CloneToScalar<AutoDiffXd>(&autodiff_plant));
  EXPECT_NO_THROW(
      model_collection.CloneToScalar<symbolic::Expression>(&symbolic_plant));
}

GTEST_TEST(PhysicalModelCollectionTest, AddEmptyModels) {
  MultibodyPlant<double> plant(0.01);
  PhysicalModelCollection<double> model_collection;

  auto dummy_model = std::make_unique<DummyPhysicalModel<double>>(&plant);
  DummyPhysicalModel<double>* dummy_model_ptr = dummy_model.get();
  model_collection.AddDummyModel(std::move(dummy_model));

  EXPECT_EQ(model_collection.dummy_model(), dummy_model_ptr);
  EXPECT_EQ(model_collection.mutable_dummy_model(), dummy_model_ptr);
  EXPECT_EQ(model_collection.deformable_model(), nullptr);
  EXPECT_EQ(model_collection.mutable_deformable_model(), nullptr);
  EXPECT_EQ(model_collection.owned_models().size(), 1);

  auto deformable_model = std::make_unique<DeformableModel<double>>(&plant);
  DeformableModel<double>* deformable_model_ptr = deformable_model.get();
  model_collection.AddDeformableModel(std::move(deformable_model));

  EXPECT_EQ(model_collection.dummy_model(), dummy_model_ptr);
  EXPECT_EQ(model_collection.mutable_dummy_model(), dummy_model_ptr);
  EXPECT_EQ(model_collection.deformable_model(), deformable_model_ptr);
  EXPECT_EQ(model_collection.mutable_deformable_model(), deformable_model_ptr);
  EXPECT_EQ(model_collection.owned_models().size(), 2);

  /* Mimic Finalizing the plant after all PhysicalModels have been added. */
  model_collection.DeclareSystemResources();
  plant.Finalize();

  /* Neither the dummy model or the empty deformable model prevents scalar
   conversion. */
  EXPECT_TRUE(model_collection.is_cloneable_to_double());
  EXPECT_TRUE(model_collection.is_cloneable_to_autodiff());
  EXPECT_TRUE(model_collection.is_cloneable_to_symbolic());
  /* Target owning plants. */
  MultibodyPlant<double> double_plant(0.01);
  MultibodyPlant<AutoDiffXd> autodiff_plant(0.01);
  MultibodyPlant<symbolic::Expression> symbolic_plant(0.01);

  EXPECT_NO_THROW(model_collection.CloneToScalar<double>(&double_plant));
  EXPECT_NO_THROW(model_collection.CloneToScalar<AutoDiffXd>(&autodiff_plant));
  EXPECT_NO_THROW(
      model_collection.CloneToScalar<symbolic::Expression>(&symbolic_plant));
}

/* Adding a body to the deformable model prevents the model from scalar
 converting to anything but double. Consequently, this prevents scalar
 conversion to anything but double for the collection too. */
GTEST_TEST(PhysicalModelCollectionTest, NonEmptyDeformableModel) {
  systems::DiagramBuilder<double> builder;
  /* Use discrete plant and contact approximation that allows us to add
   deformable bodies. */
  MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.01;
  plant_config.discrete_contact_approximation = "sap";
  auto [plant, _] = AddMultibodyPlant(plant_config, &builder);
  PhysicalModelCollection<double> model_collection;

  DeformableModel<double>& deformable_model =
      model_collection.AddDeformableModel(
          std::make_unique<DeformableModel<double>>(&plant));

  /* Add a deformable body to the DeformableModel. */
  auto geometry = std::make_unique<geometry::GeometryInstance>(
      math::RigidTransformd{}, std::make_unique<geometry::Sphere>(1.0),
      "sphere");
  deformable_model.RegisterDeformableBody(
      std::move(geometry), fem::DeformableBodyConfig<double>{}, 1.0);

  /* Mimic Finalizing the plant after all PhysicalModels have been added. */
  model_collection.DeclareSystemResources();
  plant.Finalize();

  EXPECT_TRUE(model_collection.is_cloneable_to_double());
  EXPECT_FALSE(model_collection.is_cloneable_to_autodiff());
  EXPECT_FALSE(model_collection.is_cloneable_to_symbolic());
  /* Target owning plants. */
  MultibodyPlant<double> double_plant(0.01);
  EXPECT_NO_THROW(model_collection.CloneToScalar<double>(&double_plant));
}

GTEST_TEST(PhysicalModelCollectionTest, IncompatibleModel) {
  MultibodyPlant<double> plant(0.01);
  PhysicalModelCollection<double> model_collection;
  model_collection.AddDeformableModel(
      std::make_unique<DeformableModel<double>>(&plant));

  MultibodyPlant<double> another_plant(0.01);
  auto model_with_wrong_plant =
      std::make_unique<DummyPhysicalModel<double>>(&another_plant);
  auto ok_model = std::make_unique<DummyPhysicalModel<double>>(&plant);
  DRAKE_EXPECT_THROWS_MESSAGE(
      model_collection.AddDummyModel(std::move(model_with_wrong_plant)),
      "The given model belongs to a different MultibodyPlant.");
  EXPECT_NO_THROW(model_collection.AddDummyModel(std::move(ok_model)));
}

GTEST_TEST(PhysicalModelCollectionTest, DeclareSceneGraphPorts) {
  MultibodyPlant<double> plant(0.01);
  PhysicalModelCollection<double> model_collection;

  model_collection.AddDummyModel(
      std::make_unique<DummyPhysicalModel<double>>(&plant));
  const DummyPhysicalModel<double>& dummy_model =
      *model_collection.dummy_model();
  DRAKE_ASSERT_THROWS_MESSAGE(dummy_model.GetSceneGraphPortOrThrow(),
                              ".*SceneGraph.*port.*not.*declared.*");
  model_collection.DeclareSceneGraphPorts();
  EXPECT_NO_THROW(dummy_model.GetSceneGraphPortOrThrow());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
