#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using Eigen::VectorXd;

namespace drake {

using multibody::Parser;
using systems::Context;
using systems::Diagram;

namespace multibody {
namespace {

class MultibodyPlantGravityForceTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Load two Iiwa models.
    const std::string url =
        "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);

    // Add the model twice.
    iiwa1_ = Parser(plant_.get(), "iiwa1").AddModelsFromUrl(url).at(0);
    iiwa2_ = Parser(plant_.get(), "iiwa2").AddModelsFromUrl(url).at(0);
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetFrameByName("iiwa_link_0", iiwa1_));
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetFrameByName("iiwa_link_0", iiwa2_));
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_{nullptr};
  ModelInstanceIndex iiwa1_;
  ModelInstanceIndex iiwa2_;
};

TEST_F(MultibodyPlantGravityForceTest,
       UniformGravityFieldElementApisToDisableGravity) {
  UniformGravityFieldElement<double>& gravity = plant_->mutable_gravity_field();
  // Enabled by default.
  EXPECT_TRUE(gravity.is_enabled(iiwa1_));
  EXPECT_TRUE(gravity.is_enabled(iiwa2_));

  // Disable gravity.
  gravity.set_enabled(iiwa1_, false);
  EXPECT_FALSE(gravity.is_enabled(iiwa1_));
  EXPECT_TRUE(gravity.is_enabled(iiwa2_));
  gravity.set_enabled(iiwa2_, false);
  EXPECT_FALSE(gravity.is_enabled(iiwa1_));
  EXPECT_FALSE(gravity.is_enabled(iiwa2_));

  // Enable gravity.
  gravity.set_enabled(iiwa1_, true);
  EXPECT_TRUE(gravity.is_enabled(iiwa1_));
  EXPECT_FALSE(gravity.is_enabled(iiwa2_));
  gravity.set_enabled(iiwa2_, true);
  EXPECT_TRUE(gravity.is_enabled(iiwa1_));
  EXPECT_TRUE(gravity.is_enabled(iiwa2_));
}

TEST_F(MultibodyPlantGravityForceTest, InvalidModelInstancesThrow) {
  UniformGravityFieldElement<double>& gravity = plant_->mutable_gravity_field();
  const ModelInstanceIndex invalid_model_instance(
      plant_->num_model_instances());
  DRAKE_EXPECT_THROWS_MESSAGE(gravity.is_enabled(invalid_model_instance),
                              "Model instance index is invalid.");
  DRAKE_EXPECT_THROWS_MESSAGE(gravity.set_enabled(invalid_model_instance, true),
                              "Model instance index is invalid.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->is_gravity_enabled(invalid_model_instance),
      "Model instance index is invalid.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->set_gravity_enabled(invalid_model_instance, true),
      "Model instance index is invalid.");
}

TEST_F(MultibodyPlantGravityForceTest,
       UniformGravityFieldElementSetEnabledThrowsIfCalledPostFinalize) {
  UniformGravityFieldElement<double>& gravity = plant_->mutable_gravity_field();
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(gravity.set_enabled(iiwa1_, true),
                              "Gravity can only be enabled pre-finalize.");
}

TEST_F(MultibodyPlantGravityForceTest,
       MultibodyPlantSetEnabledThrowsIfCalledPostFinalize) {
  // Pre-finalize we can call enable/disable as we please.
  EXPECT_NO_THROW(plant_->set_gravity_enabled(iiwa1_, true));

  // Verify that enabling/disabling gravity post-finalize throws.
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->set_gravity_enabled(iiwa1_, true),
      "Post-finalize calls to 'set_gravity_enabled\\(\\)' are not allowed; .*");
}

TEST_F(MultibodyPlantGravityForceTest, DisableGravity) {
  EXPECT_TRUE(plant_->is_gravity_enabled(iiwa1_));
  EXPECT_TRUE(plant_->is_gravity_enabled(iiwa2_));
  // We disable gravity on the first model only.
  plant_->set_gravity_enabled(iiwa1_, false);
  EXPECT_FALSE(plant_->is_gravity_enabled(iiwa1_));
  EXPECT_TRUE(plant_->is_gravity_enabled(iiwa2_));
  plant_->Finalize();
  std::unique_ptr<Context<double>> context = plant_->CreateDefaultContext();
  const VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*context);
  const VectorXd tau_g1 = plant_->GetVelocitiesFromArray(iiwa1_, tau_g);
  EXPECT_EQ(tau_g1.norm(), 0.0);
  const VectorXd tau_g2 = plant_->GetVelocitiesFromArray(iiwa2_, tau_g);
  EXPECT_GT(tau_g2.norm(), 0.01 /* arbitrary, clearly non-zero, value */);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
