#include "drake/multibody/plant/deformable_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::Sphere;
using math::RigidTransformd;
using std::make_unique;

class DeformableModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    constexpr double kDt = 0.01;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder_, kDt);
    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    deformable_model_ptr_ = deformable_model.get();
    plant_->AddPhysicalModel(move(deformable_model));
  }

  systems::DiagramBuilder<double> builder_;
  const fem::DeformableBodyConfig<double> default_body_config_{};
  DeformableModel<double>* deformable_model_ptr_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
};

/* Verifies that a DeformableModel has been successfully created. */
TEST_F(DeformableModelTest, Constructor) {
  ASSERT_NE(deformable_model_ptr_, nullptr);
  EXPECT_EQ(deformable_model_ptr_->num_bodies(), 0);
}

TEST_F(DeformableModelTest, RegisterDeformableBody) {
  constexpr double kRezHint = 0.5;
  auto geometry = make_unique<GeometryInstance>(
      RigidTransformd(), make_unique<Sphere>(1), "sphere");
  DeformableBodyId body_id = deformable_model_ptr_->RegisterDeformableBody(
      std::move(geometry), default_body_config_, kRezHint);
  EXPECT_EQ(deformable_model_ptr_->num_bodies(), 1);
  /* Verify that a corresponding FemModel has been built. */
  EXPECT_NO_THROW(deformable_model_ptr_->GetFemModel(body_id));

  /* Registering deformable bodies after Finalize() is prohibited. */
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->RegisterDeformableBody(
          make_unique<GeometryInstance>(RigidTransformd(),
                                        make_unique<Sphere>(1), "sphere"),
          default_body_config_, kRezHint),
      ".*RegisterDeformableBody.*after system resources have been declared.*");
}

TEST_F(DeformableModelTest, DiscreteStateIndexAndReferencePositions) {
  constexpr double kRezHint = 0.5;
  Sphere sphere(1.0);
  auto geometry = make_unique<GeometryInstance>(
      RigidTransformd(), make_unique<Sphere>(sphere), "sphere");
  DeformableBodyId body_id = deformable_model_ptr_->RegisterDeformableBody(
      std::move(geometry), default_body_config_, kRezHint);

  /* Getting state index before Finalize() is prohibited. */
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetDiscreteStateIndex(body_id),
      ".*GetDiscreteStateIndex.*before system resources have been declared.*");

  /* Verify that the position values in  default discrete state is given by the
   reference positions. */
  plant_->Finalize();
  systems::DiscreteStateIndex state_index =
      deformable_model_ptr_->GetDiscreteStateIndex(body_id);
  auto context = plant_->CreateDefaultContext();
  const VectorX<double>& discrete_state =
      context->get_discrete_state(state_index).value();
  const int num_dofs = deformable_model_ptr_->GetFemModel(body_id).num_dofs();
  EXPECT_EQ(discrete_state.head(num_dofs),
            deformable_model_ptr_->GetReferencePositions(body_id));
  /* Verify that the velocity and acceleration values in default discrete state
   is given by the reference positions. */
  EXPECT_EQ(discrete_state.tail(2 * num_dofs),
            VectorX<double>::Zero(2 * num_dofs));
}

/* Verifies that calling any member function with an invalid body id throws,
 even if everything else was done correctly. */
TEST_F(DeformableModelTest, InvalidBodyId) {
  DeformableBodyId fake_id = DeformableBodyId::get_new_id();
  /* Pre-finalize calls. */
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetFemModel(fake_id),
                              "GetFemModel.*No deformable body with id.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetReferencePositions(fake_id),
      "GetReferencePositions.*No deformable body with id.*");

  plant_->Finalize();
  /* Post-finalize calls. */
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetDiscreteStateIndex(fake_id),
      "GetDiscreteStateIndex.*No deformable body with id.*");
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetFemModel(fake_id),
                              "GetFemModel.*No deformable body with id.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetReferencePositions(fake_id),
      "GetReferencePositions.*No deformable body with id.*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
