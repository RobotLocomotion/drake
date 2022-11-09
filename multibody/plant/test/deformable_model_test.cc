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
using geometry::SceneGraphInspector;
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

  DeformableBodyId RegisterSphere(double resolution_hint) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(1), "sphere");
    DeformableBodyId body_id = deformable_model_ptr_->RegisterDeformableBody(
        std::move(geometry), default_body_config_, resolution_hint);
    return body_id;
  }
};

/* Verifies that a DeformableModel has been successfully created. */
TEST_F(DeformableModelTest, Constructor) {
  ASSERT_NE(deformable_model_ptr_, nullptr);
  EXPECT_EQ(deformable_model_ptr_->num_bodies(), 0);
}

TEST_F(DeformableModelTest, RegisterDeformableBody) {
  constexpr double kRezHint = 0.5;
  DeformableBodyId body_id = RegisterSphere(kRezHint);
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

/* Coarsely tests that SetWallBoundaryCondition adds some sort of boundary
 condition. Showing that boundary conditions only get conditionally added (based
 on location of the boundary wall) is sufficient evidence to infer that the
 right dofs are being given the right constraints. See
 deformable_boundary_condition_test.cc for a more complete test on the effect of
 calling this function. */
TEST_F(DeformableModelTest, SetWallBoundaryCondition) {
  constexpr double kRezHint = 0.5;
  DeformableBodyId body_id = RegisterSphere(kRezHint);
  const auto& fem_model = deformable_model_ptr_->GetFemModel(body_id);
  const auto& dirichlet_bc = fem_model.dirichlet_boundary_condition();
  /* No boundary condition has been added yet. */
  EXPECT_TRUE(dirichlet_bc.index_to_boundary_state().empty());
  /* Put the wall just far away enough so that no boundary condition is added.
   */
  const Eigen::Vector3d p_WQ1(0, 0, -1.0 - 1e-10);
  const Eigen::Vector3d n_W(0, 0, 1);
  deformable_model_ptr_->SetWallBoundaryCondition(body_id, p_WQ1, n_W);
  EXPECT_TRUE(dirichlet_bc.index_to_boundary_state().empty());
  /* Put the wall just close enough so that some boundary conditions are added.
   */
  const Eigen::Vector3d p_WQ2(0, 0, -1.0 + 1e-10);
  deformable_model_ptr_->SetWallBoundaryCondition(body_id, p_WQ2, n_W);
  EXPECT_FALSE(dirichlet_bc.index_to_boundary_state().empty());

  /* Throws when called on unregistered id. */
  const DeformableBodyId fake_body_id = DeformableBodyId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->SetWallBoundaryCondition(fake_body_id, p_WQ2, n_W),
      fmt::format(".*No.*id.*{}.*registered.*", fake_body_id));

  /* Setting boudnary condition must be done pre-finalize. */
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->SetWallBoundaryCondition(body_id, p_WQ2, n_W),
      ".*SetWallBoundaryCondition.*after system resources have been "
      "declared.*");
}

TEST_F(DeformableModelTest, DiscreteStateIndexAndReferencePositions) {
  constexpr double kRezHint = 0.5;
  Sphere sphere(1.0);
  auto geometry = make_unique<GeometryInstance>(
      RigidTransformd(), make_unique<Sphere>(sphere), "sphere");
  const DeformableBodyId body_id =
      deformable_model_ptr_->RegisterDeformableBody(
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

TEST_F(DeformableModelTest, GetBodyIdFromBodyIndex) {
  constexpr double kRezHint = 0.5;
  const DeformableBodyId body_id = RegisterSphere(kRezHint);
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetBodyId(DeformableBodyIndex(0)),
      ".*GetBodyId.*before system resources have been declared.*");
  plant_->Finalize();
  EXPECT_EQ(deformable_model_ptr_->GetBodyId(DeformableBodyIndex(0)), body_id);
  // Throws for invalid indexes.
  EXPECT_THROW(deformable_model_ptr_->GetBodyId(DeformableBodyIndex(1)),
               std::exception);
  EXPECT_THROW(deformable_model_ptr_->GetBodyId(DeformableBodyIndex()),
               std::exception);
}

TEST_F(DeformableModelTest, GetBodyIndex) {
  constexpr double kRezHint = 0.5;
  const DeformableBodyId body_id = RegisterSphere(kRezHint);
  /* Throws for pre-finalize call. */
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetBodyIndex(body_id),
                              ".*before system resources.*declared.*");
  plant_->Finalize();
  EXPECT_EQ(deformable_model_ptr_->GetBodyIndex(body_id),
            DeformableBodyIndex(0));
  /* Throws for unregistered body id. */
  const DeformableBodyId fake_body_id = DeformableBodyId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetBodyIndex(fake_body_id),
      fmt::format(".*No.*id.*{}.*registered.*", fake_body_id));
}

TEST_F(DeformableModelTest, GetGeometryId) {
  constexpr double kRezHint = 0.5;
  const DeformableBodyId body_id = RegisterSphere(kRezHint);
  geometry::GeometryId geometry_id =
      deformable_model_ptr_->GetGeometryId(body_id);
  const SceneGraphInspector<double>& inspector =
      scene_graph_->model_inspector();
  EXPECT_TRUE(inspector.IsDeformableGeometry(geometry_id));
  DeformableBodyId fake_id = DeformableBodyId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetGeometryId(fake_id),
                              "GetGeometryId.*No deformable body with id.*");
}

TEST_F(DeformableModelTest, GetBodyIdFromGeometryId) {
  constexpr double kRezHint = 0.5;
  const DeformableBodyId body_id = RegisterSphere(kRezHint);
  geometry::GeometryId geometry_id =
      deformable_model_ptr_->GetGeometryId(body_id);
  /* Test pre-finalize call. */
  EXPECT_EQ(deformable_model_ptr_->GetBodyId(geometry_id), body_id);
  plant_->Finalize();
  /* Test post-finalize call. */
  EXPECT_EQ(deformable_model_ptr_->GetBodyId(geometry_id), body_id);
  /* Throws for unregistered geometry id. */
  const geometry::GeometryId fake_geometry_id =
      geometry::GeometryId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetBodyId(fake_geometry_id),
      ".*GeometryId.*not.*registered.*");
}

TEST_F(DeformableModelTest, ToPhysicalModelPointerVariant) {
  PhysicalModelPointerVariant<double> variant =
      deformable_model_ptr_->ToPhysicalModelPointerVariant();
  EXPECT_TRUE(std::holds_alternative<const DeformableModel<double>*>(variant));
}

TEST_F(DeformableModelTest, VertexPositionsOutputPort) {
  Sphere sphere(1.0);
  auto geometry = make_unique<GeometryInstance>(
      RigidTransformd(), make_unique<Sphere>(sphere), "sphere");
  constexpr double kRezHint = 0.5;
  DeformableBodyId body_id = deformable_model_ptr_->RegisterDeformableBody(
      std::move(geometry), default_body_config_, kRezHint);
  plant_->Finalize();

  std::unique_ptr<systems::Context<double>> context =
      plant_->CreateDefaultContext();
  std::unique_ptr<AbstractValue> output_value =
      deformable_model_ptr_->vertex_positions_port().Allocate();
  /* Compute the configuration for each geometry in the model. */
  deformable_model_ptr_->vertex_positions_port().Calc(*context,
                                                      output_value.get());
  const geometry::GeometryConfigurationVector<double>& configurations =
      output_value->get_value<geometry::GeometryConfigurationVector<double>>();

  /* There's only one body and one geometry. */
  EXPECT_EQ(configurations.size(), 1);
  const geometry::GeometryId geometry_id =
      deformable_model_ptr_->GetGeometryId(body_id);
  ASSERT_TRUE(configurations.has_id(geometry_id));
  /* Verify that the vertex positions port returns expected values, which in
   this case should be the reference positions. */
  EXPECT_EQ(configurations.value(geometry_id),
            deformable_model_ptr_->GetReferencePositions(body_id));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
