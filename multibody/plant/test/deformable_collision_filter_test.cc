#include <gtest/gtest.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using geometry::Box;
using geometry::CollisionFilterDeclaration;
using geometry::CollisionFilterScope;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::GeometrySet;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::Sphere;
using geometry::VolumeMesh;
using math::RigidTransformd;

class DeformableCollisionFilterTest : public ::testing::Test {
 protected:
  /* Sets up a scene with a deformable sphere intersecting two rigid bodies, one
   welded to the world and the other free. */
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, 0.01);
    auto deformable_model = std::make_unique<DeformableModel<double>>(plant_);
    DeformableBodyId deformable_body_id =
        RegisterDeformableOctahedron(deformable_model.get(), "deformable");
    deformable_geometry_id_ =
        deformable_model->GetGeometryId(deformable_body_id);
    model_ = deformable_model.get();
    plant_->AddPhysicalModel(std::move(deformable_model));
    // N.B. Deformables are only supported with the SAP solver.
    // Thus for testing we choose one arbitrary contact approximation that uses
    // the SAP solver.
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    /* Register a rigid body welded to the world. */
    ProximityProperties proximity_prop;
    geometry::AddContactMaterial({}, {}, CoulombFriction{0.4, 0.4},
                                 &proximity_prop);
    /* A resolution hint property is required for rigid body to collide with
     deformable bodies. */
    proximity_prop.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kRezHint, 1.0);
    const SpatialInertia<double> M_Bcm =
        SpatialInertia<double>::SolidCubeWithMass(1.0, 1.0);
    const RigidBody<double>& welded_body =
        plant_->AddRigidBody("welded body", M_Bcm);
    plant_->AddJoint<WeldJoint>("weld joint", plant_->world_body(),
                                std::nullopt, welded_body, std::nullopt,
                                RigidTransformd::Identity());
    const Box box(1, 1, 1);
    welded_geometry_id_ = plant_->RegisterCollisionGeometry(
        welded_body, RigidTransformd::Identity(), box, "welded",
        proximity_prop);

    const RigidBody<double>& free_body =
        plant_->AddRigidBody("free body", M_Bcm);
    free_geometry_id_ = plant_->RegisterCollisionGeometry(
        free_body, RigidTransformd::Identity(), box, "free", proximity_prop);

    /* ExcludeCollisionGeometriesWithCollisionFilterGroupPair() is invoked
     during parsing, but it still must adhere to MultibodyPlant's promise to
     not introduce collision filters on deformable bodies. Rather than exercise
     via parsing, we'll execute it here with some collision filters which
     *explicitly* include the deformable body's geometry id. If the dut has
     correct behavior (ignoring deformable geometries), there will be no
     collision filters involving the deformable geometry. */
    GeometrySet welded_and_deformable{deformable_geometry_id_,
                                      welded_geometry_id_};
    GeometrySet free_and_deformable{deformable_geometry_id_, free_geometry_id_};
    std::pair<std::string, GeometrySet> collision_filter_group_a = {
        "a", welded_and_deformable};
    std::pair<std::string, GeometrySet> collision_filter_group_b = {
        "b", free_and_deformable};
    plant_->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
        collision_filter_group_a, collision_filter_group_a);
    plant_->ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
        collision_filter_group_a, collision_filter_group_b);
    DRAKE_DEMAND(!scene_graph_->model_inspector().CollisionFiltered(
        deformable_geometry_id_, welded_geometry_id_));
    DRAKE_DEMAND(!scene_graph_->model_inspector().CollisionFiltered(
        deformable_geometry_id_, free_geometry_id_));

    plant_->Finalize();

    builder.Connect(model_->vertex_positions_port(),
                    scene_graph_->get_source_configuration_port(
                        plant_->get_source_id().value()));

    diagram_ = builder.Build();
  }

  /* Registers a deformable octahedron with the given `name` in the world to the
   given `model`. The octahedron looks like this in its geometry frame, F.
                    +Fz   -Fx
                     |   /
                     v5 v3
                     | /
                     |/
     -Fy---v4------v0+------v2---+ Fy
                    /| Fo
                   / |
                 v1  v6
                 /   |
               +Fx   |
                    -Fz     */
  DeformableBodyId RegisterDeformableOctahedron(DeformableModel<double>* model,
                                                std::string name) {
    auto geometry = std::make_unique<GeometryInstance>(
        RigidTransformd(), std::make_unique<Sphere>(2.0), std::move(name));
    ProximityProperties props;
    geometry::AddContactMaterial({}, {}, CoulombFriction<double>(0.3, 0.3),
                                 &props);
    geometry->set_proximity_properties(std::move(props));
    fem::DeformableBodyConfig<double> body_config;
    /* Make the resolution hint large enough so that we get an octahedron. */
    constexpr double kRezHint = 10.0;
    DeformableBodyId id = model->RegisterDeformableBody(std::move(geometry),
                                                        body_config, kRezHint);
    /* Verify that the geometry has 7 vertices and is indeed an octahedron. */
    const SceneGraphInspector<double>& inspector =
        scene_graph_->model_inspector();
    GeometryId g_id = model->GetGeometryId(id);
    const VolumeMesh<double>* mesh_G = inspector.GetReferenceMesh(g_id);
    DRAKE_DEMAND(mesh_G != nullptr);
    DRAKE_DEMAND(mesh_G->num_vertices() == 7);
    return id;
  }

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  DeformableModel<double>* model_{nullptr};
  GeometryId deformable_geometry_id_;
  GeometryId welded_geometry_id_;
  GeometryId free_geometry_id_;
  std::unique_ptr<systems::Diagram<double>> diagram_{nullptr};
};

/* Tests that the default collision filters applied in MbP do not affect
 deformable bodies. That is, the deformable body is in contact with both rigid
 geometries. */
TEST_F(DeformableCollisionFilterTest, DefaultMbPFilterIgnoresDeformable) {
  auto diagram_context = diagram_->CreateDefaultContext();
  const auto& scene_graph_context =
      scene_graph_->GetMyContextFromRoot(*diagram_context);
  const QueryObject<double>& query_object =
      scene_graph_->get_query_output_port().Eval<QueryObject<double>>(
          scene_graph_context);
  geometry::internal::DeformableContact<double> deformable_contact;
  query_object.ComputeDeformableContact(&deformable_contact);
  /* Both rigid geometries should still be in collision with the deformable
   geometry. */
  EXPECT_EQ(deformable_contact.contact_surfaces().size(), 2);
}

/* Tests collision filter has an effect on deformable geometries if a collision
 filter is declared on the deformable geometry with the appropriate scope. */
TEST_F(DeformableCollisionFilterTest, ExplicitlyFilterDeformable) {
  scene_graph_->collision_filter_manager().Apply(
      CollisionFilterDeclaration(CollisionFilterScope::kAll)
          .ExcludeBetween(GeometrySet(deformable_geometry_id_),
                          GeometrySet(welded_geometry_id_)));
  auto diagram_context = diagram_->CreateDefaultContext();
  const auto& scene_graph_context =
      scene_graph_->GetMyContextFromRoot(*diagram_context);
  const QueryObject<double>& query_object =
      scene_graph_->get_query_output_port().Eval<QueryObject<double>>(
          scene_graph_context);
  geometry::internal::DeformableContact<double> deformable_contact;
  query_object.ComputeDeformableContact(&deformable_contact);
  /* Only the free body's collision geometry is still in collision with the
   deformable geometry. */
  ASSERT_EQ(deformable_contact.contact_surfaces().size(), 1);
  EXPECT_EQ(deformable_contact.contact_surfaces()[0].id_A(),
            deformable_geometry_id_);
  EXPECT_EQ(deformable_contact.contact_surfaces()[0].id_B(), free_geometry_id_);
}

/* Tests collision filter has no effect on deformable geometries if a collision
 filter is declared on the deformable geometry with the omit deformable scope.
 */
TEST_F(DeformableCollisionFilterTest, ExplicitlyFilterDeformableWrongScope) {
  scene_graph_->collision_filter_manager().Apply(
      CollisionFilterDeclaration(CollisionFilterScope::kOmitDeformable)
          .ExcludeBetween(GeometrySet(deformable_geometry_id_),
                          GeometrySet(welded_geometry_id_)));
  auto diagram_context = diagram_->CreateDefaultContext();
  const auto& scene_graph_context =
      scene_graph_->GetMyContextFromRoot(*diagram_context);
  const QueryObject<double>& query_object =
      scene_graph_->get_query_output_port().Eval<QueryObject<double>>(
          scene_graph_context);
  geometry::internal::DeformableContact<double> deformable_contact;
  query_object.ComputeDeformableContact(&deformable_contact);
  /* Both rigid geometries should still be in collision with the deformable
   geometry. */
  EXPECT_EQ(deformable_contact.contact_surfaces().size(), 2);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
