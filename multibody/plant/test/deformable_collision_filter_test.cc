#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/sorted_pair.h"
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
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder_, 0.01);
    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();
    DeformableBodyId deformable_body_id =
        RegisterDeformableOctahedron(&deformable_model, "deformable");
    deformable_geometry_id_ =
        deformable_model.GetGeometryId(deformable_body_id);
    // N.B. Deformables are only supported with the SAP solver.
    // Thus for testing we choose one arbitrary contact approximation that uses
    // the SAP solver.
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    /* Register a rigid body welded to the world. */
    ProximityProperties proximity_prop;
    geometry::AddContactMaterial({}, {}, CoulombFriction{0.4, 0.4},
                                 &proximity_prop);
    /* A resolution hint property and a hydroelastic modulus are required for
     rigid body to collide with deformable bodies. */
    proximity_prop.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kRezHint, 1.0);
    proximity_prop.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kElastic, 1e6);
    const SpatialInertia<double> M_Bcm =
        SpatialInertia<double>::SolidCubeWithMass(1.0, 1.0);
    const RigidBody<double>& welded_body =
        plant_->AddRigidBody("welded body", M_Bcm);
    plant_->AddJoint<WeldJoint>("weld joint", plant_->world_body(),
                                std::nullopt, welded_body, std::nullopt,
                                RigidTransformd::Identity());
    const Box box(1, 1, 1);
    welded_geometry_id_ = plant_->RegisterCollisionGeometry(
        welded_body, RigidTransformd(Vector3d(-0.75, 0, 0)), box, "welded",
        proximity_prop);

    const RigidBody<double>& free_body =
        plant_->AddRigidBody("free body", M_Bcm);
    free_geometry_id_ = plant_->RegisterCollisionGeometry(
        free_body, RigidTransformd(Vector3d(0.75, 0, 0)), box, "free",
        proximity_prop);
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
        RigidTransformd(), std::make_unique<Sphere>(1.0), std::move(name));
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

  systems::DiagramBuilder<double> builder_;
  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  GeometryId deformable_geometry_id_;
  GeometryId welded_geometry_id_;
  GeometryId free_geometry_id_;
};

/* Tests that multiple deformable bodies, which are adjacent to each other as
 children of the world body, are not automatically filtered. */
TEST_F(DeformableCollisionFilterTest, DefaultMbPFilterIgnoresDeformable) {
  /* Add a second deformable body to the scene which is in contact with the
   other rigid and deformable bodies. */
  DeformableModel<double>& deformable_model =
      plant_->mutable_deformable_model();
  DeformableBodyId other_deformable_body_id =
      RegisterDeformableOctahedron(&deformable_model, "other_deformable");
  GeometryId other_deformable_geometry_id =
      deformable_model.GetGeometryId(other_deformable_body_id);

  /* Contact detection not allowed pre-Finalize. */
  plant_->Finalize();
  auto diagram = builder_.Build();

  auto diagram_context = diagram->CreateDefaultContext();
  const auto& scene_graph_context =
      scene_graph_->GetMyContextFromRoot(*diagram_context);
  const QueryObject<double>& query_object =
      scene_graph_->get_query_output_port().Eval<QueryObject<double>>(
          scene_graph_context);
  geometry::internal::DeformableContact<double> deformable_contact;
  query_object.ComputeDeformableContact(&deformable_contact);
  /* No contacts are filtered, so each deformable contacts the other and both of
   the rigid bodies. The rigid bodies do not contact each other due to their
   placement. */
  EXPECT_EQ(deformable_contact.contact_surfaces().size(), 5);

  std::unordered_set<SortedPair<GeometryId>> expected_pairs = {
      {deformable_geometry_id_, welded_geometry_id_},
      {deformable_geometry_id_, free_geometry_id_},
      {deformable_geometry_id_, other_deformable_geometry_id},
      {other_deformable_geometry_id, welded_geometry_id_},
      {other_deformable_geometry_id, free_geometry_id_},
  };

  std::unordered_set<SortedPair<GeometryId>> actual_pairs = {};
  for (const auto& contact_surface : deformable_contact.contact_surfaces()) {
    actual_pairs.insert({contact_surface.id_A(), contact_surface.id_B()});
  }

  EXPECT_THAT(expected_pairs, ::testing::ContainerEq(actual_pairs));
}

/* Tests that applying collision filters directly to the MbP via the
 ExcludeCollisionGeometriesWithCollisionFilterGroupPair() method does affect
 deformable geometries. */
TEST_F(DeformableCollisionFilterTest, ExplicitlyFilterDeformableOnPlant) {
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
  DRAKE_DEMAND(scene_graph_->model_inspector().CollisionFiltered(
      deformable_geometry_id_, welded_geometry_id_));
  DRAKE_DEMAND(scene_graph_->model_inspector().CollisionFiltered(
      deformable_geometry_id_, free_geometry_id_));

  /* Contact detection not allowed pre-Finalize. */
  plant_->Finalize();
  auto diagram = builder_.Build();

  auto diagram_context = diagram->CreateDefaultContext();
  const auto& scene_graph_context =
      scene_graph_->GetMyContextFromRoot(*diagram_context);
  const QueryObject<double>& query_object =
      scene_graph_->get_query_output_port().Eval<QueryObject<double>>(
          scene_graph_context);
  geometry::internal::DeformableContact<double> deformable_contact;
  query_object.ComputeDeformableContact(&deformable_contact);
  /* All deformable contacts are filtered by the plant collision exclusion. */
  EXPECT_EQ(deformable_contact.contact_surfaces().size(), 0);
}

/* Tests collision filter has an effect on deformable geometries if a collision
 filter is declared on the deformable geometry with the appropriate scope. */
TEST_F(DeformableCollisionFilterTest, ExplicitlyFilterDeformable) {
  /* Collision filter declaration and contact detection not allowed
   pre-Finalize. */
  plant_->Finalize();
  auto diagram = builder_.Build();

  scene_graph_->collision_filter_manager().Apply(
      CollisionFilterDeclaration(CollisionFilterScope::kAll)
          .ExcludeBetween(GeometrySet(deformable_geometry_id_),
                          GeometrySet(welded_geometry_id_)));
  auto diagram_context = diagram->CreateDefaultContext();
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
  /* Collision filter declaration and contact detection not allowed
   pre-Finalize. */
  plant_->Finalize();
  auto diagram = builder_.Build();

  scene_graph_->collision_filter_manager().Apply(
      CollisionFilterDeclaration(CollisionFilterScope::kOmitDeformable)
          .ExcludeBetween(GeometrySet(deformable_geometry_id_),
                          GeometrySet(welded_geometry_id_)));
  auto diagram_context = diagram->CreateDefaultContext();
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
