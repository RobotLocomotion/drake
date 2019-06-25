#include "drake/multibody/hydroelastics/hydroelastic_engine.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::ContactSurface;
using drake::geometry::SurfaceMesh;
using drake::math::RigidTransformd;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {
namespace {

class SphereVsPlaneTest : public ::testing::Test {
 public:
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    // Make a non-finalized plant so that we can tests methods with pre/post
    // Finalize() conditions.
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/hydroelastics/test/sphere_vs_plane.sdf");
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder);
    Parser(plant_).AddModelFromFile(full_name);

    // Retrieve bodies and geometry ids.
    sphere_ = &plant_->GetBodyByName("sphere");
    ground_ = &plant_->GetBodyByName("ground");
    ASSERT_EQ(plant_->GetCollisionGeometriesForBody(*sphere_).size(), 1u);
    sphere_geometry_id_ = plant_->GetCollisionGeometriesForBody(*sphere_)[0];
    ASSERT_EQ(plant_->GetCollisionGeometriesForBody(*ground_).size(), 1u);
    ground_geometry_id_ = plant_->GetCollisionGeometriesForBody(*ground_)[0];

    // Set elastic properties pre-finalize.
    const double elastic_modulus = 10.0;
    plant_->set_elastic_modulus(sphere_geometry_id_, elastic_modulus);

    plant_->Finalize();
    diagram_ = builder.Build();
    // Sanity check on the availability of the optional source id before using
    // it.
    DRAKE_DEMAND(plant_->get_source_id() != nullopt);

    MakeNewContext();

    engine_ = std::make_unique<HydroelasticEngine<double>>();

    SetInContactConfiguration();
  }

  // Sets a configuration for which there is contact between the sphere and the
  // ground. The 5 cm sphere is placed with its center 4 cm above the ground and
  // thefore its lowest point is 1 cm below the ground.
  void SetInContactConfiguration() {
    const RigidTransformd X_WS = Eigen::Translation3d(0.0, 0.0, height_);
    plant_->SetFreeBodyPoseInWorldFrame(plant_context_, *sphere_, X_WS);
  }

  void MakeNewContext() {
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, context_.get());

    const auto& query_port = plant_->get_geometry_query_input_port();
    query_object_ = &query_port.template Eval<geometry::QueryObject<double>>(
        *plant_context_);
  }

 protected:
  MultibodyPlant<double>* plant_{nullptr};
  geometry::SceneGraph<double>* scene_graph_{nullptr};
  // The diagram owning the plant and scene graph.
  std::unique_ptr<systems::Diagram<double>> diagram_;
  const Body<double>* sphere_{nullptr};
  const Body<double>* ground_{nullptr};
  geometry::GeometryId sphere_geometry_id_, ground_geometry_id_;
  std::unique_ptr<systems::Context<double>> context_;
  systems::Context<double>* plant_context_{nullptr};
  const geometry::QueryObject<double>* query_object_{nullptr};
  std::unique_ptr<HydroelasticEngine<double>> engine_;
  // Numerical parameters of the problem.
  double height_{0.04};
  double radius_{0.05};  // consistent with the *.sdf file.
};

TEST_F(SphereVsPlaneTest, RespectsCollisionFilter) {
  // Before filtering is applied the engine should report contact.
  EXPECT_EQ(engine_->ComputeContactSurfaces(*query_object_).size(), 1u);

  // Add filter to exclude collisions between the ground and the sphere.
  optional<geometry::FrameId> ground_id =
      plant_->GetBodyFrameIdIfExists(ground_->index());
  ASSERT_TRUE(ground_id.has_value());
  optional<geometry::FrameId> sphere_id =
      plant_->GetBodyFrameIdIfExists(sphere_->index());
  ASSERT_TRUE(sphere_id.has_value());
  scene_graph_->ExcludeCollisionsBetween(context_.get(),
                                         geometry::GeometrySet(*ground_id),
                                         geometry::GeometrySet(*sphere_id));

  // Set objects to be in contact.
  SetInContactConfiguration();

  EXPECT_EQ(engine_->ComputeContactSurfaces(*query_object_).size(), 0u);
}

TEST_F(SphereVsPlaneTest, VerifyModelSizeAndResults) {
  EXPECT_EQ(plant_->num_bodies(), 4);  // Including the "world" body.

  // Models are created not until the first query.
  EXPECT_EQ(engine_->num_models(), 0);

  // First query will create the underlying hydroelastic models including
  // tetrahedral meshes, fields and level sets.
  const std::vector<ContactSurface<double>> all_surfaces =
      engine_->ComputeContactSurfaces(*query_object_);

  // We expect three collision geomtries from the SDF: a plane for the ground, a
  // box attached to the ground, and a sphere.
  EXPECT_EQ(scene_graph_->model_inspector().num_geometries(), 3);

  // Expect a model for the sphere and for the half-space. Currently, the box is
  // ignored.
  EXPECT_EQ(engine_->num_models(), 2);

  ASSERT_EQ(all_surfaces.size(), 1u);
  const ContactSurface<double>& surface = all_surfaces[0];
  // HydroelasticEngine always makes M to be the id for the rigid geometry and
  // N the id for the soft geometry.
  EXPECT_EQ(surface.id_M(), ground_geometry_id_);
  EXPECT_EQ(surface.id_N(), sphere_geometry_id_);

  // Verify that the sphere is soft and the ground is rigid.
  EXPECT_TRUE(engine_->get_model(sphere_geometry_id_).is_soft());
  EXPECT_FALSE(engine_->get_model(ground_geometry_id_).is_soft());

  // This is merely a sanity check on the returned results. The contact
  // surface calculation implemented in ContactSurfaceCalculator, already has
  // thorough unit tests.
  // Since id_M corresponds to the ground geometry, the mesh is expressed in
  // frame G of the ground.
  const SurfaceMesh<double>& mesh_G = surface.mesh();
  EXPECT_GT(mesh_G.num_vertices(), 0);
  const double kTolerance = 5.0 * std::numeric_limits<double>::epsilon();
  for (geometry::SurfaceVertexIndex v(0); v < mesh_G.num_vertices(); ++v) {
    // Position of a vertex V in the frame S of the soft sphere.
    const Vector3d p_SV = mesh_G.vertex(v).r_MV();

    // We verify that the positions were correctly interpolated to lie on the
    // plane.
    EXPECT_NEAR(p_SV[2], 0.0, kTolerance);

    // Verify surface vertices lie within a circle of the expected radius.
    const double surface_radius =
        std::sqrt(radius_ * radius_ - height_ * height_);
    const double radius = p_SV.norm();  // since z component is zero.
    EXPECT_LE(radius, surface_radius);
  }

  // The number of models should not change on further queries.
  engine_->ComputeContactSurfaces(*query_object_);
  EXPECT_EQ(engine_->num_models(), 2);
}

}  // namespace
}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
