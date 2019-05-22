#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/surface_mesh.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::MeshFieldLinear;
using geometry::SceneGraph;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceFace;
using geometry::SurfaceMesh;
using geometry::SurfaceVertex;
using geometry::SurfaceVertexIndex;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::Parser;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

namespace multibody {

// A friend class to MultibodyPlant.
class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static VectorX<double> CalcGeneralizedForceFromTractionAtPoint(
    const MultibodyPlant<double>& plant,
    const Context<double>& context,
    GeometryId geometryM_id, GeometryId geometryN_id,
    const ContactSurface<double>& surface,
    SurfaceFaceIndex face_index,
    const SurfaceMesh<double>::Barycentric& r_barycentric,
    double dissipation, double mu_coulomb) {
        return plant.CalcGeneralizedForceFromTractionAtPoint(
            context, geometryM_id, geometryN_id, surface, face_index,
            r_barycentric, dissipation, mu_coulomb);
    }


  static GeometryId FindGeometry(
      const MultibodyPlant<double>& plant, const std::string body_name) {
    const auto& geometries = plant.GetCollisionGeometriesForBody(
        plant.GetBodyByName(body_name));
    DRAKE_DEMAND(geometries.size() == 1);
    return geometries[0];
  }
};

class MultibodyPlantHydroelasticTractionTests : public ::testing::Test {
 public:
  const MultibodyPlant<double>& plant() const { return *plant_; }
  Context<double>& plant_context() { return *plant_context_; }
  const ContactSurface<double>& contact_surface() const {
      return *contact_surface_;
  }

  VectorX<double> CalcGeneralizedForceFromTractionAtPoint(
      SurfaceFaceIndex face_index,
      const SurfaceMesh<double>::Barycentric& r_barycentric,
      double dissipation, double mu_coulomb) {
    return MultibodyPlantTester::CalcGeneralizedForceFromTractionAtPoint(
          plant(), plant_context(),
          MultibodyPlantTester::FindGeometry(plant(), "ground"),
          MultibodyPlantTester::FindGeometry(plant(), "box"),
          contact_surface(), face_index, r_barycentric,
          dissipation, mu_coulomb);
  }

 private:
  void SetUp() override {
    // Read the two bodies into the plant.
    DiagramBuilder<double> builder;
    SceneGraph<double>* scene_graph;
    std::tie(plant_, scene_graph) = AddMultibodyPlantSceneGraph(&builder);
    MultibodyPlant<double>& plant = *plant_;
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/plant/test/block_on_halfspace.sdf");
    Parser(&plant, scene_graph).AddModelFromFile(full_name);

    // Add gravity to the model.
    plant.AddForceElement<UniformGravityFieldElement>();

    plant.Finalize();
    diagram_ = builder.Build();

    // Create a context for this system.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(plant, context_.get());

    contact_surface_ = CreateContactSurface();

    // Set the pose and velocity for the box.
    ASSERT_EQ(plant.num_velocities(), 6);
    ASSERT_EQ(plant.num_positions(), 7);
    VectorX<double> state(plant.num_positions() + plant.num_velocities());
    state << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    auto& state_vector = plant_context_->get_mutable_continuous_state_vector();
    state_vector.SetFromVector(state);
  }

  std::unique_ptr<ContactSurface<double>> CreateContactSurface() const {
    // Get the geometry IDs for the ground halfspace and the block.
    GeometryId ground_geom = MultibodyPlantTester::FindGeometry(
        *plant_, "ground");
    GeometryId block_geom = MultibodyPlantTester::FindGeometry(*plant_, "box");

    // Create the surface mesh first.
    auto mesh = CreateSurfaceMesh();

    // Create the "e" field values using negated "z" values.
    std::vector<double> e_MN(mesh->num_vertices());
    for (SurfaceVertexIndex i(0); i < mesh->num_vertices(); ++i)
      e_MN[i] = -mesh->vertex(i).r_MV()[2];

    // Create the gradient of the "h" field, pointing toward what will be
    // geometry "M" (the halfspace).
    std::vector<Vector3<double>> h_MN_M(mesh->num_vertices(),
        Vector3<double>(0, 0, -1));

    return std::make_unique<ContactSurface<double>>(
      ground_geom, block_geom, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh.get()),
      std::make_unique<MeshFieldLinear<Vector3<double>, SurfaceMesh<double>>>(
          "h_MN_M", std::move(h_MN_M), mesh.get()));
  }

  // Creates a surface mesh that covers the bottom of the "wetted surface",
  // where the wetted surface is the part of the box that would be wet if the
  // halfspace were a fluid. The entire wetted surface *would* yield
  // an open box with five faces comprising ten triangles but, for
  // simplicity, we'll only use the bottom face (two triangles).
  std::unique_ptr<SurfaceMesh<double>> CreateSurfaceMesh() const {
    std::vector<SurfaceVertex<double>> vertices;
    std::vector<SurfaceFace> faces;

    // Create the vertices, all of which are offset vectors defined in the
    // ground body frame.
    vertices.emplace_back(Vector3<double>(0.5, 0.5, -0.5));
    vertices.emplace_back(Vector3<double>(-0.5, 0.5, -0.5));
    vertices.emplace_back(Vector3<double>(-0.5, -0.5, -0.5));
    vertices.emplace_back(Vector3<double>(0.5, -0.5, -0.5));

    // Create the face comprising two triangles.
    faces.emplace_back(
        SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2));
    faces.emplace_back(
        SurfaceVertexIndex(2), SurfaceVertexIndex(3), SurfaceVertexIndex(0));

    return std::make_unique<SurfaceMesh<double>>(
        std::move(faces), std::move(vertices));
  }

  MultibodyPlant<double>* plant_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  systems::Context<double>* plant_context_;
  std::unique_ptr<ContactSurface<double>> contact_surface_;
};

// Tests the traction calculation without any frictional or dissipation forces.
TEST_F(MultibodyPlantHydroelasticTractionTests, VanillaTraction) {
  const double dissipation = 0.0;
  const double mu_coulomb = 0.0;

  // Compute traction vectors at two opposing corners of the box.
  // Point in the world frame: .5, .5, -.5
  VectorX<double> t1 = CalcGeneralizedForceFromTractionAtPoint(
      SurfaceFaceIndex(0),
          SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
          dissipation, mu_coulomb);
  // Point in the world frame: -.5, -.5, -.5
  VectorX<double> t2 = CalcGeneralizedForceFromTractionAtPoint(
      SurfaceFaceIndex(1),
          SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
          dissipation, mu_coulomb);

  // Sum the tractions together. We know that the field has a value of 0.5 at
  // the two corners, so the total normal traction has unit magnitude.
  // Since the field values are equal and there are no tangential tractions
  // (or forces), we expect there to be no moment.
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  VectorX<double> traction = t1 + t2;
  EXPECT_NEAR(traction.segment(0, 5).norm(), 0.0, eps);
  EXPECT_NEAR(traction[5], 1.0, eps);
}

// Tests the traction calculation with friction but without dissipation forces.
TEST_F(MultibodyPlantHydroelasticTractionTests, TractionWithFraction) {
  const double dissipation = 0.0;
  const double mu_coulomb = 1.0;

  // Give the box an initial (horizontal) velocity along the +x axis.
  const int num_velocities = plant().num_velocities();
  ASSERT_EQ(num_velocities, 6);
  VectorX<double> box_velocity(num_velocities);
  box_velocity << 0, 0, 0, 1, 0, 0;
  plant().SetVelocities(&plant_context(), box_velocity);

  // Compute traction vectors at two opposing corners of the box.
  // Point in the world frame: .5, .5, -.5
  VectorX<double> t1 = CalcGeneralizedForceFromTractionAtPoint(
      SurfaceFaceIndex(0),
          SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
          dissipation, mu_coulomb);
  // Point in the world frame: -.5, -.5, -.5
  VectorX<double> t2 = CalcGeneralizedForceFromTractionAtPoint(
      SurfaceFaceIndex(1),
          SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
          dissipation, mu_coulomb);

  // Sum the tractions together. We know that the field has a value of 0.5 at
  // the two corners, so the total normal traction has unit magnitude. The
  // coefficient of friction is unity, so the total frictional traction will
  // have unit magnitude as well.
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  VectorX<double> traction = t1 + t2;
  EXPECT_NEAR(traction[3], -1.0, eps);
  EXPECT_NEAR(traction[4], 0.0, eps);
  EXPECT_NEAR(traction[5], 1.0, eps);

  // A moment on the box will be generated due to the frictional tractions. The
  // center-of-mass of the box will be located at (0,0,0) in the world frame.
  // The moment arm at the first point will be (.5,.5,-.5). Crossing this vector
  // with the traction at that point (-.5,0,.5) yields the following.
  EXPECT_NEAR(t1[0], 0.25, eps);
  EXPECT_NEAR(t1[1], 0.0, eps);
  EXPECT_NEAR(t1[2], 0.25, eps);

  // The moment arm at the first point will be (-.5,-.5,-.5). Crossing this
  // vector with the traction at that point (-.5,0,.5) yields the following.
  EXPECT_NEAR(t2[0], -0.25, eps);
  EXPECT_NEAR(t2[1], 0.5, eps);
  EXPECT_NEAR(t2[2], -0.25, eps);
}

// Tests the traction calculation with dissipation forces but without friction.
TEST_F(MultibodyPlantHydroelasticTractionTests, TractionWithDissipation) {
  const double dissipation = 1.0;
  const double mu_coulomb = 0.0;

  // Give the box an initial vertical velocity along the -z axis.
  const int num_velocities = plant().num_velocities();
  ASSERT_EQ(num_velocities, 6);
  VectorX<double> box_velocity(num_velocities);
  const double separating_velocity = -1.0;
  box_velocity << 0, 0, 0, 0, 0, separating_velocity;
  plant().SetVelocities(&plant_context(), box_velocity);

  // Compute traction vectors at two opposing corners of the box.
  // Point in the world frame: .5, .5, -.5
  VectorX<double> t1 = CalcGeneralizedForceFromTractionAtPoint(
      SurfaceFaceIndex(0),
          SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
          dissipation, mu_coulomb);
  // Point in the world frame: -.5, -.5, -.5
  VectorX<double> t2 = CalcGeneralizedForceFromTractionAtPoint(
      SurfaceFaceIndex(1),
          SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
          dissipation, mu_coulomb);

  // The damping constant at each point will be 1.5 * field value * dissipation
  // coefficient.
  const double field_value = 0.5;
  const double c = 1.5 * field_value * dissipation;
  const double damping_traction_per_point = c * -separating_velocity;
  const double normal_traction_per_point = field_value +
      damping_traction_per_point;

  const double eps = 10 * std::numeric_limits<double>::epsilon();
  VectorX<double> traction = t1 + t2;
  EXPECT_NEAR(traction.segment(0, 5).norm(), 0.0, eps);
  const int num_points = 2;
  EXPECT_NEAR(traction[5], normal_traction_per_point * num_points, eps);
}

}  // namespace multibody
}  // namespace drake
