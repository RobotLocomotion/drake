#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/surface_mesh.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/hydroelastic_traction_calculator.h"
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
using multibody::HydroelasticTractionCalculator;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

namespace multibody {

namespace internal {

GeometryId FindGeometry(
    const MultibodyPlant<double>& plant, const std::string body_name) {
  const auto& geometries = plant.GetCollisionGeometriesForBody(
      plant.GetBodyByName(body_name));
  DRAKE_DEMAND(geometries.size() == 1);
  return geometries[0];
}

}  // namespace internal

class MultibodyPlantHydroelasticTractionTests : public ::testing::Test {
 public:
  const MultibodyPlant<double>& plant() const { return *plant_; }
  Context<double>& plant_context() { return *plant_context_; }
  const ContactSurface<double>& contact_surface() const {
      return *contact_surface_;
  }
  const HydroelasticTractionCalculator<double>& traction_calculator() const {
    return *traction_calculator_;
  }

 private:
  void SetUp() override {
    // Read the two bodies into the plant. The SDF file refers to a block
    // penetrating a halfspace while the geometry that we use in the tests will
    // assume a tetrahedron penetrating a halfspace. Since we don't use any
    // inertial or geometric properties from the SDF file, that is not a
    // problem.
    DiagramBuilder<double> builder;
    SceneGraph<double>* scene_graph;
    std::tie(plant_, scene_graph) = AddMultibodyPlantSceneGraph(&builder);
    MultibodyPlant<double>& plant = *plant_;
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/plant/test/block_on_halfspace.sdf");
    Parser(&plant, scene_graph).AddModelFromFile(full_name);

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

    // Instantiate the traction calculator.
    traction_calculator_ =
        std::make_unique<HydroelasticTractionCalculator<double>>(plant_);
  }

  std::unique_ptr<ContactSurface<double>> CreateContactSurface() const {
    // Get the geometry IDs for the ground halfspace and the block.
    GeometryId halfspace_geom = internal::FindGeometry(*plant_, "ground");
    GeometryId block_geom = internal::FindGeometry(*plant_, "box");

    // Create the surface mesh first.
    auto mesh = CreateSurfaceMesh();

    // Create the "e" field values (i.e., "hydroelastic pressure") using
    // negated "z" values.
    std::vector<double> e_MN(mesh->num_vertices());
    for (SurfaceVertexIndex i(0); i < mesh->num_vertices(); ++i)
      e_MN[i] = -mesh->vertex(i).r_MV()[2];

    // Create the gradient of the "h" field, pointing toward what will be
    // geometry "M" (the halfspace).
    std::vector<Vector3<double>> h_MN_M(mesh->num_vertices(),
        Vector3<double>(0, 0, -1));

    return std::make_unique<ContactSurface<double>>(
      halfspace_geom, block_geom, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh.get()),
      std::make_unique<MeshFieldLinear<Vector3<double>, SurfaceMesh<double>>>(
          "h_MN_M", std::move(h_MN_M), mesh.get()));
  }

  // Creates a surface mesh that covers the bottom of the "wetted surface",
  // where the wetted surface is the part of the tet that would be wet if the
  // halfspace were a fluid. The entire wetted surface *would* yield
  // an open, lopped-off prism with five faces but, for simplicity, we'll only
  // use the bottom face (a single triangle).
  std::unique_ptr<SurfaceMesh<double>> CreateSurfaceMesh() const {
    std::vector<SurfaceVertex<double>> vertices;
    std::vector<SurfaceFace> faces;

    // Create the vertices, all of which are offset vectors defined in the
    // halfspace body frame.
    vertices.emplace_back(Vector3<double>(0.5, 0.5, -0.5));
    vertices.emplace_back(Vector3<double>(-0.5, 0.5, -0.5));
    vertices.emplace_back(Vector3<double>(-0.5, -0.5, -0.5));

    // Create the face comprising a single triangle.
    faces.emplace_back(
        SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2));

    return std::make_unique<SurfaceMesh<double>>(
        std::move(faces), std::move(vertices));
  }

  MultibodyPlant<double>* plant_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  systems::Context<double>* plant_context_;
  std::unique_ptr<ContactSurface<double>> contact_surface_;
  std::unique_ptr<HydroelasticTractionCalculator<double>> traction_calculator_;
};

// Tests the traction calculation without any frictional or dissipation forces.
TEST_F(MultibodyPlantHydroelasticTractionTests, VanillaTraction) {
  const double dissipation = 0.0;
  const double mu_coulomb = 0.0;

  // First compute the traction.
  Vector3<double> p_W;
  const Vector3<double> traction = traction_calculator().CalcTractionAtPoint(
      plant_context(), contact_surface(), SurfaceFaceIndex(0),
      SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
      dissipation, mu_coulomb, &p_W);

  // Verify the point of contact.
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  ASSERT_NEAR(p_W[0], 0.5, eps);
  ASSERT_NEAR(p_W[1], 0.5, eps);
  ASSERT_NEAR(p_W[2], -0.5, eps);

  // Now compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Mo_W, F_No_W;
  traction_calculator().ComputeSpatialForcesAtBodyOriginsFromTraction(
      plant_context(), contact_surface(), p_W, traction, &F_Mo_W, &F_No_W);

  // Check the spatial force at p. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead. Note that the
  // tangential components are zero.
  EXPECT_NEAR(F_No_W.translational()[0], 0.0, eps);
  EXPECT_NEAR(F_No_W.translational()[1], 0.0, eps);
  EXPECT_NEAR(F_No_W.translational()[2], 0.5, eps);

  // A moment on the tet will be generated due to the normal traction. The
  // origin of the tet frame is located at (0,0,0) in the world frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (0, 0, 0.5) yields the following.
  EXPECT_NEAR(F_No_W.rotational()[0], 0.25, eps);
  EXPECT_NEAR(F_No_W.rotational()[1], -0.25, eps);
  EXPECT_NEAR(F_No_W.rotational()[2], 0, eps);

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((F_No_W.translational() + F_Mo_W.translational()).norm(),
      0, eps);
}

// Tests the traction calculation with friction but without dissipation forces.
TEST_F(MultibodyPlantHydroelasticTractionTests, TractionWithFraction) {
  const double dissipation = 0.0;
  const double mu_coulomb = 1.0;

  // Give the tet an initial (horizontal) velocity along the +x axis.
  const int num_velocities = plant().num_velocities();
  ASSERT_EQ(num_velocities, 6);
  VectorX<double> tet_velocity(num_velocities);
  tet_velocity << 0, 0, 0, 1, 0, 0;
  plant().SetVelocities(&plant_context(), tet_velocity);

    // First compute the traction.
  Vector3<double> p_W;
  const Vector3<double> traction = traction_calculator().CalcTractionAtPoint(
      plant_context(), contact_surface(), SurfaceFaceIndex(0),
      SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0), dissipation,
      mu_coulomb, &p_W);

  // Verify the point of contact.
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  ASSERT_NEAR(p_W[0], 0.5, eps);
  ASSERT_NEAR(p_W[1], 0.5, eps);
  ASSERT_NEAR(p_W[2], -0.5, eps);

  // Now compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Mo_W, F_No_W;
  traction_calculator().ComputeSpatialForcesAtBodyOriginsFromTraction(
      plant_context(), contact_surface(), p_W, traction, &F_Mo_W, &F_No_W);

  // Check the spatial force at p. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have
  // approximately the same magnitude as the normal traction.
  const double regularization_scalar =
      traction_calculator().regularization_scalar();
  EXPECT_NEAR(F_No_W.translational()[0], -0.5, regularization_scalar);
  EXPECT_NEAR(F_No_W.translational()[1], 0.0, eps);
  EXPECT_NEAR(F_No_W.translational()[2], 0.5, eps);

  // A moment on the tet will be generated due to the traction. The
  // origin of the tet frame is located at (0,0,0) in the world frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (-.5, 0, 0.5) yields the following.
  EXPECT_NEAR(F_No_W.rotational()[0], 0.25, eps);
  EXPECT_NEAR(F_No_W.rotational()[1], 0.0, regularization_scalar);
  EXPECT_NEAR(F_No_W.rotational()[2], 0.25, regularization_scalar);

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((F_No_W.translational() + F_Mo_W.translational()).norm(),
      0, eps);
}

// Tests the traction calculation with dissipation forces but without friction.
TEST_F(MultibodyPlantHydroelasticTractionTests, TractionWithDissipation) {
  const double dissipation = 1.0;
  const double mu_coulomb = 0.0;

  // Give the tet an initial (vertical) velocity along the -z axis.
  const int num_velocities = plant().num_velocities();
  ASSERT_EQ(num_velocities, 6);
  VectorX<double> tet_velocity(num_velocities);
  const double separating_velocity = -1.0;
  tet_velocity << 0, 0, 0, 0, 0, separating_velocity;
  plant().SetVelocities(&plant_context(), tet_velocity);

  // Compute the magnitude of the normal traction. Note that the damping
  // constant at each point will be field value * dissipation coefficient.
  const double field_value = 0.5;
  const double c = field_value * dissipation;
  const double damping_traction_magnitude = c * -separating_velocity;
  const double normal_traction_magnitude = field_value +
      damping_traction_magnitude;

  // First compute the traction.
  Vector3<double> p_W;
  const Vector3<double> traction = traction_calculator().CalcTractionAtPoint(
      plant_context(), contact_surface(), SurfaceFaceIndex(0),
      SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
      dissipation, mu_coulomb, &p_W);

  // Verify the point of contact.
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  ASSERT_NEAR(p_W[0], 0.5, eps);
  ASSERT_NEAR(p_W[1], 0.5, eps);
  ASSERT_NEAR(p_W[2], -0.5, eps);

  // Now compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Mo_W, F_No_W;
  traction_calculator().ComputeSpatialForcesAtBodyOriginsFromTraction(
      plant_context(), contact_surface(), p_W, traction, &F_Mo_W, &F_No_W);

  // Check the spatial force at p. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have the same
  // magnitude as the normal traction.
  EXPECT_NEAR(F_No_W.translational()[0], 0.0, eps);
  EXPECT_NEAR(F_No_W.translational()[1], 0.0, eps);
  EXPECT_NEAR(F_No_W.translational()[2], normal_traction_magnitude, eps);

  // A moment on the tet will be generated due to the traction. The
  // origin of the tet frame is located at (0,0,0) in the world frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (0, 0, normal_traction_magnitude) yields
  // the following.
  EXPECT_NEAR(F_No_W.rotational()[0], 0.5 * normal_traction_magnitude, eps);
  EXPECT_NEAR(F_No_W.rotational()[1], -0.5 * normal_traction_magnitude, eps);
  EXPECT_NEAR(F_No_W.rotational()[2], 0.0, eps);

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((F_No_W.translational() + F_Mo_W.translational()).norm(),
      0, eps);
}

}  // namespace multibody
}  // namespace drake
