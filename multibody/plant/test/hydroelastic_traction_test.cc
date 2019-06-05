#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
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

class MultibodyPlantHydroelasticTractionTests :
public ::testing::TestWithParam<math::RigidTransform<double>> {
 public:
  const MultibodyPlant<double>& plant() const { return *plant_; }
  Context<double>& plant_context() { return *plant_context_; }
  const ContactSurface<double>& contact_surface() const {
      return *contact_surface_;
  }
  const HydroelasticTractionCalculator<double>& traction_calculator() const {
    return *traction_calculator_;
  }
  const HydroelasticTractionCalculatorData<double>& traction_calculator_data()
      const {
    return *traction_calculator_data_;
  }
  math::RigidTransform<double> GetGeometryTransformationToWorld(
      GeometryId g) const {
    return traction_calculator_data_->GetGeometryTransformationToWorld(
        *plant_context_, g);
  }

  // Returns the default numerical tolerance.
  double eps() const { return eps_; }

  // Computes the spatial forces due to the hydroelastic model.
  void ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      double dissipation, double mu_coulomb,
      SpatialForce<double>* F_Mo_W, SpatialForce<double>* F_No_W) {
    // Instantiate the traction calculator data.
    traction_calculator_data_ =
        std::make_unique<HydroelasticTractionCalculatorData<double>>(
            *plant_context_, *contact_surface_, plant());

    // First compute the traction, expressed in the world frame.
    Vector3<double> p_WQ;
    const Vector3<double> traction_Q_W = traction_calculator().
        CalcTractionAtPoint(
            traction_calculator_data(), SurfaceFaceIndex(0),
            SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
            dissipation, mu_coulomb, &p_WQ);

    // Compute the expected point of contact in the world frame. Recall that
    // the parameter to this test transforms both bodies to a pose Y in the
    // world.
    const Vector3<double> p_WQ_expected = GetParam() *
        Vector3<double>(0.5, 0.5, -0.5);

    // Verify the point of contact.
    for (int i = 0; i < 3; ++i)
      ASSERT_NEAR(p_WQ[i], p_WQ_expected[i], eps());

    traction_calculator().ComputeSpatialForcesAtBodyOriginsFromTraction(
        traction_calculator_data(), p_WQ, traction_Q_W, F_Mo_W, F_No_W);
  }

  void SetBoxTranslationalVelocity(const Vector3<double>& v) {
    plant_->SetFreeBodySpatialVelocity(plant_context_,
        plant_->GetBodyByName("box"),
        SpatialVelocity<double>(Vector3<double>::Zero(), v));
  }

 private:
  void SetUp() override {
    // Read the two bodies into the plant. The SDF file refers to a rigid block
    // penetrating a compliant halfspace while the geometry that we use in the
    // tests will assume a tetrahedron penetrating a halfspace. Since we don't
    // use any inertial or geometric properties from the SDF file, that is not
    // a problem.
    DiagramBuilder<double> builder;
    SceneGraph<double>* scene_graph;
    std::tie(plant_, scene_graph) = AddMultibodyPlantSceneGraph(&builder);
    MultibodyPlant<double>& plant = *plant_;
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/plant/test/block_on_halfspace.sdf");
    Parser(&plant, scene_graph).AddModelFromFile(full_name);

    plant.Finalize();
    diagram_ = builder.Build();

    ASSERT_EQ(plant.num_velocities(), 12);
    ASSERT_EQ(plant.num_positions(), 14);

    // Create a context for this system.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(plant, context_.get());

    contact_surface_ = CreateContactSurface();

    // Set the poses for the two bodies.
    plant.SetFreeBodyPose(plant_context_,
        plant.GetBodyByName("box"), GetParam());
    plant.SetFreeBodyPose(plant_context_,
        plant.GetBodyByName("ground"), GetParam());

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

  const double eps_{10 * std::numeric_limits<double>::epsilon()};
  MultibodyPlant<double>* plant_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  systems::Context<double>* plant_context_;
  std::unique_ptr<ContactSurface<double>> contact_surface_;
  std::unique_ptr<HydroelasticTractionCalculator<double>> traction_calculator_;
  std::unique_ptr<HydroelasticTractionCalculatorData<double>>
      traction_calculator_data_;
};

// Tests the traction calculation without any frictional or dissipation forces.
TEST_P(MultibodyPlantHydroelasticTractionTests, VanillaTraction) {
  const double dissipation = 0.0;
  const double mu_coulomb = 0.0;

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Mo_W, F_No_W;
  ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &F_Mo_W, &F_No_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const Vector3<double> f_No_Y = R_WY.transpose() * F_No_W.translational();
  const Vector3<double> tau_No_Y = R_WY.transpose() * F_No_W.rotational();

  // Check the spatial force at p. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead. Note that the
  // tangential components are zero.
  EXPECT_NEAR(f_No_Y[0], 0.0, eps());
  EXPECT_NEAR(f_No_Y[1], 0.0, eps());
  EXPECT_NEAR(f_No_Y[2], 0.5, eps());

  // A moment on the tet will be generated due to the normal traction. The
  // origin of the tet frame is located at (0,0,0) in the Y frame.
  // The moment arm at the point will be (.5, .5, -.5), again in the Y frame.
  // Crossing this vector with the traction at that point (0, 0, 0.5) yields the
  // following.
  EXPECT_NEAR(tau_No_Y[0], 0.25, eps());
  EXPECT_NEAR(tau_No_Y[1], -0.25, eps());
  EXPECT_NEAR(tau_No_Y[2], 0, eps());

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((F_No_W.translational() + F_Mo_W.translational()).norm(),
      0, eps());
}

// Tests the traction calculation with friction but without dissipation forces.
TEST_P(MultibodyPlantHydroelasticTractionTests, TractionWithFraction) {
  const double dissipation = 0.0;
  const double mu_coulomb = 1.0;

  // Give the tet an initial (vertical) velocity along the +x axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  SetBoxTranslationalVelocity(R_WY * Vector3<double>(1, 0, 0));

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Mo_W, F_No_W;
  ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &F_Mo_W, &F_No_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const Vector3<double> f_No_Y = R_WY.transpose() * F_No_W.translational();
  const Vector3<double> tau_No_Y = R_WY.transpose() * F_No_W.rotational();

  // Check the spatial force at p. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have
  // approximately the same magnitude as the normal traction. Note that the
  // regularized Coulomb friction model requires backing off of the tolerance
  // for comparing frictional force components (against the Coulomb model); the
  // units are dissimilar (m/s vs. N and Nm) but using this as the tolerance
  // has proven useful.
  const double regularization_scalar =
      traction_calculator().regularization_scalar();
  EXPECT_NEAR(f_No_Y[0], -0.5, regularization_scalar);
  EXPECT_NEAR(f_No_Y[1], 0.0, eps());
  EXPECT_NEAR(f_No_Y[2], 0.5, eps());

  // A moment on the tet will be generated due to the traction. The
  // origin of the tet frame is located at (0,0,0) in the Y frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (-.5, 0, 0.5) yields the following.
  EXPECT_NEAR(tau_No_Y[0], 0.25, eps());
  EXPECT_NEAR(tau_No_Y[1], 0.0, regularization_scalar);
  EXPECT_NEAR(tau_No_Y[2], 0.25, regularization_scalar);

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((F_No_W.translational() + F_Mo_W.translational()).norm(),
      0, eps());
}

// Tests the traction calculation with dissipation forces but without friction.
TEST_P(MultibodyPlantHydroelasticTractionTests, TractionWithDissipation) {
  const double dissipation = 1.0;
  const double mu_coulomb = 0.0;

  // Give the tet an initial (vertical) velocity along the -z axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const double separating_velocity = -1.0;
  SetBoxTranslationalVelocity(R_WY *
      Vector3<double>(0, 0, separating_velocity));

  // Compute the magnitude of the normal traction. Note that the damping
  // constant at each point will be field value * dissipation coefficient.
  const double field_value = 0.5;
  const double c = field_value * dissipation;
  const double damping_traction_magnitude = c * -separating_velocity;
  const double normal_traction_magnitude = field_value +
      damping_traction_magnitude;

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Mo_W, F_No_W;
  ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &F_Mo_W, &F_No_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const Vector3<double> f_No_Y = R_WY.transpose() * F_No_W.translational();
  const Vector3<double> tau_No_Y = R_WY.transpose() * F_No_W.rotational();

  // Check the spatial force at p. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have the same
  // magnitude as the normal traction.
  EXPECT_NEAR(f_No_Y[0], 0.0, eps());
  EXPECT_NEAR(f_No_Y[1], 0.0, eps());
  EXPECT_NEAR(f_No_Y[2], normal_traction_magnitude, eps());

  // A moment on the tet will be generated due to the traction. The
  // origin of the tet frame is located at (0,0,0) in the world frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (0, 0, normal_traction_magnitude) yields
  // the following.
  EXPECT_NEAR(tau_No_Y[0], 0.5 * normal_traction_magnitude, eps());
  EXPECT_NEAR(tau_No_Y[1], -0.5 * normal_traction_magnitude, eps());
  EXPECT_NEAR(tau_No_Y[2], 0.0, eps());

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((F_No_W.translational() + F_Mo_W.translational()).norm(),
      0, eps());
}

const math::RigidTransform<double> poses[] = {
    math::RigidTransform<double>::Identity(),
    math::RigidTransform<double>(
        math::RotationMatrix<double>::MakeYRotation(M_PI_4),
        drake::Vector3<double>(1, 2, 3))
};

INSTANTIATE_TEST_CASE_P(PoseInstantiations,
                        MultibodyPlantHydroelasticTractionTests,
                        ::testing::ValuesIn(poses));

}  // namespace multibody
}  // namespace drake

