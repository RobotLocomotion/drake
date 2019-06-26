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
using math::RigidTransform;
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

// This fixture defines a contacting configuration between a box and a
// half-space in a local frame, Y. In Frame Y, half-space Frame H is
// positioned such that it passes through the origin and its normal Hz is
// aligned with Yz. In other words, the pose X_YH is the identity pose.
// The box pose X_YB is also set to be the identity pose and therefore the
// geometric center of the box is at the origin Yo, with the bottom (in frame Y)
// half of the box overlapping the half-space. The fixture unit-tests
// an arbitrary set of poses X_WY of frame Y in the world frame W to assess the
// frame invariance of the computed results, which only depend (modulo the
// "expressed-in" frame) on the relative pose of the bodies.
class MultibodyPlantHydroelasticTractionTests :
public ::testing::TestWithParam<RigidTransform<double>> {
 public:
  const HydroelasticTractionCalculator<double>& traction_calculator() const {
    return traction_calculator_;
  }
  const MultibodyPlant<double>& plant() const { return *plant_; }
  const ContactSurface<double>& contact_surface() const {
      return *contact_surface_;
  }
  const Context<double>& plant_context() const { return *plant_context_; }

  // Returns the default numerical tolerance.
  double tol() const { return tol_; }

  // Computes the spatial tractions due to the hydroelastic model. This computes
  // the spatial traction (in units of N/m²) at Vertex 0 of Triangle 0. We're
  // using the SpatialForce data type to hold these tractions, but they are
  // not forces. We'll use "Ft" rather than "F" here so we remember these are
  // tractions.
  void ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      double dissipation, double mu_coulomb, SpatialForce<double>* Ft_Ao_W,
      SpatialForce<double>* Ft_Bo_W) {
    // Instantiate the traction calculator data.
    HydroelasticTractionCalculator<double>::HydroelasticTractionCalculatorData
        calculator_data(*plant_context_, *plant_, contact_surface_.get());

    // First compute the traction applied to Body A at point Q, expressed in the
    // world frame.
    Vector3<double> p_WQ;
    const Vector3<double> traction_Aq_W =
        traction_calculator().CalcTractionAtPoint(
            calculator_data, SurfaceFaceIndex(0),
            SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0), dissipation,
            mu_coulomb, &p_WQ);

    // Compute the expected point of contact in the world frame. The class
    // definition and SetUp() note that the parameter to this test transforms
    // both bodies from their definition in Frame Y to the world frame.
    const RigidTransform<double>& X_WY = GetParam();
    const Vector3<double> p_YQ(0.5, 0.5, -0.5);
    const Vector3<double> p_WQ_expected = X_WY * p_YQ;

    // Verify the point of contact.
    for (int i = 0; i < 3; ++i) ASSERT_NEAR(p_WQ[i], p_WQ_expected[i], tol());

    const SpatialForce<double> Ft_Ac_W =
        traction_calculator().ComputeSpatialTractionAtAcFromTractionAtAq(
            calculator_data, p_WQ, traction_Aq_W);

    // Shift to body origins. Traction on body B is equal and opposite.
    const Vector3<double>& p_WC = calculator_data.p_WC();
    const Vector3<double>& p_WAo = calculator_data.X_WA().translation();
    const Vector3<double>& p_WBo = calculator_data.X_WB().translation();
    const Vector3<double> p_CAo_W = p_WAo - p_WC;
    const Vector3<double> p_CBo_W = p_WBo - p_WC;
    *Ft_Ao_W = Ft_Ac_W.Shift(p_CAo_W);
    *Ft_Bo_W = -(Ft_Ac_W.Shift(p_CBo_W));
  }

  void SetBoxTranslationalVelocity(const Vector3<double>& v) {
    plant_->SetFreeBodySpatialVelocity(plant_context_,
        plant_->GetBodyByName("box"),
        SpatialVelocity<double>(Vector3<double>::Zero(), v));
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

    plant.Finalize();
    diagram_ = builder.Build();

    ASSERT_EQ(plant.num_velocities(), 12);
    ASSERT_EQ(plant.num_positions(), 14);

    // Create a context for this system.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(plant, context_.get());

    contact_surface_ = CreateContactSurface();

    // See class documentation for description of Frames Y, B, and H.
    const RigidTransform<double>& X_WY = GetParam();
    const auto& X_YH = RigidTransform<double>::Identity();
    const auto& X_YB = RigidTransform<double>::Identity();
    const RigidTransform<double> X_WH = X_WY * X_YH;
    const RigidTransform<double> X_WB = X_WY * X_YB;
    plant.SetFreeBodyPose(plant_context_, plant.GetBodyByName("ground"), X_WH);
    plant.SetFreeBodyPose(plant_context_, plant.GetBodyByName("box"), X_WB);
  }

  std::unique_ptr<ContactSurface<double>> CreateContactSurface() const {
    // Get the geometry IDs for the ground halfspace and the block.
    GeometryId halfspace_id = internal::FindGeometry(*plant_, "ground");
    GeometryId block_id = internal::FindGeometry(*plant_, "box");

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

    SurfaceMesh<double>* mesh_pointer = mesh.get();
    return std::make_unique<ContactSurface<double>>(
      halfspace_id, block_id, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh_pointer),
      std::make_unique<MeshFieldLinear<Vector3<double>, SurfaceMesh<double>>>(
          "h_MN_M", std::move(h_MN_M), mesh_pointer));
  }

  // Creates a surface mesh that covers the bottom of the "wetted surface",
  // where the wetted surface is the part of the box that would be wet if the
  // halfspace were a fluid. The entire wetted surface *would* yield
  // an open box with five faces but, for simplicity, we'll only
  // use the bottom face (two triangles).
  std::unique_ptr<SurfaceMesh<double>> CreateSurfaceMesh() const {
    std::vector<SurfaceVertex<double>> vertices;
    std::vector<SurfaceFace> faces;

    // Create the vertices, all of which are offset vectors defined in the
    // halfspace body frame.
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

  const double tol_{10 * std::numeric_limits<double>::epsilon()};
  MultibodyPlant<double>* plant_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  systems::Context<double>* plant_context_;
  std::unique_ptr<ContactSurface<double>> contact_surface_;
  HydroelasticTractionCalculator<double> traction_calculator_;
};

// Tests the traction calculation without any frictional or dissipation
// tractions.
TEST_P(MultibodyPlantHydroelasticTractionTests, VanillaTraction) {
  const double dissipation = 0.0;  // Units: s/m.
  const double mu_coulomb = 0.0;

  // Compute the spatial tractions at the origins of the body frames.
  multibody::SpatialForce<double> Ft_Ao_W, Ft_Bo_W;
  ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &Ft_Ao_W, &Ft_Bo_W);

  // Re-express the spatial tractions in Y's frame for easy interpretability.
  // Note that f and tau here are still tractions, so our notation is being
  // misused a little here.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const Vector3<double> f_Bo_Y = R_WY.transpose() * Ft_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * Ft_Bo_W.rotational();

  // Check the spatial traction at p. We know that geometry M is the halfspace,
  // so we'll check the spatial traction for geometry N instead. Note that the
  // tangential components are zero.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], 0.5, tol());

  // A moment on the box will be generated due to the normal traction. The
  // origin of the box frame is located at (0,0,0) in the Y frame.
  // The moment arm at the point will be (.5, .5, -.5), again in the Y frame.
  // Crossing this vector with the traction at that point (0, 0, 0.5) yields the
  // following.
  EXPECT_NEAR(tau_Bo_Y[0], 0.25, tol());
  EXPECT_NEAR(tau_Bo_Y[1], -0.25, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0, tol());

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((Ft_Bo_W.translational() + Ft_Ao_W.translational()).norm(),
      0, tol());
}

// Tests the traction calculation with friction but without dissipation
// tractions.
TEST_P(MultibodyPlantHydroelasticTractionTests, TractionWithFriction) {
  const double dissipation = 0.0;  // Units: s/m.
  const double mu_coulomb = 1.0;

  // Give the box an initial (vertical) velocity along the +x axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  SetBoxTranslationalVelocity(R_WY * Vector3<double>(1, 0, 0));

  // Compute the spatial tractions at the origins of the body frames.
  multibody::SpatialForce<double> Ft_Ao_W, Ft_Bo_W;
  ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &Ft_Ao_W, &Ft_Bo_W);

  // Re-express the spatial tractions in Y's frame for easy interpretability.
  // Note that f and tau here are still tractions, so our notation is being
  // misused a little here.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * Ft_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * Ft_Bo_W.rotational();

  // Check the spatial traction at p. We know that geometry M is the halfspace,
  // so we'll check the spatial traction for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have
  // approximately the same magnitude as the normal traction. Note that the
  // regularized Coulomb friction model requires backing off of the tolerance
  // for comparing frictional traction components (against the Coulomb model);
  // the units are dissimilar (m/s vs. N and Nm) but using this as the tolerance
  // has proven useful.
  const double field_value = 0.5;  // in N/m².
  const double regularization_scalar =
      traction_calculator().regularization_scalar();
  EXPECT_NEAR(f_Bo_Y[0], -mu_coulomb * field_value, regularization_scalar);
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], field_value, tol());

  // A moment on the box will be generated due to the traction. The
  // origin of the box frame is located at (0,0,0) in the Y frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (-.5, 0, 0.5) yields the following.
  EXPECT_NEAR(tau_Bo_Y[0], 0.25, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.0, regularization_scalar);
  EXPECT_NEAR(tau_Bo_Y[2], 0.25, regularization_scalar);

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((Ft_Bo_W.translational() + Ft_Ao_W.translational()).norm(),
      0, tol());
}

// Tests the traction calculation with dissipation tractions but without
// friction.
TEST_P(MultibodyPlantHydroelasticTractionTests, TractionWithDissipation) {
  const double dissipation = 1.0;  // Units: s/m.
  const double mu_coulomb = 0.0;

  // Give the box an initial (vertical) velocity along the -z axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const double separating_velocity = -1.0;
  SetBoxTranslationalVelocity(R_WY *
      Vector3<double>(0, 0, separating_velocity));

  // Compute the magnitude of the normal traction. Note that the damping
  // constant at each point will be field value * dissipation coefficient.
  const double field_value = 0.5;  // in N/m².
  const double c = field_value * dissipation;  // N/m² * s/m = Ns/m³.
  const double damping_traction_magnitude = c * -separating_velocity;  // N/m².
  const double normal_traction_magnitude = field_value +
      damping_traction_magnitude;

  // Compute the spatial tractions at the origins of the body frames.
  multibody::SpatialForce<double> Ft_Ao_W, Ft_Bo_W;
  ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &Ft_Ao_W, &Ft_Bo_W);

  // Re-express the spatial tractions in Y's frame for easy interpretability.
  // Note that f and tau here are still tractions, so our notation is being
  // misused a little here.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * Ft_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * Ft_Bo_W.rotational();

  // Check the spatial traction at p. We know that geometry M is the halfspace,
  // so we'll check the spatial traction for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have the same
  // magnitude as the normal traction.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], normal_traction_magnitude, tol());

  // A moment on the box will be generated due to the traction. The
  // origin of the box frame is located at (0,0,0) in the world frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (0, 0, normal_traction_magnitude) yields
  // the following.
  EXPECT_NEAR(tau_Bo_Y[0], 0.5 * normal_traction_magnitude, tol());
  EXPECT_NEAR(tau_Bo_Y[1], -0.5 * normal_traction_magnitude, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((Ft_Bo_W.translational() + Ft_Ao_W.translational()).norm(),
      0, tol());
}

// Tests the traction calculation over an entire contact patch without
// friction or dissipation effecting the traction.
TEST_P(MultibodyPlantHydroelasticTractionTests, VanillaTractionOverPatch) {
  const double dissipation = 0.0;
  const double mu_coulomb = 0.0;

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Ao_W, F_Bo_W;
  traction_calculator().ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      plant_context(), plant(), contact_surface(), dissipation, mu_coulomb,
      &F_Ao_W, &F_Bo_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const Vector3<double> f_Bo_Y = R_WY.transpose() * F_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * F_Bo_W.rotational();

  // Check the spatial force. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead.
  const double field_value = 0.5;  // in N/m².
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], field_value, tol());

  // We expect no moment on the box.
  EXPECT_NEAR(tau_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());
}

// Tests the traction calculation over an entire contact patch with
// friction but without dissipation effecting the traction.
TEST_P(MultibodyPlantHydroelasticTractionTests, FrictionalTractionOverPatch) {
  const double dissipation = 0.0;
  const double mu_coulomb = 1.0;

  // Give the box an initial (vertical) velocity along the +x axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  SetBoxTranslationalVelocity(R_WY * Vector3<double>(1, 0, 0));

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Ao_W, F_Bo_W;
  traction_calculator().ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      plant_context(), plant(), contact_surface(), dissipation, mu_coulomb,
      &F_Ao_W, &F_Bo_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * F_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * F_Bo_W.rotational();

  // Check the spatial force. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead.  Note that the
  // regularized Coulomb friction model requires backing off of the tolerance
  // for comparing frictional traction components (against the Coulomb model);
  // the units are dissimilar (m/s vs. N and Nm) but using this as the tolerance
  // has proven useful.
  const double field_value = 0.5;  // in N/m².
  const double regularization_scalar =
      traction_calculator().regularization_scalar();
  EXPECT_NEAR(f_Bo_Y[0], -mu_coulomb * field_value, regularization_scalar);
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], field_value, tol());

  // A moment on the box will be generated due to the integral of the tractions.
  // The origin of the box frame is located at (0,0,0) in the Y frame.
  // The mean of all of the moment arms (we expect the tractions at each point
  // on the contact surface to be identical) will be (0, 0, -.5). Crossing this
  // vector with the traction at each point (-.5, 0, 0.5) yields (0, 0.25, 0).
  // The area of the contact surface is unity, so scaling this vector by the
  // area changes only the units, not the values.
  EXPECT_NEAR(tau_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.25, regularization_scalar);
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());
}

// Tests the traction calculation over an entire contact patch without
// friction butwith dissipation effecting the traction.
TEST_P(MultibodyPlantHydroelasticTractionTests,
    TractionOverPatchWithDissipation) {
  const double dissipation = 1.0;  // Units: s/m.
  const double mu_coulomb = 0.0;

  // Give the box an initial (vertical) velocity along the -z axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const double separating_velocity = -1.0;
  SetBoxTranslationalVelocity(R_WY *
      Vector3<double>(0, 0, separating_velocity));

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Ao_W, F_Bo_W;
  traction_calculator().ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      plant_context(), plant(), contact_surface(), dissipation, mu_coulomb,
      &F_Ao_W, &F_Bo_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * F_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * F_Bo_W.rotational();

  // Compute the magnitude of the normal traction. Note that the damping
  // constant at each point will be field value * dissipation coefficient.
  const double field_value = 0.5;  // in N/m².
  const double c = field_value * dissipation;  // N/m² * s/m = Ns/m³.
  const double damping_traction_magnitude = c * -separating_velocity;  // N/m².
  const double normal_traction_magnitude = field_value +
      damping_traction_magnitude;

  // Check the spatial force. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], normal_traction_magnitude, tol());

  // We expect no moment on the box.
  EXPECT_NEAR(tau_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());
}

// These transformations, denoted X_WY, are passed as parameters to the tests
// to allow changing the absolute (but not relative) poses of the two bodies.
const RigidTransform<double> poses[] = {
    RigidTransform<double>::Identity(),
    RigidTransform<double>(
        math::RotationMatrix<double>::MakeYRotation(M_PI_4),
        drake::Vector3<double>(1, 2, 3))
};

INSTANTIATE_TEST_CASE_P(PoseInstantiations,
                        MultibodyPlantHydroelasticTractionTests,
                        ::testing::ValuesIn(poses));

}  // namespace internal
}  // namespace multibody
}  // namespace drake

