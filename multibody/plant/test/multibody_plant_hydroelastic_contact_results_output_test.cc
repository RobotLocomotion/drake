#include <gtest/gtest.h>

#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

using Eigen::Vector3d;

namespace drake {

using geometry::ContactSurface;
using geometry::MeshField;
using geometry::SurfaceFace;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceVertex;
using geometry::SurfaceVertexIndex;
using geometry::SurfaceMesh;

namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static std::vector<geometry::ContactSurface<double>> ComputeContactSurfaces(
      const MultibodyPlant<double>& plant,
      const geometry::QueryObject<double>& query_object) {
    return plant.hydroelastics_engine_.ComputeContactSurfaces(query_object);
  }
};

namespace {
class HydroelasticContactResultsOutputTester : public ::testing::Test {
 protected:
  void SetUp() {
    const double radius = 1.0;  // sphere radius (m).

    // The vertical location of the sphere. Since this value is smaller than the
    // sphere radius, the sphere will intersect the half-space described by
    // z <= 0.
    const double z0 = 0.95 * radius;

    // Set friction and dissipation so that there is no effect on the traction.
    const double dissipation = 0.0;                    // s/m.
    const CoulombFriction<double> friction(0.0, 0.0);  // Static/dynamic.

    // Set some reasonable, but arbitrary, parameters: none of these will
    // affect the test results.
    const double mass = 2.0;                           // kg.
    const double elastic_modulus = 1e7;                // Pascals.
    const Vector3<double> gravity_W(0, 0, -9.8);       // m/s^2.

    // Create the plant.
    systems::DiagramBuilder<double> builder;
    geometry::SceneGraph<double>& scene_graph =
        *builder.AddSystem<geometry::SceneGraph<double>>();
    scene_graph.set_name("scene_graph");
    plant_ = builder.AddSystem(
        examples::multibody::bouncing_ball::MakeBouncingBallPlant(
            radius, mass, elastic_modulus, dissipation, friction, gravity_W,
            &scene_graph));
    plant_->set_contact_model(ContactModel::kHydroelasticsOnly);
    plant_->Finalize();

    // Sanity check on the availability of the optional source id before using
    // it.
    DRAKE_DEMAND(!!plant_->get_source_id());

    builder.Connect(scene_graph.get_query_output_port(),
                    plant_->get_geometry_query_input_port());
    builder.Connect(
        plant_->get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(plant_->get_source_id().value()));

    diagram_ = builder.Build();

    // Create a context for this system:
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());

    // Set the sphere's pose.
    math::RigidTransformd X_WB(Vector3d(0.0, 0.0, z0));
    plant_->SetFreeBodyPose(plant_context_, plant_->GetBodyByName("Ball"),
                            X_WB);

    // Set the sphere's velocity.
    const Vector3d w(0, 0, 0);  // angular velocity.
    const Vector3d v(1, 0, 0);  // linear velocity, for testing the field.
    const SpatialVelocity<double> V_WB(w, v);
    plant_->SetFreeBodySpatialVelocity(plant_context_,
                                       plant_->GetBodyByName("Ball"), V_WB);
  }

  const HydroelasticContactInfo<double>& contact_results() const {
    // Get the contact results from the plant.
    const ContactResults<double>& contact_results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            *plant_context_);
    DRAKE_DEMAND(contact_results.num_hydroelastic_contacts() == 1);
    return contact_results.hydroelastic_contact_info(0);
  }

  MultibodyPlant<double>* plant_{};
  systems::Context<double>* plant_context_{};

 private:
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> diagram_context_{};
};

// Checks that the ContactSurface from the output port is equivalent to that
// returned from the HydroelasticEngine.
TEST_F(HydroelasticContactResultsOutputTester, ContactSurfaceEquivalent) {
  // Get the query object so that we can compute the contact surfaces.
  const auto& query_object =
      plant_->get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<double>>(*plant_context_);

  // Compute the contact surface using the hydroelastic engine.
  std::vector<geometry::ContactSurface<double>> contact_surfaces =
      MultibodyPlantTester::ComputeContactSurfaces(*plant_, query_object);

  // Check that the two contact surfaces are equivalent.
  ASSERT_EQ(contact_surfaces.size(), 1);
  EXPECT_TRUE(
      contact_results().contact_surface().Equal(contact_surfaces.front()));
}

// Checks that the slip velocity field from the output port is consistent with
// the ball velocity that we have set.
TEST_F(HydroelasticContactResultsOutputTester, SlipVelocity) {
  const HydroelasticContactInfo<double>& results = contact_results();
  const MeshField<Vector3d, SurfaceMesh<double>>& vslip_AB_W =
      results.vslip_AB_W();

  // If Body A is the ball, then every point in the slip velocity field should
  // be +x. Otherwise, it should be -x.
  const Vector3d x(1, 0, 0);
  std::vector<geometry::GeometryId> ball_collision_geometries =
      plant_->GetCollisionGeometriesForBody(
          plant_->GetBodyByName("Ball"));
  const bool body_A_is_ball = (std::find(
      ball_collision_geometries.begin(), ball_collision_geometries.end(),
      results.contact_surface().id_M()) != ball_collision_geometries.end());
  const Vector3d expected_slip = body_A_is_ball ? x : -x;

  // Check that value of the slip velocity field points to +x. Checking just
  // the vertex values is sufficient only when vslip_AB_W() is of type
  // MeshFieldLinear.
  const geometry::MeshFieldLinear<Vector3d, SurfaceMesh<double>>* linear_field =
      dynamic_cast<
          const geometry::MeshFieldLinear<Vector3d, SurfaceMesh<double>>*>(
          &vslip_AB_W);
  DRAKE_DEMAND(linear_field);
  for (SurfaceVertexIndex i(0); i < vslip_AB_W.mesh().num_vertices(); ++i)
    ASSERT_EQ(vslip_AB_W.EvaluateAtVertex(i), expected_slip);
}

// Checks that the tractions from the output port is consistent with the normal
// and pressure.
TEST_F(HydroelasticContactResultsOutputTester, Traction) {
  const HydroelasticContactInfo<double>& results = contact_results();
  const MeshField<Vector3d, SurfaceMesh<double>>& traction_A_W =
      results.traction_A_W();

  // If Body A is the ball, then the traction field should point along +z.
  // Otherwise, it should point along -z.
  const Vector3d z(0, 0, 1);
  std::vector<geometry::GeometryId> ball_collision_geometries =
      plant_->GetCollisionGeometriesForBody(plant_->GetBodyByName("Ball"));
  const bool body_A_is_ball = (std::find(
      ball_collision_geometries.begin(), ball_collision_geometries.end(),
      results.contact_surface().id_M()) != ball_collision_geometries.end());
  const Vector3d expected_traction_direction = body_A_is_ball ? z : -z;

  // Check the traction. Checking just the vertex values is sufficient only when
  // traction_A_W() is of type MeshFieldLinear.
  const geometry::MeshFieldLinear<Vector3d, SurfaceMesh<double>>* linear_field =
      dynamic_cast<
          const geometry::MeshFieldLinear<Vector3d, SurfaceMesh<double>>*>(
          &traction_A_W);
  DRAKE_DEMAND(linear_field);
  for (SurfaceVertexIndex i(0); i < traction_A_W.mesh().num_vertices(); ++i) {
    const double pressure =
        results.contact_surface().e_MN().EvaluateAtVertex(i);
    ASSERT_EQ(traction_A_W.EvaluateAtVertex(i),
              expected_traction_direction * pressure);
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
