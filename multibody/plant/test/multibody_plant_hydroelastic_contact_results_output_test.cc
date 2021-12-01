#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

using Eigen::Vector3d;

namespace drake {

using geometry::ContactSurface;

namespace multibody {
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
    const double hydroelastic_modulus = 1e7;           // Pascals.
    const Vector3<double> gravity_W(0, 0, -9.8);       // m/s^2.

    // Create the plant.
    systems::DiagramBuilder<double> builder;
    geometry::SceneGraph<double>& scene_graph =
        *builder.AddSystem<geometry::SceneGraph<double>>();
    scene_graph.set_name("scene_graph");
    // TODO(SeanCurtis-TRI): This should _not_ be using code from the examples/
    //  directory. Examples code shouldn't feed back into other code.
    plant_ = builder.AddSystem(
        examples::multibody::bouncing_ball::MakeBouncingBallPlant(
            0.0 /* mbp_dt */, radius, mass, hydroelastic_modulus, dissipation,
            friction, gravity_W, false /* rigid_sphere */,
            false /* soft_ground */, &scene_graph));
    plant_->set_contact_model(ContactModel::kHydroelastic);
    plant_->Finalize();

    // Sanity check on the availability of the optional source id before using
    // it.
    DRAKE_DEMAND(plant_->get_source_id().has_value());

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

  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> diagram_context_{};
};

// Checks that the ContactSurface from the output port is equivalent to that
// computed by SceneGraph.
TEST_F(HydroelasticContactResultsOutputTester, ContactSurfaceEquivalent) {
  // Get the query object so that we can compute the contact surfaces.
  const auto& query_object =
      plant_->get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<double>>(*plant_context_);

  // Compute the contact surface using the hydroelastic engine.
  ASSERT_FALSE(plant_->is_discrete());
  std::vector<geometry::ContactSurface<double>> contact_surfaces =
      query_object.ComputeContactSurfaces(
          geometry::HydroelasticContactRepresentation::kTriangle);

  // Check that the two contact surfaces are equivalent.
  ASSERT_EQ(contact_surfaces.size(), 1);
  EXPECT_TRUE(
      contact_results().contact_surface().Equal(contact_surfaces.front()));
}

// Checks that the spatial force (applied at the centroid) from the output port
// is consistent with the ball state that we have set.
TEST_F(HydroelasticContactResultsOutputTester, SpatialForceAtCentroid) {
  const HydroelasticContactInfo<double>& results = contact_results();
  const SpatialForce<double>& F_Ac_W = results.F_Ac_W();

  // The following crude quadrature process relies upon there being three
  // quadrature points per triangle.
  ASSERT_EQ(results.contact_surface().num_faces() * 3,
            results.quadrature_point_data().size());

  // Sanity check that geometry ID is consistent with direction of spatial
  // force. This check is based upon the convention that the force points into
  // Body A. If Body A is the ball, we expect the z-dimension of the
  // translational component of the spatial force to be positive; otherwise
  // we expect it to be negative.
  const std::vector<geometry::GeometryId> ball_collision_geometries =
      plant_->GetCollisionGeometriesForBody(
          plant_->GetBodyByName("Ball"));
  const bool body_A_is_ball = (std::find(
      ball_collision_geometries.begin(), ball_collision_geometries.end(),
      results.contact_surface().id_M()) != ball_collision_geometries.end());
  const double sign_scalar = (body_A_is_ball) ? 1.0 : -1.0;
  ASSERT_GT(sign_scalar * F_Ac_W.translational()[2], 0);

  // Our crude quadrature process, which uses the mean traction over the
  // surface of the triangle, gives us the same accuracy as Gaussian quadrature
  // because the frictional components of each traction are zero. Because this
  // shape is a sphere, we expect there to be no moment around the centroid.
  SpatialForce<double> F_Ac_W_expected;
  F_Ac_W_expected.SetZero();
  for (const HydroelasticQuadraturePointData<double>& datum :
       results.quadrature_point_data()) {
    F_Ac_W_expected.translational() +=
        results.contact_surface().area(datum.face_index) * 1.0 / 3 *
        datum.traction_Aq_W;
  }

  // Use a relative tolerance since the magnitude of the spatial force will be
  // on the order of 10^4.
  const double tol = 10 * F_Ac_W.translational().norm() *
                     std::numeric_limits<double>::epsilon();
  EXPECT_NEAR((F_Ac_W_expected.translational() - F_Ac_W.translational()).norm(),
              0, tol);
  EXPECT_NEAR((F_Ac_W_expected.rotational() - F_Ac_W.rotational()).norm(), 0,
              tol);
}

// Checks that the slip velocity from the output port is consistent with
// the ball velocity that we have set.
TEST_F(HydroelasticContactResultsOutputTester, SlipVelocity) {
  const HydroelasticContactInfo<double>& results = contact_results();
  const std::vector<HydroelasticQuadraturePointData<double>>&
      quadrature_point_data = results.quadrature_point_data();

  // If Body A is the ball, then every point in the slip velocity field should
  // be +x. Otherwise, it should be -x.
  const Vector3d x(1, 0, 0);
  const std::vector<geometry::GeometryId> ball_collision_geometries =
      plant_->GetCollisionGeometriesForBody(
          plant_->GetBodyByName("Ball"));
  const bool body_A_is_ball = (std::find(
      ball_collision_geometries.begin(), ball_collision_geometries.end(),
      results.contact_surface().id_M()) != ball_collision_geometries.end());
  const Vector3d expected_slip = body_A_is_ball ? x : -x;

  // Check that value of the slip velocity field points to +x.
  for (const auto& quadrature_point_datum : quadrature_point_data)
    ASSERT_EQ(quadrature_point_datum.vt_BqAq_W, expected_slip);
}

// Checks that the tractions from the output port are consistent with the normal
// and pressure.
TEST_F(HydroelasticContactResultsOutputTester, Traction) {
  const HydroelasticContactInfo<double>& results = contact_results();
  const std::vector<HydroelasticQuadraturePointData<double>>&
      quadrature_point_data = results.quadrature_point_data();

  // If Body A (with geometry M) is the ball, then the traction field should
  // point along +z (i.e., the contact surfaces normals point out of N and
  // into M and the tractions should point in the same direction). Otherwise, it
  // should point along -z.
  const Vector3d z(0, 0, 1);
  const std::vector<geometry::GeometryId> ball_collision_geometries =
      plant_->GetCollisionGeometriesForBody(plant_->GetBodyByName("Ball"));
  const bool body_A_is_ball = (std::find(
      ball_collision_geometries.begin(), ball_collision_geometries.end(),
      results.contact_surface().id_M()) != ball_collision_geometries.end());
  const Vector3d expected_traction_direction = body_A_is_ball ? z : -z;

  // Check the traction.
  for (const auto& quadrature_point_datum : quadrature_point_data) {
    // Convert the quadrature point to barycentric coordinates.
    const Vector3d p_barycentric =
        results.contact_surface().tri_mesh_W().CalcBarycentric(
            quadrature_point_datum.p_WQ, quadrature_point_datum.face_index);

    const double pressure = results.contact_surface().tri_e_MN().Evaluate(
        quadrature_point_datum.face_index, p_barycentric);

    // The conversion from Cartesian to barycentric coordinates introduces some
    // roundoff error. Test the values using a relative tolerance since the
    // pressure is generally much greater than unity.
    const double tol = pressure * 30 * std::numeric_limits<double>::epsilon();
    const Vector3d expected_traction = expected_traction_direction * pressure;
    EXPECT_NEAR(
        (quadrature_point_datum.traction_Aq_W - expected_traction).norm(), 0,
        tol);
  }
}

// TODO(amcastro-tri): Replace this *suggestive* test with an alternative test
//  that tests for actual derivative values. See the comments in PR 15219 for
//  discussion:
//  https://github.com/RobotLocomotion/drake/pull/15219#pullrequestreview-689675394

// Checks that an AutoDiffXd-valued plant will have forces with appropriate
// derivatives. In this case, the "correctness" of derivatives are not
// extensively evaluated. We're merely looking for indicators that things
// are correct (presence and size of derivatives, and expected values as far
// as convenient).
TEST_F(HydroelasticContactResultsOutputTester, AutoDiffXdSupport) {
  std::unique_ptr<systems::System<AutoDiffXd>> system_ad =
      diagram_->ToAutoDiffXd();
  systems::Diagram<AutoDiffXd>* diagram_ad =
      dynamic_cast<systems::Diagram<AutoDiffXd>*>(system_ad.get());
  auto context_ad = diagram_ad->CreateDefaultContext();
  const auto& plant_ad = dynamic_cast<const MultibodyPlant<AutoDiffXd>&>(
      diagram_ad->GetSubsystemByName(plant_->get_name()));
  systems::Context<AutoDiffXd>& plant_context_ad =
      diagram_ad->GetMutableSubsystemContext(plant_ad, context_ad.get());

  // Set the sphere's pose. We know the sphere's radius = 1, we want contact.
  // We'll leave a zero velocity.
  const double z0 = 0.95;
  const Vector3d p_WBo{0, 0, z0};
  const math::RigidTransform<AutoDiffXd> X_WB(math::InitializeAutoDiff(p_WBo));
  plant_ad.SetFreeBodyPose(&plant_context_ad, plant_ad.GetBodyByName("Ball"),
                           X_WB);

  const auto& query_object =
      plant_ad.get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<AutoDiffXd>>(plant_context_ad);

  // Compute the contact surface using the hydroelastic engine. We'll use this
  // as a reality check; make sure that the underlying contact surface has
  // derivatives as expected.
  std::vector<geometry::ContactSurface<AutoDiffXd>> contact_surfaces =
      query_object.ComputeContactSurfaces(
          geometry::HydroelasticContactRepresentation::kTriangle);

  ASSERT_EQ(contact_surfaces.size(), 1);
  // Contact surface documents the surface normal as pointing "out of N and into
  // M". So, if the ball maps to id_N, the normal points out of the ball and
  // into the half space. We exploit the knowledge that the ball body is named
  // "Ball" by MakeBouncingBallPlant(). So,
  //   id_N = the Ball body
  //     -> body A in the force results is half space
  //     -> f_Ac_W points *into* the half space.
  const geometry::FrameId frame_for_N =
      query_object.inspector().GetFrameId(contact_surfaces[0].id_N());
  ASSERT_EQ(plant_ad.GetBodyFromFrameId(frame_for_N)->name(), "Ball");

  // The area has derivatives (three) and the area only changes magnitude based
  // on p_WBo.z.
  const AutoDiffXd area = contact_surfaces[0].total_area();
  ASSERT_EQ(area.derivatives().size(), 3);
  ASSERT_NEAR(area.derivatives()[0], 0, 1e-15);
  ASSERT_NEAR(area.derivatives()[1], 0, 1e-15);
  // The area shrinks *quickly* as the sphere moves up.
  ASSERT_LT(area.derivatives()[2], -1);

  // Now we actually compute the forces.
  const ContactResults<AutoDiffXd>& contact_results =
      plant_ad.get_contact_results_output_port()
          .Eval<ContactResults<AutoDiffXd>>(plant_context_ad);
  DRAKE_DEMAND(contact_results.num_hydroelastic_contacts() == 1);

  const auto& contact_info = contact_results.hydroelastic_contact_info(0);

  // For the simple geometry in this problem, and since we asserted that body
  // A is the half space, the force f_Ac_W points downwards and is given by:
  //
  //   f_Ac_W = -fₙ(z)Ŵz
  //
  // where Ŵz is the z-axis of the world frame W, z is the component of p_WBo in
  // Ŵz and we defined the normal force fₙ(z) to be positive. For this problem
  // the normal vector is n̂ = Ŵz, independent of p_WBo.
  //
  // Thus the gradient of the contact force is:
  //
  //                      │ 0  0      0   |
  //    ∂f_Ac_W/∂p_WBo =  │ 0  0      0   |
  //                      │ 0  0  -∂fₙ/∂z |
  //
  // This reflects a decrease in force magnitude as we separate the sphere from
  // the ground (z increases). We'll verify this invariant below.
  //
  // Note: We're not computing the exact value of ∂fₙ/∂z. For an analytical
  // sphere this would be straightforward. In this case, we're using a
  // tessellated sphere and the full details depend on the nature of that
  // tessellation. So, this test simply confirms the sign.
  const auto& f_Ac_W = contact_info.F_Ac_W().translational();
  EXPECT_EQ(f_Ac_W.x().derivatives().size(), 3);
  EXPECT_EQ(f_Ac_W.y().derivatives().size(), 3);
  EXPECT_EQ(f_Ac_W.z().derivatives().size(), 3);
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_LT(f_Ac_W.z().value(), 0);
  EXPECT_TRUE(
      CompareMatrices(f_Ac_W.x().derivatives(), Vector3d{0, 0, 0}, kEps));
  EXPECT_TRUE(
      CompareMatrices(f_Ac_W.y().derivatives(), Vector3d{0, 0, 0}, kEps));
  EXPECT_NEAR(f_Ac_W.z().derivatives()[0], 0, kEps);
  EXPECT_NEAR(f_Ac_W.z().derivatives()[1], 0, kEps);
  EXPECT_GT(f_Ac_W.z().derivatives()[2], kEps);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
