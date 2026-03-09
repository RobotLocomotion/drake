#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/multibody/rolling_sphere/populate_ball_plant.h"
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
    const double mass = 2.0;                      // kg.
    const double hydroelastic_modulus = 1e7;      // Pascals.
    const Vector3<double> gravity_W(0, 0, -9.8);  // m/s^2.

    // Create the plant.
    systems::DiagramBuilder<double> builder;
    plant_ = &AddMultibodyPlantSceneGraph(&builder, 0.0 /* time_step */).plant;

    // TODO(SeanCurtis-TRI): This should _not_ be using code from the examples/
    //  directory. Examples code shouldn't feed back into other code.
    examples::multibody::bouncing_ball::PopulateBallPlant(
        radius, mass, hydroelastic_modulus, dissipation, friction, gravity_W,
        false /* rigid_sphere */, false /* compliant_ground */, plant_);
    plant_->set_contact_model(ContactModel::kHydroelastic);
    plant_->Finalize();

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
    DRAKE_DEMAND(contact_results.plant() == plant_);
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
  // "Ball" by PopulateBallPlant(). So,
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
