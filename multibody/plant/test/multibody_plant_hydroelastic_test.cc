#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::systems::Context;
using drake::systems::Diagram;
using Eigen::Vector3d;

namespace drake {
namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static const std::vector<SpatialForce<double>>& EvalHydroelasticContactForces(
      const MultibodyPlant<double>& plant,
      const systems::Context<double>& context) {
    return plant.EvalHydroelasticContactForces(context).F_BBo_W_array;
  }
};

namespace {

// This fixture sets up a MultibodyPlant model of a compliant sphere and a rigid
// half-space to aid the testing of the proper wiring of MultibodyPlant with
// HydroelasticEngine.
class HydroelasticModelTests : public ::testing::Test {
 protected:
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, 0.0);

    AddGround(kFrictionCoefficient_, plant_);
    body_ = &AddObject(plant_, kSphereRadius_, kElasticModulus_, kDissipation_,
                       kFrictionCoefficient_);

    // The default contact model today is point contact.
    EXPECT_EQ(plant_->get_contact_model(), ContactModel::kPointContactOnly);

    // Tell the plant to use the hydroelastic model.
    plant_->set_contact_model(ContactModel::kHydroelasticsOnly);
    ASSERT_EQ(plant_->get_contact_model(), ContactModel::kHydroelasticsOnly);

    plant_->Finalize();

    diagram_ = builder.Build();

    // Create a context for this system:
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());
  }

  void AddGround(double friction_coefficient, MultibodyPlant<double>* plant) {
    const double kSize = 10;
    const RigidTransformd X_WG{Vector3d{0, 0, -kSize / 2}};
    const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
    geometry::Box ground = geometry::Box::MakeCube(kSize);
    plant->RegisterVisualGeometry(plant->world_body(), X_WG, ground,
                                  "GroundVisualGeometry", green);
    geometry::ProximityProperties props;
    geometry::AddRigidHydroelasticProperties(kSize, &props);
    geometry::AddContactMaterial(
        {}, {},
        CoulombFriction<double>(friction_coefficient, friction_coefficient),
        &props);
    plant->RegisterCollisionGeometry(plant->world_body(), X_WG, ground,
                                     "GroundCollisionGeometry",
                                     std::move(props));
  }

  const RigidBody<double>& AddObject(MultibodyPlant<double>* plant,
                                     double radius,
                                     double elastic_modulus,
                                     double dissipation,
                                     double friction_coefficient) {
    // Inertial properties are only needed when verifying accelerations since
    // hydro forces are only a function of state.
    const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
    const UnitInertia<double> G_BBcm = UnitInertia<double>::SolidSphere(radius);
    const SpatialInertia<double> M_BBcm_B(kMass_, p_BoBcm_B, G_BBcm);

    // Create a rigid body B with the mass properties of a uniform sphere.
    const RigidBody<double>& body = plant->AddRigidBody("body", M_BBcm_B);

    // Body B's visual geometry and collision geometry are a sphere.
    // The pose X_BG of block B's geometry frame G is an identity transform.
    Sphere shape(radius);
    const RigidTransformd X_BG;  // Identity transform.
    const Vector4<double> lightBlue(0.5, 0.8, 1.0, 1.0);
    plant->RegisterVisualGeometry(body, X_BG, shape, "BodyVisualGeometry",
                                  lightBlue);

    geometry::ProximityProperties props;
    // This should produce a level-2 refinement (two steps beyond octohedron).
    geometry::AddSoftHydroelasticProperties(radius / 2, &props);
    geometry::AddContactMaterial(
        elastic_modulus, dissipation,
        CoulombFriction<double>(friction_coefficient, friction_coefficient),
        &props);
    plant->RegisterCollisionGeometry(
        body, X_BG, shape, "BodyCollisionGeometry", std::move(props));
    return body;
  }

  void SetPose(double penetration) {
    RigidTransformd X_WB(Vector3d(0.0, 0.0, kSphereRadius_ - penetration));
    plant_->SetFreeBodyPose(plant_context_, *body_, X_WB);
  }

  // This method computes the repulsion force between a soft sphere and a rigid
  // half-space as predicted by the hydroelastic model, when dissipation is
  // zero. The integral is performed analytically. For this case,  the
  // extent field is specified to be e(r) = 1 - r / R, where `r` is the radial
  // spherical coordinate and `R` is the radius of the sphere. For a given
  // penetration distance d, the hydroelastic model predicts a contact
  // patch of radius `a` which is the intersection of the sphere with the half
  // space. Using trigonometry the contact patch radius is given by a² = d (2R -
  // d). The normal force is then computed by integrating the pressure p(r) = E
  // e(r) over the circular patch. Given the axial symmetry about the center of
  // the patch, we can write this integral as:
  //   P = 2π∫dρ⋅ρ⋅p(r(ρ))
  // with `ρ` the radial (2D) coordinate in the patch and `r` as before the
  // spherical coordinate. Since `ρ` and `r` are related by ρ² + (R - d)² = r²
  // we can perform the integral in either variable `ρ` or `r`.
  // The result is:
  //   P = π/3⋅E⋅d²(3 - 2d/R)
  // with a² = d (2R - d) the contact patch radius.
  double CalcAnalyticalHydroelasticsForce(double d) {
    DRAKE_DEMAND(0.0 <= d);
    // The patch radius predicted by the hydroelastic model.
    const double normal_force =
        M_PI / 3.0 * kElasticModulus_ * d * d * (3 - 2 * d / kSphereRadius_);
    return normal_force;
  }

  const double kFrictionCoefficient_{0.0};  // [-]
  const double kSphereRadius_{0.05};        // [m]
  const double kElasticModulus_{1.e5};      // [Pa]
  const double kDissipation_{0.0};          // [s/m]
  const double kMass_{1.2};                 // [kg]

  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* body_{nullptr};
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};
};

// This test verifies the value of the normal force computed numerically using
// the hydroelastic model implementation in Drake against an analytically
// computed force with the same model.
// We observed that the results do converge to the analytical solution as the
// mesh is refined, however we cannot yet show this in a test given we still do
// not have a way to specify a refinement level.
// However, the main purpose of this test is to verify that MultibodyPlant is
// properly wired with HydroelasticEngine. Correctness of the numerical
// computation of contact forces can be found in hydroelastic_traction_test.cc.
// TODO(amcastro-tri): Extend this test to verify convergence on mesh refinement
// once we have the capability to specify mesh refinement.
TEST_F(HydroelasticModelTests, ContactForce) {
  auto calc_force = [this](double penetration) -> double {
    SetPose(penetration);
    const auto& F_BBo_W_array =
        MultibodyPlantTester::EvalHydroelasticContactForces(*plant_,
                                                            *plant_context_);
    const SpatialForce<double>& F_BBo_W = F_BBo_W_array[body_->node_index()];
    return F_BBo_W.translational()[2];  // Normal force.
  };

  // We observed that the difference between the numerically computed
  // hydroelastic forces and the analytically computed hydroelastic force is
  // larger at smaller penetrations. This trend is expected since for a
  // tessellation of the sphere smaller patches are not as accurately resolved
  // as the larger patches which arise at greater penetration distances.
  // This trend was measured and captured in this lambda; the error at d = 0.01
  // is less than 25% while it goes below 15% at d = 0.04. (Note: this is
  // sensitive to mesh refinement; a coarser mesh is likely to fail this test.)
  auto calc_observed_percentile_error =
      [R = kSphereRadius_](double penetration) {
        return 27.5 - 10 / 0.6 * penetration / R;
      };

  for (const double extent : {0.2, 0.4, 0.6, 0.8}) {
    const double penetration = extent * kSphereRadius_;
    const double analytical_force =
        CalcAnalyticalHydroelasticsForce(penetration);
    const double numerical_force = calc_force(penetration);
    const double percentile_error =
        (analytical_force - numerical_force) / analytical_force;
    const double observed_percentile_error =
        calc_observed_percentile_error(penetration);
    // We expect the numerical results to be smaller than the analytical ones
    // since the tessellated sphere has a volume always smaller than the true
    // sphere.
    EXPECT_GT(percentile_error, 0.0);
    EXPECT_LT(percentile_error, observed_percentile_error);
  }
}

// The computation of hydroelastic forces and the effect external forces have on
// accelerations is tested elsewhere. This test merely verifies the proper
// wiring of the hydroelastic model into the computation of accelerations.
// Therefore we only test the acceleration of the CoM and ignore angular
// accelerations.
TEST_F(HydroelasticModelTests, ContactDynamics) {
  const double penetration = 0.02;
  SetPose(penetration);
  const auto& F_BBo_W_array =
      MultibodyPlantTester::EvalHydroelasticContactForces(*plant_,
                                                          *plant_context_);
  const SpatialForce<double>& F_BBo_W = F_BBo_W_array[body_->node_index()];
  // Contact force by hydroelastics.
  const Vector3<double> fhydro_BBo_W = F_BBo_W.translational();

  auto derivatives = plant_->AllocateTimeDerivatives();
  plant_->CalcTimeDerivatives(*plant_context_, derivatives.get());
  const VectorX<double> vdot =
      derivatives->get_generalized_velocity().CopyToVector();
  std::vector<SpatialAcceleration<double>> A_WBo(plant_->num_bodies());
  plant_->CalcSpatialAccelerationsFromVdot(*plant_context_, vdot, &A_WBo);
  // Translational acceleration of Bo.
  const Vector3<double> a_WBo = A_WBo[body_->index()].translational();

  // Expected acceleration, including gravity.
  const Vector3<double> a_WBo_expected =
      fhydro_BBo_W / kMass_ + plant_->gravity_field().gravity_vector();
  EXPECT_TRUE(CompareMatrices(a_WBo_expected, a_WBo,
                              40 * std::numeric_limits<double>::epsilon()));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
