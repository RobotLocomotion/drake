#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::FrameId;
using drake::geometry::GeometryId;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::Simulator;
using Eigen::Vector3d;
using std::make_unique;
using std::unique_ptr;

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

  static const std::vector<SpatialForce<double>>&
  EvalSpatialContactForcesContinuous(const MultibodyPlant<double>& plant,
                                     const systems::Context<double>& context) {
    return plant.EvalSpatialContactForcesContinuous(context);
  }
};

namespace {

// This fixture sets up a MultibodyPlant model of a compliant sphere and a rigid
// half-space to confirm that MBP computes the correct forces due to
// hydroelastic contact.
class HydroelasticModelTests : public ::testing::Test {
 protected:
  void SetUpModel(double mbp_dt) {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, mbp_dt);

    AddGround(kFrictionCoefficient, plant_);
    body_ = &AddObject(plant_, kSphereRadius, kElasticModulus, kDissipation,
                       kFrictionCoefficient);

    // The default contact model today is point contact.
    EXPECT_EQ(plant_->get_contact_model(), ContactModel::kPoint);

    // Tell the plant to use the hydroelastic model.
    plant_->set_contact_model(ContactModel::kHydroelastic);
    ASSERT_EQ(plant_->get_contact_model(), ContactModel::kHydroelastic);

    plant_->Finalize();

    // Connect visualizer. Useful for when this test is used for debugging.
    drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph_);
    ConnectContactResultsToDrakeVisualizer(&builder, *plant_, *scene_graph_);

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
                                     double radius, double hydroelastic_modulus,
                                     double dissipation,
                                     double friction_coefficient) {
    // Inertial properties are only needed when verifying accelerations since
    // hydro forces are only a function of state.
    const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
    const UnitInertia<double> G_BBcm = UnitInertia<double>::SolidSphere(radius);
    const SpatialInertia<double> M_BBcm_B(kMass, p_BoBcm_B, G_BBcm);

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
    // This should produce a level-2 refinement (two steps beyond octahedron).
    geometry::AddSoftHydroelasticProperties(
        radius / 2, hydroelastic_modulus, &props);
    geometry::AddContactMaterial(
        dissipation, {},
        CoulombFriction<double>(friction_coefficient, friction_coefficient),
        &props);
    plant->RegisterCollisionGeometry(body, X_BG, shape, "BodyCollisionGeometry",
                                     std::move(props));
    return body;
  }

  void SetPose(double penetration) {
    RigidTransformd X_WB(Vector3d(0.0, 0.0, kSphereRadius - penetration));
    plant_->SetFreeBodyPose(plant_context_, *body_, X_WB);
  }

  // This method computes the repulsion force between a compliant sphere and a
  // rigid half-space as predicted by the hydroelastic contact model, when
  // dissipation is zero. The integral is performed analytically. For this
  // case, the extent field is specified to be e(r) = 1 - r / R, where `r` is
  // the radial spherical coordinate and `R` is the radius of the sphere. For
  // a given penetration distance d, the hydroelastic model predicts a contact
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
        M_PI / 3.0 * kElasticModulus * d * d * (3 - 2 * d / kSphereRadius);
    return normal_force;
  }

  const double kFrictionCoefficient{0.0};  // [-]
  const double kSphereRadius{0.05};        // [m]
  const double kElasticModulus{1.e5};      // [Pa]
  // A non-zero dissipation value is used to quickly dissipate energy in tests
  // running a simulation on this case.
  const double kDissipation{10.0};         // [s/m]
  const double kMass{1.2};                 // [kg]

  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* body_{nullptr};
  unique_ptr<Diagram<double>> diagram_;
  unique_ptr<Context<double>> diagram_context_;
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
  SetUpModel(0.0);
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
      [R = kSphereRadius](double penetration) {
        return 27.5 - 10 / 0.6 * penetration / R;
      };

  for (const double extent : {0.2, 0.4, 0.6, 0.8}) {
    const double penetration = extent * kSphereRadius;
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
  SetUpModel(0.0);
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
      fhydro_BBo_W / kMass + plant_->gravity_field().gravity_vector();
  EXPECT_TRUE(CompareMatrices(a_WBo_expected, a_WBo,
                              40 * std::numeric_limits<double>::epsilon()));
}

// In this test we run the model of the sphere lying on the ground for long
// enough to reach a steady state in which the hydroelastic forces balance the
// weight of the sphere. The purpose of this test is to verify the results of a
// simulation using the discrete approximation of hydroelastics.
TEST_F(HydroelasticModelTests, DiscreteTamsiSolver) {
  SetUpModel(5.0e-3);
  Simulator<double> simulator(*diagram_);
  auto& diagram_context = simulator.get_mutable_context();
  auto& plant_context = plant_->GetMyMutableContextFromRoot(&diagram_context);

  // Set initial condition.
  const RigidTransformd X_WB(Vector3d(0.0, 0.0, kSphereRadius));
  plant_->SetFreeBodyPose(&plant_context, *body_, X_WB);
  diagram_->Publish(diagram_context);

  // Run simulation for long enough to reach the steady state.
  simulator.AdvanceTo(0.5);

  // In steady state, the normal hydroelastic force must match the weight of
  // the sphere. We verify this.
  const auto& F_BBo_W_array =
      MultibodyPlantTester::EvalHydroelasticContactForces(*plant_,
                                                          plant_context);
  const SpatialForce<double>& F_BBo_W = F_BBo_W_array[body_->node_index()];
  const Vector3<double> fz_BBo_W = F_BBo_W.translational();

  // The contact force should match the weight of the sphere.
  const Vector3<double> fz_BBo_W_expected =
      -plant_->gravity_field().gravity_vector() * kMass;

  // We use a tolerance value based on previous runs of this test.
  const double tolerance = 2.0e-8;
  EXPECT_TRUE(CompareMatrices(fz_BBo_W, fz_BBo_W_expected, tolerance));
}

// This tests consistency across the ContactModel modes: point pair,
// hydroelastic only, and hydroelastic with fallback. We create a scenario with
// three objects: two rigid spheres and a soft box. One rigid sphere is in
// contact with the other two shapes. In this scenario:
//   - Evaluating with point pair produces two contact forces on the common
//     rigid sphere.
//   - Evaluating with hydroelastic only should throw (rigid-rigid contact
//     is not supported).
//   - Evaluating with hydroelastic with fallback should produce a point contact
//     and a contact surface contact.
// In this case, we'll query the plant for both the contact results and the
// spatial contact forces. They should match and show the heterogeneity of
// contact types (as appropriate).
class ContactModelTest : public ::testing::Test {
 protected:
  // Sets up a system consisting of two rigid balls and a compliant box.
  // @param connect_scene_graph
  //                   For testing error handling when SceneGraph is added but
  //                   not connected to MultibodyPlant. Set to true for the
  //                   usual operation. Set to false to test the error handling.
  // @param time_step  Set to 0 to set up a continuous system and
  //                   non-zero to set up a discrete system.
  // @param are_rigid_spheres_in_contact
  //                   Set to true to have a rigid-rigid contact between two
  //                   balls and a rigid-compliant contact between the rigid
  //                   ball and a compliant box. Set to false to have only the
  //                   rigid-compliant contact between a rigid ball and a
  //                   compliant box, and the two rigid balls will be too far
  //                   apart to make contact.
  void Configure(ContactModel model, bool connect_scene_graph = true,
                 double time_step = 0.0,
                 bool are_rigid_spheres_in_contact = true) {
    systems::DiagramBuilder<double> builder;
    if (connect_scene_graph) {
      std::tie(plant_, scene_graph_) =
          AddMultibodyPlantSceneGraph(&builder, time_step);
    } else {
      // Even though we add a SceneGraph, with this option we leave it
      // disconnected so that we can test the correct throw message
      // from:
      // TEST_F(ContactModelTest, HydroelasticWithFallbackDisconnectedPorts).
      plant_ = builder.AddSystem(
          std::make_unique<MultibodyPlant<double>>(time_step));
      scene_graph_ = builder.AddSystem(std::make_unique<SceneGraph<double>>());
      plant_->RegisterAsSourceForSceneGraph(scene_graph_);
    }

    geometry::ProximityProperties props;
    geometry::AddContactMaterial(
        kDissipation, {},
        CoulombFriction<double>(kFrictionCoefficient, kFrictionCoefficient),
        &props);
    AddGround(props, plant_);

    // Although we're providing elastic modulus and dissipation for the rigid
    // spheres, those values will be ignored.
    first_ball_ = &AddSphere("sphere1", kSphereRadius, props, plant_);
    second_ball_ = &AddSphere("sphere2", kSphereRadius, props, plant_);

    // Tell the plant to use the given model.
    plant_->set_contact_model(model);
    ASSERT_EQ(plant_->get_contact_model(), model);

    plant_->Finalize();

    diagram_ = builder.Build();

    // Create a context for this system:
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());

    // Pose the ball.
    RigidTransformd X_WS1{Vector3d{0.0, 0.0, kSphereRadius * 0.9}};
    plant_->SetFreeBodyPose(plant_context_, *first_ball_, X_WS1);
    RigidTransformd X_WS2{Vector3d{0.0, 0.0, 2 * kSphereRadius}};
    if (!are_rigid_spheres_in_contact) {
      X_WS2 = RigidTransformd(100.0 * kSphereRadius * Vector3d::UnitZ());
    }
    plant_->SetFreeBodyPose(plant_context_, *second_ball_, X_WS2);
  }

  void AddGround(geometry::ProximityProperties contact_material,
                 MultibodyPlant<double>* plant) {
    const double kSize = 10;
    const RigidTransformd X_WG{Vector3d{0, 0, -kSize / 2}};
    geometry::Box ground = geometry::Box::MakeCube(kSize);
    geometry::AddSoftHydroelasticProperties(
        kSize, kElasticModulus, &contact_material);
    plant->RegisterCollisionGeometry(plant->world_body(), X_WG, ground,
                                     "GroundCollisionGeometry",
                                     std::move(contact_material));
  }

  const RigidBody<double>& AddSphere(
      const std::string& name, double radius,
      geometry::ProximityProperties contact_material,
      MultibodyPlant<double>* plant) {
    // Inertial properties are only needed when verifying accelerations since
    // hydro forces are only a function of state.
    const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
    const UnitInertia<double> G_BBcm = UnitInertia<double>::SolidSphere(radius);
    const SpatialInertia<double> M_BBcm_B(kMass, p_BoBcm_B, G_BBcm);

    // Create a rigid body B with the mass properties of a uniform sphere.
    const RigidBody<double>& body = plant->AddRigidBody(name, M_BBcm_B);

    // Body B's collision geometry is a sphere.
    // The pose X_BG of block B's geometry frame G is an identity transform.
    Sphere shape(radius);
    const RigidTransformd X_BG;  // Identity transform.
    geometry::AddRigidHydroelasticProperties(radius, &contact_material);
    plant->RegisterCollisionGeometry(body, X_BG, shape, "collision",
                                     std::move(contact_material));
    return body;
  }

  // Compute a set of spatial forces from the given contact results. The
  // translational component of the force acting on a body is defined to be
  // acting at the _origin_ of the body.
  // This method is used to compute an expected result from
  // MultibodyPlant::EvalSpatialContactForcesContinuous() from contact results.
  // EvalSpatialContactForcesContinuous() is an internal private method of
  // MultibodyPlant and, as many other multibody methods, sorts the results in
  // the returned array of spatial forces by BodyNodeIndex. Therefore, the
  // expected results being generated must also be sorted by BodyNodeIndex.
  std::vector<SpatialForce<double>> SpatialForceFromContactResults(
      const ContactResults<double>& contacts) {
    std::vector<SpatialForce<double>> F_BBo_W_array(
        plant_->num_bodies(),
        SpatialForce<double>{Vector3d::Zero(), Vector3d::Zero()});

    for (int i = 0; i < contacts.num_point_pair_contacts(); ++i) {
      const auto& contact_info = contacts.point_pair_contact_info(i);
      const SpatialForce<double> F_Bc_W{Vector3d::Zero(),
                                        contact_info.contact_force()};
      const Vector3d& p_WC = contact_info.contact_point();
      const auto& bodyA = plant_->get_body(contact_info.bodyA_index());
      const Vector3d& p_WAo =
          bodyA.EvalPoseInWorld(*plant_context_).translation();
      const Vector3d& p_CAo_W = p_WAo - p_WC;
      const auto& bodyB = plant_->get_body(contact_info.bodyB_index());
      const Vector3d& p_WBo =
          bodyB.EvalPoseInWorld(*plant_context_).translation();
      const Vector3d& p_CBo_W = p_WBo - p_WC;

      // N.B. Since we are using this method to test the internal (private)
      // MultibodyPlant::EvalSpatialContactForcesContinuous(), we must use
      // internal API to generate a forces vector sorted in the same way, by
      // internal::BodyNodeIndex.
      F_BBo_W_array[bodyB.node_index()] += F_Bc_W.Shift(p_CBo_W);
      F_BBo_W_array[bodyA.node_index()] -= F_Bc_W.Shift(p_CAo_W);
    }

    for (int i = 0; i < contacts.num_hydroelastic_contacts(); ++i) {
      const auto& contact_info = contacts.hydroelastic_contact_info(i);
      const auto& surface = contact_info.contact_surface();
      const auto& inspector = scene_graph_->model_inspector();

      const GeometryId A_id = surface.id_M();
      const FrameId fA_id = inspector.GetFrameId(A_id);
      const Body<double>& body_A = *plant_->GetBodyFromFrameId(fA_id);
      const GeometryId B_id = surface.id_N();
      const FrameId fB_id = inspector.GetFrameId(B_id);
      const Body<double>& body_B = *plant_->GetBodyFromFrameId(fB_id);

      const Vector3d& p_WC = surface.centroid();
      const Vector3d& p_WAo =
          body_A.EvalPoseInWorld(*plant_context_).translation();
      const Vector3d p_CAo_W = p_WAo - p_WC;
      const Vector3d& p_WBo =
          body_B.EvalPoseInWorld(*plant_context_).translation();
      const Vector3d p_CBo_W = p_WBo - p_WC;

      // The force applied to body A at a fixed point coincident with the
      // centroid point C.
      const SpatialForce<double>& F_Ac_W = contact_info.F_Ac_W();
      F_BBo_W_array[body_A.node_index()] += F_Ac_W.Shift(p_CAo_W);
      F_BBo_W_array[body_B.node_index()] -= F_Ac_W.Shift(p_CBo_W);
    }

    return F_BBo_W_array;
  }

  // Get the contact results from the plant (calculating them as necessary).
  const ContactResults<double>& GetContactResults() const {
    return plant_->get_contact_results_output_port()
        .Eval<ContactResults<double>>(*plant_context_);
  }

  const double kFrictionCoefficient{0.0};  // [-]
  const double kSphereRadius{0.05};        // [m]
  const double kElasticModulus{1.e5};      // [Pa]
  const double kDissipation{0.0};          // [s/m]
  const double kMass{1.2};                 // [kg]

  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* first_ball_{nullptr};
  const RigidBody<double>* second_ball_{nullptr};
  unique_ptr<Diagram<double>> diagram_;
  unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};
};

TEST_F(ContactModelTest, PointPairContact) {
  this->Configure(ContactModel::kPoint);
  const ContactResults<double>& contact_results = GetContactResults();
  ASSERT_EQ(contact_results.num_point_pair_contacts(), 2);
  ASSERT_EQ(contact_results.num_hydroelastic_contacts(), 0);

  std::vector<SpatialForce<double>> F_BBo_W_array_expected =
      this->SpatialForceFromContactResults(contact_results);

  const std::vector<SpatialForce<double>>& F_BBo_W_array =
      MultibodyPlantTester::EvalSpatialContactForcesContinuous(*plant_,
                                                               *plant_context_);
  EXPECT_EQ(F_BBo_W_array.size(), plant_->num_bodies());
  // Note: We're skipping the _world_ body; EvalSpatialContactForcesContinuous()
  // reports zero spatial force for the world body. (This ultimately comes from
  // the implementation of MBP::CalcAndAddContactForcesByPenaltyMethod().)
  for (int b = 1; b < plant_->num_bodies(); ++b) {
    // Confirm that we don't trivially have matching zero-magnitude forces.
    EXPECT_GT(F_BBo_W_array[b].get_coeffs().norm(), 0);
    EXPECT_TRUE(CompareMatrices(F_BBo_W_array[b].get_coeffs(),
                                F_BBo_W_array_expected[b].get_coeffs()));
  }
}

TEST_F(ContactModelTest, HydroelasticOnly) {
  this->Configure(ContactModel::kHydroelastic);
  // Rigid-rigid contact precludes successful evaluation.
  DRAKE_EXPECT_THROWS_MESSAGE(GetContactResults(), std::logic_error,
                              "Requested contact between two rigid objects .+");
}

TEST_F(ContactModelTest, HydroelasitcWithFallback) {
  this->Configure(ContactModel::kHydroelasticWithFallback);
  const ContactResults<double>& contact_results = GetContactResults();
  EXPECT_EQ(contact_results.num_point_pair_contacts(), 1);
  EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);

  std::vector<SpatialForce<double>> F_BBo_W_array_expected =
      this->SpatialForceFromContactResults(contact_results);

  const std::vector<SpatialForce<double>>& F_BBo_W_array =
      MultibodyPlantTester::EvalSpatialContactForcesContinuous(*plant_,
                                                               *plant_context_);
  EXPECT_EQ(F_BBo_W_array.size(), plant_->num_bodies());
  // Note: We're skipping the _world_ body; EvalSpatialContactForcesContinuous()
  // reports zero spatial force for the world body. (This ultimately comes from
  // the implementation of MBP::CalcAndAddContactForcesByPenaltyMethod().)
  for (int b = 1; b < plant_->num_bodies(); ++b) {
    // Confirm that we don't trivially have matching zero-magnitude forces.
    EXPECT_GT(F_BBo_W_array[b].get_coeffs().norm(), 0);
    EXPECT_TRUE(CompareMatrices(F_BBo_W_array[b].get_coeffs(),
                                F_BBo_W_array_expected[b].get_coeffs()));
  }
}

TEST_F(ContactModelTest, HydroelasticWithFallbackDisconnectedPorts) {
  this->Configure(ContactModel::kHydroelasticWithFallback, false);

  // Plant was not connected to the SceneGraph in a diagram, so its input port
  // should be invalid.
  DRAKE_EXPECT_THROWS_MESSAGE(
      GetContactResults(), std::logic_error,
      "The geometry query input port \\(see "
      "MultibodyPlant::get_geometry_query_input_port\\(\\)\\) "
      "of this MultibodyPlant is not connected. Please connect the"
      "geometry query output port of a SceneGraph object "
      "\\(see SceneGraph::get_query_output_port\\(\\)\\) to this plants input "
      "port in a Diagram.");
}

// TODO(DamrongGuoy): Create an independent test fixture instead of using
//  inheritance and consider using parameter-value tests.

// Tests MultibodyPlant::CalcContactSurfaces() which is used in
// kHydroelastic contact model for both continuous systems and discrete
// systems.
//
// This fixture sets up only rigid-compliant contact without rigid-rigid
// contact.
class CalcContactSurfacesTest : public ContactModelTest {
 protected:
  // @param time_step   Set to 0 to select a continuous system, and non-zero
  //                    for a discrete system. The actual non-zero value is not
  //                    relevant because we are not doing time stepping.
  void Configure(double time_step) {
    const bool connect_scene_graph = true;
    // No rigid-rigid contact. Only the rigid-compliant contact.
    bool are_rigid_spheres_in_contact = false;
    ContactModelTest::Configure(ContactModel::kHydroelastic,
                                connect_scene_graph, time_step,
                                are_rigid_spheres_in_contact);
  }
};

TEST_F(CalcContactSurfacesTest, ContinuousSystem_Triangles) {
  const double time_step = 0.0;  // Zero to select continuous system.
  this->Configure(time_step);

  const ContactResults<double>& contact_results = GetContactResults();

  EXPECT_EQ(contact_results.num_point_pair_contacts(), 0);
  EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);
  EXPECT_TRUE(
      contact_results.hydroelastic_contact_info(0).contact_surface().Equal(
          plant_->get_geometry_query_input_port()
              .template Eval<geometry::QueryObject<double>>(*plant_context_)
              .ComputeContactSurfaces(
                  geometry::HydroelasticContactRepresentation::kTriangle)
              .at(0)));
}

TEST_F(CalcContactSurfacesTest, DiscreteSystem_Polygons) {
  const double time_step = 5.0e-3;  // Non-zero to select discrete system.
  this->Configure(time_step);

  const ContactResults<double>& contact_results = GetContactResults();

  EXPECT_EQ(contact_results.num_point_pair_contacts(), 0);
  EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);
  EXPECT_TRUE(
      contact_results.hydroelastic_contact_info(0).contact_surface().Equal(
          plant_->get_geometry_query_input_port()
              .template Eval<geometry::QueryObject<double>>(*plant_context_)
              .ComputeContactSurfaces(
                  geometry::HydroelasticContactRepresentation::kPolygon)
              .at(0)));
}

// TODO(DamrongGuoy): Create an independent test fixture instead of using
//  inheritance and consider using parameter-value tests.

// Tests MultibodyPlant::CalcHydroelasticWithFallback() which is used in
// kHydroelasticWithFallback contact model for both continuous systems and
// discrete systems.
//
// This fixture sets up both rigid-compliant contact and rigid-rigid
// contact.
class CalcHydroelasticWithFallbackTest : public CalcContactSurfacesTest {
 protected:
  // @param time_step   Set to 0 to select a continuous system, and non-zero
  //                    for a discrete system. The actual non-zero value is not
  //                    relevant because we are not doing time stepping.
  void Configure(double time_step) {
    const bool connect_scene_graph = true;
    // Get both the rigid-rigid-sphere contact and the rigid-compliant
    // sphere-box contact.
    bool are_rigid_spheres_in_contact = true;
    ContactModelTest::Configure(ContactModel::kHydroelasticWithFallback,
                                connect_scene_graph, time_step,
                                are_rigid_spheres_in_contact);
  }
};

TEST_F(CalcHydroelasticWithFallbackTest, ContinuousSystem_Triangles) {
  const double time_step = 0.0;  // Zero to select continuous system.
  this->Configure(time_step);

  const ContactResults<double>& contact_results = GetContactResults();

  std::vector<geometry::ContactSurface<double>> expected_surfaces;
  std::vector<geometry::PenetrationAsPointPair<double>> expected_point_pairs;
  plant_->get_geometry_query_input_port()
      .template Eval<geometry::QueryObject<double>>(*plant_context_)
      .ComputeContactSurfacesWithFallback(
          geometry::HydroelasticContactRepresentation::kTriangle,
          &expected_surfaces, &expected_point_pairs);

  // We only check the penetration depth as an evidence that the tested
  // result is what expected.
  EXPECT_EQ(contact_results.num_point_pair_contacts(), 1);
  EXPECT_EQ(contact_results.point_pair_contact_info(0).point_pair().depth,
              expected_point_pairs.at(0).depth);

  EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);
  EXPECT_TRUE(
      contact_results.hydroelastic_contact_info(0).contact_surface().Equal(
          expected_surfaces.at(0)));
}

TEST_F(CalcHydroelasticWithFallbackTest, DiscreteSystem_Polygons) {
  const double time_step = 5.0e-3;  // Non-zero to select discrete system.
  this->Configure(time_step);

  const ContactResults<double>& contact_results = GetContactResults();

  std::vector<geometry::ContactSurface<double>> expected_surfaces;
  std::vector<geometry::PenetrationAsPointPair<double>> expected_point_pairs;
  plant_->get_geometry_query_input_port()
      .template Eval<geometry::QueryObject<double>>(*plant_context_)
      .ComputeContactSurfacesWithFallback(
          geometry::HydroelasticContactRepresentation::kPolygon,
          &expected_surfaces, &expected_point_pairs);

  // We only check the penetration depth as an evidence that the tested
  // result is what expected.
  EXPECT_EQ(contact_results.num_point_pair_contacts(), 1);
  EXPECT_EQ(contact_results.point_pair_contact_info(0).point_pair().depth,
            expected_point_pairs.at(0).depth);

  EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);
  EXPECT_TRUE(
      contact_results.hydroelastic_contact_info(0).contact_surface().Equal(
          expected_surfaces.at(0)));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
