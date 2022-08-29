#include "drake/multibody/plant/compliant_contact_manager.h"

#include <algorithm>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_limit_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/space_xyz_mobilizer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/zero_order_hold.h"

using drake::geometry::GeometryId;
using drake::geometry::PenetrationAsPointPair;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::TriangleSurfaceMesh;
using drake::geometry::VolumeMesh;
using drake::geometry::VolumeMeshFieldLinear;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::MergeNormalAndTangent;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapFrictionConeConstraint;
using drake::multibody::contact_solvers::internal::SapHolonomicConstraint;
using drake::multibody::contact_solvers::internal::SapLimitConstraint;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;
using drake::multibody::internal::DiscreteContactPair;
using drake::systems::Context;
using drake::systems::PassThrough;
using drake::systems::ZeroOrderHold;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// TODO(amcastro-tri): Implement AutoDiffXd testing.

namespace drake {
namespace multibody {
namespace internal {

constexpr double kEps = std::numeric_limits<double>::epsilon();

// Helper function for NaN initialization.
static constexpr double nan() {
  return std::numeric_limits<double>::quiet_NaN();
}

// Friend class used to provide access to a selection of private functions in
// CompliantContactManager for testing purposes.
class CompliantContactManagerTest {
 public:
  static const internal::MultibodyTreeTopology& topology(
      const CompliantContactManager<double>& manager) {
    return manager.tree_topology();
  }

  static const std::vector<geometry::ContactSurface<double>>&
  EvalContactSurfaces(const CompliantContactManager<double>& manager,
                      const Context<double>& context) {
    return manager.EvalContactSurfaces(context);
  }

  static const std::vector<DiscreteContactPair<double>>&
  EvalDiscreteContactPairs(const CompliantContactManager<double>& manager,
                           const Context<double>& context) {
    return manager.EvalDiscreteContactPairs(context);
  }

  static std::vector<ContactPairKinematics<double>> CalcContactKinematics(
      const CompliantContactManager<double>& manager,
      const Context<double>& context) {
    return manager.CalcContactKinematics(context);
  }

  static const ContactProblemCache<double>& EvalContactProblemCache(
      const CompliantContactManager<double>& manager,
      const Context<double>& context) {
    return manager.EvalContactProblemCache(context);
  }

  static VectorXd CalcFreeMotionVelocities(
      const CompliantContactManager<double>& manager,
      const Context<double>& context) {
    VectorXd v_star(manager.plant().num_velocities());
    manager.CalcFreeMotionVelocities(context, &v_star);
    return v_star;
  }

  static std::vector<MatrixXd> CalcLinearDynamicsMatrix(
      const CompliantContactManager<double>& manager,
      const Context<double>& context) {
    std::vector<MatrixXd> A;
    manager.CalcLinearDynamicsMatrix(context, &A);
    return A;
  }

  static void PackContactSolverResults(
      const CompliantContactManager<double>& manager,
      const contact_solvers::internal::SapContactProblem<double>& problem,
      int num_contacts,
      const contact_solvers::internal::SapSolverResults<double>& sap_results,
      contact_solvers::internal::ContactSolverResults<double>*
          contact_results) {
    manager.PackContactSolverResults(problem, num_contacts, sap_results,
                                     contact_results);
  }

  static void CalcNonContactForces(
      const CompliantContactManager<double>& manager,
      const Context<double>& context, MultibodyForces<double>* forces) {
    manager.CalcNonContactForces(context, forces);
  }

  static void AddLimitConstraints(
      const CompliantContactManager<double>& manager,
      const Context<double>& context, const VectorXd& v_star,
      SapContactProblem<double>* problem) {
    manager.AddLimitConstraints(context, v_star, problem);
  }

  static const DeformableDriver<double>* deformable_driver(
      const CompliantContactManager<double>& manager) {
    return manager.deformable_driver_.get();
  }
};

// Tests that in SetDiscreteUpdateManager, a registered DeformableModel will
// cause a DeformableDriver to be instantiated in the manager.
GTEST_TEST(CompliantContactManagerTest, ExtractModelInfo) {
  CompliantContactManager<double> manager;
  EXPECT_EQ(CompliantContactManagerTest::deformable_driver(manager), nullptr);
  MultibodyPlant<double> plant(0.01);
  auto deformable_model = std::make_unique<DeformableModel<double>>(&plant);
  plant.AddPhysicalModel(std::move(deformable_model));
  plant.Finalize();
  auto contact_manager = std::make_unique<CompliantContactManager<double>>();
  const CompliantContactManager<double>* contact_manager_ptr =
      contact_manager.get();
  plant.SetDiscreteUpdateManager(std::move(contact_manager));
  EXPECT_NE(
      CompliantContactManagerTest::deformable_driver(*contact_manager_ptr),
      nullptr);
}

// TODO(DamrongGuoy): Simplify the test fixture somehow (need discussion
//  among the architects). Due to the existing architecture of our code,
//  our fixture is too complex for the purpose of unit tests. Ideally there
//  should be one-to-one matching between tested functions (_.h) and testing
//  functions (_test.cc) as the unit tests.

// In this fixture we set a simple model consisting of a flat ground,
// a sphere (sphere 1) on top of the ground, and another sphere (sphere 2)
// on top the first sphere. They are assigned to be rigid-hydroelastic,
// compliant-hydroelastic, or non-hydroelastic to test various cases of
// contact quantities computed by the CompliantContactManager.
class SpheresStack : public ::testing::Test {
 public:
  // Contact model parameters.
  struct ContactParameters {
    // Point contact stiffness. If nullopt, this property is not added to the
    // model.
    std::optional<double> point_stiffness;
    // Hydroelastic modulus. If nullopt, this property is not added to the
    // model.
    std::optional<double> hydro_modulus;
    // Relaxation time constant τ is used to setup the linear dissipation model
    // where dissipation is c = τ⋅k, with k the point pair stiffness.
    // If nullopt, no dissipation is specified, i.e. the corresponding
    // ProximityProperties will not have dissipation defined.
    std::optional<double> relaxation_time;
    // Coefficient of dynamic friction.
    double friction_coefficient{nan()};
  };

  // Parameters used to setup the model of a compliant sphere.
  struct SphereParameters {
    const std::string name;
    double mass;
    double radius;
    // No contact geometry is registered if nullopt.
    std::optional<ContactParameters> contact_parameters;
  };

  // Sets up this fixture to have:
  //   - A rigid-hydroelastic half-space for the ground.
  //   - A compliant-hydroelastic sphere on top of the ground.
  //   - A non-hydroelastic sphere on top of the first sphere.
  //   - Both spheres also have point contact compliance.
  //   - We set MultibodyPlant to use hydroelastic contact with fallback.
  //   - Sphere 1 penetrates into the ground penetration_distance_.
  //   - Sphere 1 and 2 penetrate penetration_distance_.
  //   - Velocities are zero.
  void SetupRigidGroundCompliantSphereAndNonHydroSphere(
      bool sphere1_on_prismatic_joint = false) {
    const ContactParameters compliant_contact{1.0e5, 1.0e5, 0.01, 1.0};
    const ContactParameters non_hydro_contact{1.0e5, {}, 0.01, 1.0};
    const ContactParameters rigid_hydro_contact{
        std::nullopt, std::numeric_limits<double>::infinity(), 0.0, 1.0};
    const SphereParameters sphere1_params{"Sphere1", 10.0 /* mass */,
                                          0.2 /* size */, compliant_contact};
    const SphereParameters sphere2_params{"Sphere2", 10.0 /* mass */,
                                          0.2 /* size */, non_hydro_contact};
    MakeModel(rigid_hydro_contact, sphere1_params, sphere2_params,
              sphere1_on_prismatic_joint);
  }

  void SetupFreeFloatingSpheresWithNoContact() {
    const bool sphere1_on_prismatic_joint = false;
    const SphereParameters sphere1_params{"Sphere1", 10.0 /* mass */,
                                          0.2 /* size */,
                                          std::nullopt /* no contact */};
    const SphereParameters sphere2_params{"Sphere2", 10.0 /* mass */,
                                          0.2 /* size */,
                                          std::nullopt /* no contact */};
    MakeModel(std::nullopt /* no ground */, sphere1_params, sphere2_params,
              sphere1_on_prismatic_joint);
  }

  // Sets up a model with two spheres and the ground.
  // Spheres are defined by their SphereParameters. Ground contact is defined by
  // `ground_params`, no ground is added if std::nullopt.
  // Sphere 1 is mounted on a prismatic joint about the z axis if
  // sphere1_on_prismatic_joint = true.
  void MakeModel(const std::optional<ContactParameters>& ground_params,
                 const SphereParameters& sphere1_params,
                 const SphereParameters& sphere2_params,
                 bool sphere1_on_prismatic_joint = false) {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, time_step_);

    // Add model of the ground.
    if (ground_params) {
      const ProximityProperties ground_properties =
          MakeProximityProperties(*ground_params);
      plant_->RegisterCollisionGeometry(plant_->world_body(), RigidTransformd(),
                                        geometry::HalfSpace(),
                                        "ground_collision", ground_properties);
      const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
      plant_->RegisterVisualGeometry(plant_->world_body(), RigidTransformd(),
                                     geometry::HalfSpace(), "ground_visual",
                                     green);
    }

    // Add models of the spheres.
    sphere1_ = &AddSphere(sphere1_params);
    sphere2_ = &AddSphere(sphere2_params);

    // Model with sphere 1 mounted on a prismatic joint with lower limits.
    if (sphere1_on_prismatic_joint) {
      slider1_ = &plant_->AddJoint<PrismaticJoint>(
          "Sphere1Slider", plant_->world_body(), std::nullopt, *sphere1_,
          std::nullopt, Vector3<double>::UnitZ(),
          sphere1_params.radius /* lower limit */);
    }

    plant_->mutable_gravity_field().set_gravity_vector(-gravity_ *
                                                       Vector3d::UnitZ());

    plant_->set_contact_model(
        drake::multibody::ContactModel::kHydroelasticWithFallback);

    plant_->Finalize();
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    contact_manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));

    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());

    SetContactState(sphere1_params, sphere2_params);
  }

  // Sphere 1 is set on top of the ground and sphere 2 sits right on top of
  // sphere 1. We set the state of the model so that sphere 1 penetrates into
  // the ground a distance penetration_distance_ and so that sphere 1 and 2 also
  // interpenetrate a distance penetration_distance_.
  // MakeModel() must have already been called.
  void SetContactState(const SphereParameters& sphere1_params,
                       const SphereParameters& sphere2_params) const {
    DRAKE_DEMAND(plant_ != nullptr);
    const double sphere1_com_z = sphere1_params.radius - penetration_distance_;
    if (slider1_ != nullptr) {
      slider1_->set_translation(plant_context_, sphere1_com_z);
    } else {
      const RigidTransformd X_WB1(Vector3d(0, 0, sphere1_com_z));
      plant_->SetFreeBodyPose(plant_context_, *sphere1_, X_WB1);
    }
    const double sphere2_com_z = 2.0 * sphere1_params.radius +
                                 sphere2_params.radius -
                                 2.0 * penetration_distance_;
    const RigidTransformd X_WB2(Vector3d(0, 0, sphere2_com_z));
    plant_->SetFreeBodyPose(plant_context_, *sphere2_, X_WB2);
  }

  // This function makes a model with the specified sphere 1 and sphere 2
  // properties and verifies the resulting contact pairs.
  // In this model sphere 1 always interacts with the ground using the
  // hydroelastic contact model.
  // Point contact stiffness must be provided for both spheres in
  // sphere1_point_params and sphere2_point_params.
  void VerifyDiscreteContactPairs(
      const ContactParameters& sphere1_point_params,
      const ContactParameters& sphere2_point_params) {
    // This test is specific to point contact. Both spheres must have point
    // contact properties.
    DRAKE_DEMAND(sphere1_point_params.point_stiffness.has_value());
    DRAKE_DEMAND(sphere2_point_params.point_stiffness.has_value());

    ContactParameters sphere1_contact_params = sphere1_point_params;
    sphere1_contact_params.hydro_modulus = 1.0e5;
    const ContactParameters sphere2_contact_params = sphere2_point_params;

    const ContactParameters hard_hydro_contact{
        std::nullopt, std::numeric_limits<double>::infinity(), 0.0, 1.0};
    const SphereParameters sphere1_params{
        "Sphere1", 10.0 /* mass */, 0.2 /* size */, sphere1_contact_params};
    const SphereParameters sphere2_params{
        "Sphere2", 10.0 /* mass */, 0.2 /* size */, sphere2_contact_params};

    // Soft sphere/hard ground.
    MakeModel(hard_hydro_contact, sphere1_params, sphere2_params);

    const std::vector<PenetrationAsPointPair<double>>& point_pair_penetrations =
        plant_->EvalPointPairPenetrations(*plant_context_);
    const int num_point_pairs = point_pair_penetrations.size();
    const std::vector<geometry::ContactSurface<double>>& surfaces =
        EvalContactSurfaces(*plant_context_);
    ASSERT_EQ(surfaces.size(), 1u);
    const int num_hydro_pairs = surfaces[0].num_faces();

    // In these tests ContactParameters::relaxation_time = nullopt
    // indicates we want to build a model for which we forgot to specify the
    // relaxation time in ProximityProperties. Here we verify this is not
    // required by the manager, since the manager specifies a default value.
    if (!sphere1_point_params.relaxation_time.has_value() ||
        !sphere2_point_params.relaxation_time.has_value()) {
      EXPECT_NO_THROW(EvalDiscreteContactPairs(*plant_context_));
      return;
    }

    // Verify that the manager throws an exception if a negative relaxation
    // times is provided.
    if (*sphere1_point_params.relaxation_time < 0 ||
        *sphere2_point_params.relaxation_time < 0) {
      DRAKE_EXPECT_THROWS_MESSAGE(EvalDiscreteContactPairs(*plant_context_),
                                  "Relaxation time must be non-negative "
                                  "and relaxation_time = .* was "
                                  "provided. For geometry .* on body .*.");
      return;
    }

    const std::vector<DiscreteContactPair<double>>& pairs =
        EvalDiscreteContactPairs(*plant_context_);
    EXPECT_EQ(pairs.size(), num_point_pairs + num_hydro_pairs);

    const GeometryId sphere2_geometry =
        plant_->GetCollisionGeometriesForBody(*sphere2_)[0];

    for (int i = 0; i < static_cast<int>(pairs.size()); ++i) {
      const DiscreteContactPair<double>& point_pair = pairs[i];

      if (i == 0) {
        // Unit tests for point contact only.
        // Here we use our knowledge that we always place point contact pairs
        // followed by hydroelastic contact pairs.
        const double phi_expected = -penetration_distance_;
        // The geometry engine computes absolute values of penetration to
        // machine epsilon (at least for sphere vs. sphere contact).
        EXPECT_NEAR(point_pair.phi0, phi_expected, kEps);

        const double k1 = *sphere1_contact_params.point_stiffness;
        const double k2 = *sphere2_contact_params.point_stiffness;
        const double stiffness_expected = (k1 * k2) / (k1 + k2);
        EXPECT_NEAR(point_pair.stiffness, stiffness_expected,
                    kEps * stiffness_expected);

        // Verify contact location.
        const double pz_WS1 =
            plant_->GetFreeBodyPose(*plant_context_, *sphere1_)
                .translation()
                .z();
        const double pz_WC = -k2 / (k1 + k2) * penetration_distance_ + pz_WS1 +
                             sphere1_params.radius;
        EXPECT_NEAR(point_pair.p_WC.z(), pz_WC, 1.0e-14);
      }

      // Unit tests for both point and hydroelastic discrete pairs.
      const int sign = point_pair.id_A == sphere2_geometry ? 1 : -1;
      const Vector3d normal_expected = sign * Vector3d::UnitZ();
      EXPECT_TRUE(CompareMatrices(point_pair.nhat_BA_W, normal_expected));

      // Verify dissipation.
      const double tau1 = *sphere1_contact_params.relaxation_time;
      const double tau2 = i == 0 ? *sphere2_contact_params.relaxation_time
                                 : *hard_hydro_contact.relaxation_time;
      const double tau_expected = tau1 + tau2;
      EXPECT_NEAR(point_pair.dissipation_time_scale, tau_expected,
                  kEps * tau_expected);

      // Verify friction.
      const double mu1 = sphere1_contact_params.friction_coefficient;
      const double mu2 = i == 0 ? sphere2_contact_params.friction_coefficient
                                : hard_hydro_contact.friction_coefficient;
      const double mu_expected = 2.0 * (mu1 * mu2) / (mu1 + mu2);
      EXPECT_NEAR(point_pair.friction_coefficient, mu_expected,
                  kEps * mu_expected);
    }
  }

  // In the functions below we use CompliantContactManagerTest to provide access
  // to private functions for unit testing.

  const internal::MultibodyTreeTopology& topology() const {
    return CompliantContactManagerTest::topology(*contact_manager_);
  }

  const std::vector<geometry::ContactSurface<double>>& EvalContactSurfaces(
      const Context<double>& context) const {
    return CompliantContactManagerTest::EvalContactSurfaces(*contact_manager_,
                                                            context);
  }

  const std::vector<DiscreteContactPair<double>>& EvalDiscreteContactPairs(
      const Context<double>& context) const {
    return CompliantContactManagerTest::EvalDiscreteContactPairs(
        *contact_manager_, context);
  }

  std::vector<ContactPairKinematics<double>> CalcContactKinematics(
      const Context<double>& context) const {
    return CompliantContactManagerTest::CalcContactKinematics(*contact_manager_,
                                                              context);
  }

  const ContactProblemCache<double>& EvalContactProblemCache(
      const Context<double>& context) const {
    return CompliantContactManagerTest::EvalContactProblemCache(
        *contact_manager_, context);
  }

  VectorXd CalcFreeMotionVelocities(const Context<double>& context) const {
    VectorXd v_star(plant_->num_velocities());
    return CompliantContactManagerTest::CalcFreeMotionVelocities(
        *contact_manager_, context);
  }

  std::vector<MatrixXd> CalcLinearDynamicsMatrix(
      const Context<double>& context) const {
    return CompliantContactManagerTest::CalcLinearDynamicsMatrix(
        *contact_manager_, context);
  }

  void PackContactSolverResults(
      const contact_solvers::internal::SapContactProblem<double>& problem,
      int num_contacts,
      const contact_solvers::internal::SapSolverResults<double>& sap_results,
      contact_solvers::internal::ContactSolverResults<double>* contact_results)
      const {
    CompliantContactManagerTest::PackContactSolverResults(
        *contact_manager_, problem, num_contacts, sap_results, contact_results);
  }

  // Returns the Jacobian J_AcBc_C. This method takes the Jacobian blocks
  // evaluated with EvalContactJacobianCache() and assembles them into a dense
  // Jacobian matrix.
  MatrixXd CalcDenseJacobianMatrixInContactFrame(
      const std::vector<ContactPairKinematics<double>>& contact_kinematics)
      const {
    const int nc = contact_kinematics.size();
    MatrixXd J_AcBc_C(3 * nc, contact_manager_->plant().num_velocities());
    J_AcBc_C.setZero();
    for (int i = 0; i < nc; ++i) {
      const int row_offset = 3 * i;
      const ContactPairKinematics<double>& pair_kinematics =
          contact_kinematics[i];
      for (const ContactPairKinematics<double>::JacobianTreeBlock&
               tree_jacobian : pair_kinematics.jacobian) {
        // If added to the Jacobian, it must have a valid index.
        EXPECT_TRUE(tree_jacobian.tree.is_valid());
        const int col_offset =
            topology().tree_velocities_start(tree_jacobian.tree);
        const int tree_nv = topology().num_tree_velocities(tree_jacobian.tree);
        J_AcBc_C.block(row_offset, col_offset, 3, tree_nv) = tree_jacobian.J;
      }
    }
    return J_AcBc_C;
  }

  // Helper method to test EvalContactJacobianCache().
  // Returns the Jacobian J_AcBc_W.
  MatrixXd CalcDenseJacobianMatrixInWorldFrame(
      const std::vector<ContactPairKinematics<double>>& contact_kinematics)
      const {
    const int nc = contact_kinematics.size();
    const MatrixXd J_AcBc_C =
        CalcDenseJacobianMatrixInContactFrame(contact_kinematics);
    MatrixXd J_AcBc_W(3 * nc, contact_manager_->plant().num_velocities());
    J_AcBc_W.setZero();
    for (int i = 0; i < nc; ++i) {
      const ContactPairKinematics<double>& pair_kinematics =
          contact_kinematics[i];
      J_AcBc_W.middleRows<3>(3 * i) =
          pair_kinematics.R_WC.matrix() * J_AcBc_C.middleRows<3>(3 * i);
    }
    return J_AcBc_W;
  }

 protected:
  // Arbitrary positive value so that the model is discrete.
  double time_step_{0.001};

  const double gravity_{10.0};  // Acceleration of gravity, in m/s².

  // Default penetration distance. The configuration of the model is set so that
  // ground/sphere1 and sphere1/sphere2 interpenetrate by this amount.
  const double penetration_distance_{1.0e-3};

  std::unique_ptr<systems::Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* sphere1_{nullptr};
  const RigidBody<double>* sphere2_{nullptr};
  const PrismaticJoint<double>* slider1_{nullptr};
  CompliantContactManager<double>* contact_manager_{nullptr};
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};

 private:
  // Helper to add a spherical body into the model.
  const RigidBody<double>& AddSphere(const SphereParameters& params) {
    // Add rigid body.
    const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
    const UnitInertia<double> G_BBcm_B =
        UnitInertia<double>::SolidSphere(params.radius);
    const SpatialInertia<double> M_BBcm_B(params.mass, p_BoBcm_B, G_BBcm_B);
    const RigidBody<double>& body = plant_->AddRigidBody(params.name, M_BBcm_B);

    // Add collision geometry.
    if (params.contact_parameters) {
      const geometry::Sphere shape(params.radius);
      const ProximityProperties properties =
          MakeProximityProperties(*params.contact_parameters);
      plant_->RegisterCollisionGeometry(body, RigidTransformd(), shape,
                                        params.name + "_collision", properties);
    }

    return body;
  }

  // Utility to make ProximityProperties from ContactParameters.
  // params.relaxation_time is ignored if nullopt.
  static ProximityProperties MakeProximityProperties(
      const ContactParameters& params) {
    DRAKE_DEMAND(params.point_stiffness || params.hydro_modulus);
    ProximityProperties properties;
    if (params.hydro_modulus) {
      if (params.hydro_modulus == std::numeric_limits<double>::infinity()) {
        geometry::AddRigidHydroelasticProperties(/* resolution_hint */ 1.0,
                                                 &properties);
      } else {
        geometry::AddCompliantHydroelasticProperties(
            /* resolution_hint */ 1.0, *params.hydro_modulus, &properties);
      }
      // N.B. Add the slab thickness property by default so that we can model a
      // half space (either compliant or rigid).
      properties.AddProperty(geometry::internal::kHydroGroup,
                             geometry::internal::kSlabThickness, 1.0);
    }
    geometry::AddContactMaterial(
        /* "hunt_crossley_dissipation" */ {}, params.point_stiffness,
        CoulombFriction<double>(params.friction_coefficient,
                                params.friction_coefficient),
        &properties);

    if (params.relaxation_time.has_value()) {
      properties.AddProperty(geometry::internal::kMaterialGroup,
                             "relaxation_time", *params.relaxation_time);
    }
    return properties;
  }
};

// Unit test to verify discrete contact pairs computed by the manager for
// different combinations of compliance.
TEST_F(SpheresStack, VerifyDiscreteContactPairs) {
  ContactParameters soft_point_contact{1.0e3, std::nullopt, 0.01, 1.0};
  ContactParameters hard_point_contact{1.0e40, std::nullopt, 0.0, 1.0};

  // Hard sphere 1/soft sphere 2.
  VerifyDiscreteContactPairs(hard_point_contact, soft_point_contact);

  // Equally soft spheres.
  VerifyDiscreteContactPairs(soft_point_contact, soft_point_contact);

  // Soft sphere 1/hard sphere 2.
  VerifyDiscreteContactPairs(soft_point_contact, hard_point_contact);
}

TEST_F(SpheresStack, RelaxationTimeIsNotRequired) {
  ContactParameters soft_point_contact{
      1.0e3, std::nullopt,
      std::nullopt /* Dissipation not included in ProximityProperties */, 1.0};
  ContactParameters hard_point_contact{1.0e40, std::nullopt, 0.0, 1.0};

  // Hard sphere 1/soft sphere 2.
  VerifyDiscreteContactPairs(hard_point_contact, soft_point_contact);

  // Equally soft spheres.
  VerifyDiscreteContactPairs(soft_point_contact, soft_point_contact);

  // Soft sphere 1/hard sphere 2.
  VerifyDiscreteContactPairs(soft_point_contact, hard_point_contact);
}

TEST_F(SpheresStack, RelaxationTimeMustBePositive) {
  ContactParameters soft_point_contact{
      1.0e3, std::nullopt, -1.0 /* Negative dissipation timescale */, 1.0};
  ContactParameters hard_point_contact{1.0e40, std::nullopt, 0.0, 1.0};

  // Hard sphere 1/soft sphere 2.
  VerifyDiscreteContactPairs(hard_point_contact, soft_point_contact);

  // Equally soft spheres.
  VerifyDiscreteContactPairs(soft_point_contact, soft_point_contact);

  // Soft sphere 1/hard sphere 2.
  VerifyDiscreteContactPairs(soft_point_contact, hard_point_contact);
}

// Unit test to verify discrete contact pairs computed by the manager for
// rigid-compliant hydroelastic contact with point-contact fall back.
TEST_F(SpheresStack,
       VerifyDiscreteContactPairsFromRigidCompliantHydroelasticContact) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();

  const std::vector<PenetrationAsPointPair<double>>& point_pairs =
      plant_->EvalPointPairPenetrations(*plant_context_);
  const int num_point_pairs = point_pairs.size();
  EXPECT_EQ(num_point_pairs, 1);
  const std::vector<DiscreteContactPair<double>>& pairs =
      EvalDiscreteContactPairs(*plant_context_);

  const std::vector<geometry::ContactSurface<double>>& surfaces =
      EvalContactSurfaces(*plant_context_);
  ASSERT_EQ(surfaces.size(), 1);
  EXPECT_EQ(pairs.size(), surfaces[0].num_faces() + num_point_pairs);
}

// Unit test to verify the computation of the contact Jacobian.
TEST_F(SpheresStack, CalcContactKinematics) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();
  const double radius = 0.2;  // Spheres's radii in the default setup.

  const std::vector<DiscreteContactPair<double>>& pairs =
      EvalDiscreteContactPairs(*plant_context_);
  const std::vector<ContactPairKinematics<double>> contact_kinematics =
      CalcContactKinematics(*plant_context_);
  const MatrixXd J_AcBc_W =
      CalcDenseJacobianMatrixInWorldFrame(contact_kinematics);

  // Arbitrary velocity of sphere 1.
  const Vector3d v_WS1(1, 2, 3);
  const Vector3d w_WS1(4, 5, 6);
  const SpatialVelocity<double> V_WS1(w_WS1, v_WS1);

  // Arbitrary velocity of sphere 2.
  const Vector3d v_WS2(7, 8, 9);
  const Vector3d w_WS2(10, 11, 12);
  const SpatialVelocity<double> V_WS2(w_WS2, v_WS2);

  plant_->SetFreeBodySpatialVelocity(plant_context_, *sphere1_, V_WS1);
  plant_->SetFreeBodySpatialVelocity(plant_context_, *sphere2_, V_WS2);
  const VectorXd v = plant_->GetVelocities(*plant_context_);

  const GeometryId sphere1_geometry =
      plant_->GetCollisionGeometriesForBody(*sphere1_)[0];

  // Verify contact Jacobian for the point pair.
  // For this model we know the first entry corresponds to the single point pair
  // between sphere 1 and sphere 2.
  {
    // For the default setup both spheres are equally compliant and therefore
    // the contact point C lies right in the middle.
    const Vector3d p_S1C_W(0, 0, radius - penetration_distance_ / 2.0);
    const Vector3d p_S2C_W(0, 0, -(radius - penetration_distance_ / 2.0));

    // Compute expected contact point velocity.
    const Vector3d v_WS1c = V_WS1.Shift(p_S1C_W).translational();
    const Vector3d v_WS2c = V_WS2.Shift(p_S2C_W).translational();
    const Vector3d expected_v_S1cS2c_W = v_WS2c - v_WS1c;

    const int sign = pairs[0].id_A == sphere1_geometry ? 1 : -1;
    const MatrixXd J_S1cS2c_W = sign * J_AcBc_W.topRows(3);
    const Vector3d v_S1cS2c_W = J_S1cS2c_W * v;
    EXPECT_TRUE(CompareMatrices(v_S1cS2c_W, expected_v_S1cS2c_W, kEps,
                                MatrixCompareType::relative));

    // Verify we loaded phi correctly.
    EXPECT_EQ(pairs[0].phi0, contact_kinematics[0].phi);
  }

  // Verify contact Jacobian for hydroelastic pairs.
  // We know hydroelastic pairs come after point pairs.
  {
    const Vector3d p_WS1(0, 0, radius - penetration_distance_);
    for (size_t q = 1; q < pairs.size(); ++q) {
      const Vector3d& p_WC = pairs[q].p_WC;
      const Vector3d p_S1C_W = p_WC - p_WS1;
      const Vector3d expected_v_WS1c = V_WS1.Shift(p_S1C_W).translational();
      const int sign = pairs[q].id_B == sphere1_geometry ? 1 : -1;
      const MatrixXd J_WS1c_W =
          sign * J_AcBc_W.block(3 * q, 0, 3, plant_->num_velocities());
      const Vector3d v_WS1c_W = J_WS1c_W * v;
      EXPECT_TRUE(CompareMatrices(v_WS1c_W, expected_v_WS1c, kEps,
                                  MatrixCompareType::relative));

      // Verify we loaded phi correctly.
      EXPECT_EQ(pairs[q].phi0, contact_kinematics[q].phi);
    }
  }
}

// This test verifies that the SapContactProblem built by the manager is
// consistent with the contact kinematics computed with CalcContactKinematics().
TEST_F(SpheresStack, EvalContactProblemCache) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();
  const ContactProblemCache<double>& problem_cache =
      EvalContactProblemCache(*plant_context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;
  const std::vector<drake::math::RotationMatrix<double>>& R_WC =
      problem_cache.R_WC;

  const std::vector<DiscreteContactPair<double>>& pairs =
      EvalDiscreteContactPairs(*plant_context_);
  const int num_contacts = pairs.size();

  // Verify sizes.
  EXPECT_EQ(problem.num_cliques(), topology().num_trees());
  EXPECT_EQ(problem.num_velocities(), plant_->num_velocities());
  EXPECT_EQ(problem.num_constraints(), num_contacts);
  EXPECT_EQ(problem.num_constraint_equations(), 3 * num_contacts);
  EXPECT_EQ(problem.time_step(), plant_->time_step());
  ASSERT_EQ(R_WC.size(), num_contacts);

  // Verify dynamics data.
  const VectorXd& v_star = CalcFreeMotionVelocities(*plant_context_);
  const std::vector<MatrixXd>& A = CalcLinearDynamicsMatrix(*plant_context_);
  EXPECT_EQ(problem.v_star(), v_star);
  EXPECT_EQ(problem.dynamics_matrix(), A);

  // Verify each of the contact constraints.
  const std::vector<ContactPairKinematics<double>> contact_kinematics =
      CalcContactKinematics(*plant_context_);
  for (size_t i = 0; i < contact_kinematics.size(); ++i) {
    const DiscreteContactPair<double>& discrete_pair = pairs[i];
    const ContactPairKinematics<double>& pair_kinematics =
        contact_kinematics[i];
    const auto* constraint =
        dynamic_cast<const SapFrictionConeConstraint<double>*>(
            &problem.get_constraint(i));
    // In this test we do know all constraints are contact constraints.
    ASSERT_NE(constraint, nullptr);
    EXPECT_EQ(constraint->constraint_function(),
              Vector3d(0., 0., pair_kinematics.phi));
    EXPECT_EQ(constraint->num_cliques(), pair_kinematics.jacobian.size());
    EXPECT_EQ(constraint->first_clique(), pair_kinematics.jacobian[0].tree);
    EXPECT_EQ(constraint->first_clique_jacobian(),
              pair_kinematics.jacobian[0].J);
    if (constraint->num_cliques() == 2) {
      EXPECT_EQ(constraint->second_clique(), pair_kinematics.jacobian[1].tree);
      EXPECT_EQ(constraint->second_clique_jacobian(),
                pair_kinematics.jacobian[1].J);
    }
    EXPECT_EQ(constraint->parameters().mu, discrete_pair.friction_coefficient);
    EXPECT_EQ(constraint->parameters().stiffness, discrete_pair.stiffness);
    EXPECT_EQ(constraint->parameters().dissipation_time_scale,
              discrete_pair.dissipation_time_scale);
    // These two parameters, beta and sigma, are for now hard-code in the
    // manager to these values. Here we simply tests they are consistent with
    // those hard-coded values.
    EXPECT_EQ(constraint->parameters().beta, 1.0);
    EXPECT_EQ(constraint->parameters().sigma, 1.0e-3);

    // Verify contact frame orientation matrix R_WC.
    EXPECT_EQ(R_WC[i].matrix(), pair_kinematics.R_WC.matrix());
  }
}

// Verifies the correctness of the computation of free motion velocities when
// external forces are applied.
TEST_F(SpheresStack, CalcFreeMotionVelocitiesWithExternalForces) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();

  // We set an arbitrary non-zero external force to the plant to verify it gets
  // properly applied as part of the computation.
  const int nv = plant_->num_velocities();
  const VectorXd tau = VectorXd::LinSpaced(nv, 1.0, 2.0);
  plant_->get_applied_generalized_force_input_port().FixValue(plant_context_,
                                                              tau);

  // Set arbitrary non-zero velocities.
  const VectorXd v0 = VectorXd::LinSpaced(nv, 2.0, 3.0);
  plant_->SetVelocities(plant_context_, v0);

  // Since the spheres's frames are located at their COM and since their
  // rotational inertias are triaxially symmetric, the Coriolis term is zero.
  // Therefore the momentum equation reduces to: M * (v-v0)/dt = tau_g + tau.
  const double dt = plant_->time_step();
  const VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*plant_context_);
  MatrixXd M(nv, nv);
  plant_->CalcMassMatrix(*plant_context_, &M);
  const VectorXd v_expected = v0 + dt * M.ldlt().solve(tau_g + tau);

  // Compute the velocities the system would have next time step in the absence
  // of constraints.
  const VectorXd v_star = CalcFreeMotionVelocities(*plant_context_);

  EXPECT_TRUE(
      CompareMatrices(v_star, v_expected, kEps, MatrixCompareType::relative));
}

// Verifies that joint limit forces are applied.
TEST_F(SpheresStack, CalcFreeMotionVelocitiesWithJointLimits) {
  // In this model sphere 1 is attached to the world by a prismatic joint with
  // lower limit z = 0.
  const bool sphere1_on_prismatic_joint = true;
  SetupRigidGroundCompliantSphereAndNonHydroSphere(sphere1_on_prismatic_joint);

  const int nv = plant_->num_velocities();

  // Set arbitrary non-zero velocities.
  const VectorXd non_zero_vs = VectorXd::LinSpaced(nv, 2.0, 3.0);
  plant_->SetVelocities(plant_context_, non_zero_vs);

  // The slider velocity is set to be negative to ensure the joint limit is
  // active. That is, the position was set to be below the lower limit and in
  // addition it is decreasing.
  slider1_->set_translation_rate(plant_context_, -1.0);

  // Initial velocities.
  const VectorXd v0 = plant_->GetVelocities(*plant_context_);

  // Compute slider1's velocity in the absence of joint limits. This is
  // equivalent to computing the free motion velocities before constraints are
  // applied.
  const VectorXd v_expected = CalcFreeMotionVelocities(*plant_context_);
  const double v_slider_no_limits = v_expected(slider1_->velocity_start());

  // Compute slider1's velocity with constraints applied. This corresponds to
  // the full discrete update computation.
  ContactSolverResults<double> contact_results;
  contact_manager_->CalcContactSolverResults(*plant_context_, &contact_results);
  const VectorXd v_star = contact_results.v_next;
  const double v_slider_star = v_star(slider1_->velocity_start());

  // While other solver specific tests verify the correctness of joint limit
  // forces, this test is simply limited to verifying the manager applied them.
  // Therefore we only check the force limits have the effect of making the
  // slider velocity larger than if not present.
  EXPECT_GT(v_slider_star, v_slider_no_limits);
}

TEST_F(SpheresStack, CalcLinearDynamicsMatrix) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();
  const std::vector<MatrixXd> A = CalcLinearDynamicsMatrix(*plant_context_);
  const int nv = plant_->num_velocities();
  MatrixXd Adense = MatrixXd::Zero(nv, nv);
  for (TreeIndex t(0); t < topology().num_trees(); ++t) {
    const int tree_start = topology().tree_velocities_start(t);
    const int tree_nv = topology().num_tree_velocities(t);
    Adense.block(tree_start, tree_start, tree_nv, tree_nv) = A[t];
  }
  MatrixXd Aexpected(nv, nv);
  plant_->CalcMassMatrix(*plant_context_, &Aexpected);
  EXPECT_TRUE(
      CompareMatrices(Adense, Aexpected, kEps, MatrixCompareType::relative));
}

// Here we test the function CompliantContactManager::PackContactSolverResults()
// which takes SapSolverResults and packs them into ContactSolverResults as
// consumed by MultibodyPlant.
TEST_F(SpheresStack, PackContactSolverResults) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();

  // We form an arbitrary set of SAP results consistent with the contact
  // kinematics for the configuration of our model.
  const std::vector<ContactPairKinematics<double>> contact_kinematics =
      CalcContactKinematics(*plant_context_);
  const int num_contacts = contact_kinematics.size();
  const int nv = plant_->num_velocities();
  SapSolverResults<double> sap_results;
  sap_results.Resize(nv, 3 * num_contacts);
  sap_results.v = VectorXd::LinSpaced(nv, -3.0, 14.0);
  sap_results.gamma = VectorXd::LinSpaced(3 * num_contacts, -12.0, 8.0);
  sap_results.vc = VectorXd::LinSpaced(3 * num_contacts, -1.0, 11.0);
  // Not used to pack contact results.
  sap_results.j = VectorXd::Constant(nv, NAN);

  // Pack SAP results into contact results.
  const SapContactProblem<double>& sap_problem =
      *EvalContactProblemCache(*plant_context_).sap_problem;
  ContactSolverResults<double> contact_results;
  PackContactSolverResults(sap_problem, num_contacts, sap_results,
                           &contact_results);

  // Verify against expected values.
  VectorXd gamma(3 * num_contacts);
  MergeNormalAndTangent(contact_results.fn, contact_results.ft, &gamma);
  gamma *= plant_->time_step();
  EXPECT_TRUE(CompareMatrices(gamma, sap_results.gamma, kEps,
                              MatrixCompareType::relative));
  VectorXd vc(3 * num_contacts);
  MergeNormalAndTangent(contact_results.vn, contact_results.vt, &vc);
  EXPECT_TRUE(
      CompareMatrices(vc, sap_results.vc, kEps, MatrixCompareType::relative));
  const MatrixXd J_AcBc_C =
      CalcDenseJacobianMatrixInContactFrame(contact_kinematics);
  const VectorXd tau_expected =
      J_AcBc_C.transpose() * sap_results.gamma / plant_->time_step();
  EXPECT_TRUE(CompareMatrices(contact_results.tau_contact, tau_expected,
                              2.0 * kEps, MatrixCompareType::relative));
}

// Unit test that the manager throws an exception whenever SAP fails to
// converge.
TEST_F(SpheresStack, SapFailureException) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();
  ContactSolverResults<double> contact_results;
  // To trigger SAP's failure, we limit the maximum number of iterations to
  // zero.
  SapSolverParameters parameters;
  parameters.max_iterations = 0;
  contact_manager_->set_sap_solver_parameters(parameters);
  DRAKE_EXPECT_THROWS_MESSAGE(contact_manager_->CalcContactSolverResults(
                                  *plant_context_, &contact_results),
                              "The SAP solver failed to converge(.|\n)*");
}

// The purpose of this test is not to verify the correctness of the computation,
// but rather to verify that data flows correctly. That is, that
// CalcContactSolverResults() loads the expected computation into the contact
// results.
TEST_F(SpheresStack, DoCalcContactSolverResults) {
  SetupRigidGroundCompliantSphereAndNonHydroSphere();
  // N.B. We make sure both the manager and the manual invocations of the SAP
  // solver in this test both use the same set of parameters.
  SapSolverParameters params;  // Default set of parameters.
  contact_manager_->set_sap_solver_parameters(params);
  ContactSolverResults<double> contact_results;
  contact_manager_->CalcContactSolverResults(*plant_context_, &contact_results);

  // Generate contact results here locally to verify that
  // CalcContactSolverResults() loads them properly.
  const SapContactProblem<double>& sap_problem =
      *EvalContactProblemCache(*plant_context_).sap_problem;
  const int num_contacts = sap_problem.num_constraints();  // Only contacts.
  SapSolver<double> sap;
  sap.set_parameters(params);
  SapSolverResults<double> sap_results;
  const SapSolverStatus status = sap.SolveWithGuess(
      sap_problem, plant_->GetVelocities(*plant_context_), &sap_results);
  ASSERT_EQ(status, SapSolverStatus::kSuccess);

  ContactSolverResults<double> contact_results_expected;
  PackContactSolverResults(sap_problem, num_contacts, sap_results,
                           &contact_results_expected);

  // Verify the expected result.
  EXPECT_TRUE(CompareMatrices(contact_results.v_next,
                              contact_results_expected.v_next, kEps,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.fn, contact_results_expected.fn,
                              kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.ft, contact_results_expected.ft,
                              kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.vn, contact_results_expected.vn,
                              kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.vt, contact_results_expected.vt,
                              kEps, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(contact_results.tau_contact,
                              contact_results_expected.tau_contact, kEps,
                              MatrixCompareType::relative));
}

// The purpose of this test is to verify that CalcDiscreteValues() loads state
// updates correctly. This can be verified even with a simple case without
// contact, for which we can manually compute the solution. The correctness of
// contact results in configurations with contact is tested elsewhere.
// In this case we setup two free falling spheres for which we can compute the
// state update by hand, under the assumption the manager is using a symplectic
// Euler scheme. This assumping might need to be updated in the future when
// other schemes are supported.
TEST_F(SpheresStack, DoCalcDiscreteValues) {
  SetupFreeFloatingSpheresWithNoContact();

  // Both spheres accelerate from zero velocity in a single time step with
  // gravity along the z-axis.
  const Vector3d v_WS(0., 0., -gravity_ * plant_->time_step());
  const SpatialVelocity<double> V_WS(Vector3d::Zero(), v_WS);

  // Positions at the previous time step.
  const Vector3d& p_WS10 =
      plant_->EvalBodyPoseInWorld(*plant_context_, *sphere1_).translation();
  const Vector3d& p_WS20 =
      plant_->EvalBodyPoseInWorld(*plant_context_, *sphere2_).translation();

  // The manager uses a symplectic update of the positions. For this case then
  // we know the positions are:
  const Vector3d p_WS1 = p_WS10 + plant_->time_step() * v_WS;
  const Vector3d p_WS2 = p_WS20 + plant_->time_step() * v_WS;

  // Create a new context for the next state.
  auto next_context = plant_->CreateDefaultContext();

  // In this simple setup only positions change since there is no angular
  // velocities nor external torques.
  plant_->SetFreeBodyPoseInWorldFrame(next_context.get(), *sphere1_,
                                      RigidTransformd(p_WS1));
  plant_->SetFreeBodyPoseInWorldFrame(next_context.get(), *sphere2_,
                                      RigidTransformd(p_WS2));
  plant_->SetFreeBodySpatialVelocity(next_context.get(), *sphere1_, V_WS);
  plant_->SetFreeBodySpatialVelocity(next_context.get(), *sphere2_, V_WS);

  // Obtain the expected value of the state.
  const VectorXd& x_next_expected = next_context->get_discrete_state().value();

  // Perform a discrete update, on the original context_.
  std::unique_ptr<systems::DiscreteValues<double>> updates =
      diagram_->AllocateDiscreteVariables();
  contact_manager_->CalcDiscreteValues(*plant_context_, updates.get());
  ASSERT_EQ(updates->num_groups(), 1);
  const VectorXd x_next = updates->value();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(x_next, x_next_expected, kEps,
                              MatrixCompareType::relative));
}

// CompliantContactManager implements a workaround for issue #12786 which might
// lead to undetected algebraic loops in the systems framework. Therefore
// CompliantContactManager implements an internal algebraic loop detection to
// properly warn users. This should go away as issue #12786 is resolved. This
// test verifies the algebraic loop detection logic.
class AlgebraicLoopDetection : public ::testing::Test {
 public:
  // Makes a system containing a multibody plant. When with_algebraic_loop =
  // true the model includes a feedback system that creates an algebraic loop.
  void MakeDiagram(bool with_algebraic_loop) {
    systems::DiagramBuilder<double> builder;
    plant_ = builder.AddSystem<MultibodyPlant>(1.0e-3);
    plant_->Finalize();
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));

    systems::System<double>* feedback{nullptr};
    if (with_algebraic_loop) {
      // We intentionally create an algebraic loop by placing a pass through
      // system between the contact forces output and the input forces. This
      // test is based on a typical user story: a user wants to write a
      // controller that uses the estimated forces as input to the controller.
      // For instance, the controller could implement force feedback for
      // grasping. To simplify the model, a user might choose to emulate a real
      // sensor or force estimator by connecting the output forces from the
      // plant straight into the controller, creating an algebraic loop.
      feedback = builder.AddSystem<PassThrough>(plant_->num_velocities());
    } else {
      // A more realistic model would include a force estimator, that most
      // likely would introduce state and break the algebraic loop. Another
      // option would be to introduce a delay between the force output ant the
      // controller, effectively modeling a delay in the measured signal. Here
      // we emulate one of these strategies using a zero-order-hold (ZOH) system
      // to add feedback. This will not create an algebraic loop.
      // N.B. The discrete period of the ZOH does not necessarily need to match
      // that of the plant. This example makes them different to illustrate this
      // point.
      feedback =
          builder.AddSystem<ZeroOrderHold>(2.0e-4, plant_->num_velocities());
    }
    builder.Connect(plant_->get_generalized_contact_forces_output_port(
                        default_model_instance()),
                    feedback->get_input_port(0));
    builder.Connect(feedback->get_output_port(0),
                    plant_->get_applied_generalized_force_input_port());
    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  }

  void VerifyLoopIsDetected() const {
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant_
            ->get_generalized_contact_forces_output_port(
                default_model_instance())
            .Eval(*plant_context_),
        "Algebraic loop detected.*");
  }

  void VerifyNoLoopIsDetected() const {
    EXPECT_NO_THROW(plant_
                        ->get_generalized_contact_forces_output_port(
                            default_model_instance())
                        .Eval(*plant_context_));
  }

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{nullptr};
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};
};

TEST_F(AlgebraicLoopDetection, LoopIsDetected) {
  MakeDiagram(true /* make diagram with algebraic loop */);
  VerifyLoopIsDetected();
}

TEST_F(AlgebraicLoopDetection, LoopIsDetectedWhenCachingIsDisabled) {
  MakeDiagram(true /* make diagram with algebraic loop */);
  diagram_context_->DisableCaching();
  VerifyLoopIsDetected();
}

TEST_F(AlgebraicLoopDetection, VerifyNoFalsePositives) {
  MakeDiagram(false /* make diagram with no algebraic loop */);
  // There is no loop and therefore no exception should be thrown.
  VerifyNoLoopIsDetected();
  // Since the computation is cached, we can evaluate it multiple times without
  // triggering the loop detection, as desired.
  VerifyNoLoopIsDetected();
}

TEST_F(AlgebraicLoopDetection, VerifyNoFalsePositivesWhenCachingIsDisabled) {
  MakeDiagram(false /* make diagram with no algebraic loop */);
  diagram_context_->DisableCaching();
  // There is no loop and therefore no exception should be thrown.
  VerifyNoLoopIsDetected();
  // Even if the computation is not cached, the loop detection is not triggered,
  // as desired.
  VerifyNoLoopIsDetected();
}

// Fixture to set up a Kuka iiwa arm model with a Schunk wsg gripper and welded
// to the world at the base link. This fixture is used to stress test the
// implementation of CompliantContactManager with a model of practical relevance
// to robotics. In particular, we unit test the implementation of damping and
// joint limits.
class KukaIiwaArmTests : public ::testing::Test {
 public:
  // Enum used to specify how we'd like to initialize the state for limit
  // constraints unit tests.
  enum class InitializePositionAt {
    BelowLowerLimit,  // q₀ < qₗ
    // Current position is above limit, though predicted position is below.
    AboveLowerLimitThoughPredictionBelow,  // q₀+δt⋅v₀ < qₗ < q₀
    AboveUpperLimit,                       // q₀ > qᵤ
    // Current position is below limit, though predicted position is above.
    BelowUpperLimitThoughPredictionAbove,  // q₀ < qᵤ < q₀+δt⋅v₀
    WellWithinLimits,  // Both q0 and q₀ + δt⋅v₀ are in (qₗ, qᵤ)
  };

  // Setup model of the Kuka iiwa arm with Schunk gripper and allocate context
  // resources. The model includes reflected inertias. Input ports are fixed to
  // arbitrary non-zero values.
  void SetSingleRobotModel() {
    // Only SAP supports the modeling of constraints.
    plant_.set_discrete_contact_solver(DiscreteContactSolver::kSap);

    // Load robot model from files.
    const std::vector<ModelInstanceIndex> models = SetUpArmModel(1, &plant_);
    plant_.Finalize();

    // The model has a single coupler constraint.
    EXPECT_EQ(plant_.num_constraints(), 1);

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    // Model with a single robot and gripper. A single coupler constraint to
    // model the gripper.
    EXPECT_EQ(plant_.num_constraints(), 1);

    context_ = plant_.CreateDefaultContext();
    SetArbitraryNonZeroActuation(plant_, models[0], models[1], context_.get());
    SetArbitraryState(plant_, context_.get());
  }

  // Setup model of the Kuka iiwa arm with Schunk gripper. The model includes
  // reflected inertias. The gripper is modeled with a coupler constraint.
  std::vector<ModelInstanceIndex> SetUpArmModel(
      int robot_number, MultibodyPlant<double>* plant) const {
    std::vector<ModelInstanceIndex> models =
        LoadIiwaWithGripper(robot_number, plant);
    AddInReflectedInertia(plant, models, kRotorInertias, kGearRatios);

    // Constrain the gripper fingers to be coupled.
    const ModelInstanceIndex gripper_model = models[1];
    const Joint<double>& left_finger_slider =
        plant->GetJointByName("left_finger_sliding_joint", gripper_model);
    const Joint<double>& right_finger_slider =
        plant->GetJointByName("right_finger_sliding_joint", gripper_model);
    // While for a typical gripper most likely the gear ratio is one and the
    // offset is zero, here we use an arbitrary set of values to verify later on
    // in the test that the manager created a constraint consistent with these
    // numbers.
    const ConstraintIndex next_constraint_index(plant->num_constraints());
    ConstraintIndex constraint_index =
        plant->AddCouplerConstraint(left_finger_slider, right_finger_slider,
                                    kCouplerGearRatio, kCouplerOffset);
    EXPECT_EQ(constraint_index, next_constraint_index);
    return models;
  }

  // The manager solves free motion velocities using a discrete scheme with
  // implicit joint dissipation. That is, it solves the momentum balance:
  //   m(v) = (M + dt⋅D)⋅(v-v₀) - dt⋅k(x₀)
  // where k(x₀) are all the non-constraint forces such as Coriolis terms and
  // external actuation, evaluated at the previous state x₀.
  // The dynamics matrix is defined as:
  //   A = ∂m/∂v = (M + dt⋅D)
  // This method computes A, including the contribution to implicit damping.
  MatrixXd CalcLinearDynamicsMatrixIncludingImplicitDampingContribution()
      const {
    const int nv = plant_.num_velocities();
    MatrixXd A(nv, nv);
    plant_.CalcMassMatrix(*context_, &A);
    // Include term due to the implicit treatment of dissipation.
    VectorXd damping = VectorXd::Zero(plant_.num_velocities());
    for (JointIndex joint_index(0); joint_index < plant_.num_joints();
         ++joint_index) {
      const Joint<double>& joint = plant_.get_joint(joint_index);
      if (joint.num_velocities() > 0) {  // skip welds.
        const VectorXd& joint_damping = joint.damping_vector();
        // For this model we expect 1 DOF revolute and prismatic joints only.
        EXPECT_EQ(joint_damping.size(), 1);
        EXPECT_EQ(joint.num_velocities(), 1);
        damping(joint.velocity_start()) = joint_damping(0);
      }
    }
    A.diagonal() += plant_.time_step() * damping;
    return A;
  }

  // Initializes `context` to store arbitrary values of state that abide by the
  // given specification in `limits_specification`. This allows us to test how
  // the manager adds constraints to the problem at different configurations.
  // limits_specification are indexed by joint dofs.
  // TODO(amcastro-tri): our testing strategy using these specifications can be
  // further improved as discussed in the review of #17083, tracked in #17137.
  void SetArbitraryStateWithLimitsSpecification(
      const MultibodyPlant<double>& plant,
      const std::vector<InitializePositionAt>& limits_specification,
      Context<double>* context) {
    // Arbitrary positive slop used for positions.
    const double kPositiveDeltaQ = M_PI / 10.0;
    const double dt = plant.time_step();
    VectorXd v0(plant.num_velocities());
    VectorXd q0(plant.num_positions());

    for (JointIndex joint_index(0); joint_index < plant.num_joints();
         ++joint_index) {
      const Joint<double>& joint = plant.get_joint(joint_index);

      if (joint.num_velocities() == 1) {  // skip welds in the model.
        const int v_index = joint.velocity_start();
        const InitializePositionAt limit_spec = limits_specification[v_index];
        const double ql = joint.position_lower_limits()[0];
        const double qu = joint.position_upper_limits()[0];

        double joint_q0 = 0.;
        double joint_v0 = 0.;
        switch (limit_spec) {
          case InitializePositionAt::BelowLowerLimit: {
            joint_q0 = ql > 0 ? 0.8 * ql : 1.2 * ql;
            joint_v0 = joint_index * 2.3;  // Arbitrary.
            break;
          }
          case InitializePositionAt::AboveLowerLimitThoughPredictionBelow: {
            // We initialize a state s.t. q₀+δt⋅v₀ < qₗ < q₀.
            joint_q0 = ql + kPositiveDeltaQ;
            const double qp = ql - kPositiveDeltaQ;
            joint_v0 = (qp - joint_q0) / dt;
            break;
          }
          case InitializePositionAt::AboveUpperLimit: {
            joint_q0 = qu > 0 ? 1.2 * qu : 0.8 * qu;
            joint_v0 = joint_index * 2.3;  // Arbitrary.
            break;
          }
          case InitializePositionAt::BelowUpperLimitThoughPredictionAbove: {
            // We initialize a state s.t. q₀ < qᵤ < q₀+δt⋅v₀.
            joint_q0 = qu - kPositiveDeltaQ;
            const double qp = qu + kPositiveDeltaQ;
            joint_v0 = (qp - joint_q0) / dt;
            break;
          }
          case InitializePositionAt::WellWithinLimits: {
            joint_q0 = 0.5 * (ql + qu);  // q in (ql, qu)
            joint_v0 = 0.0;
            break;
          }
          default:
            DRAKE_UNREACHABLE();
        }

        q0(v_index) = joint_q0;
        v0(v_index) = joint_v0;
      }
    }

    plant.SetPositions(context, q0);
    plant.SetVelocities(context, v0);
  }

  // Fixes all input ports to have non-zero actuation.
  void SetArbitraryNonZeroActuation(const MultibodyPlant<double>& plant,
                                    ModelInstanceIndex arm_model,
                                    ModelInstanceIndex gripper_model,
                                    Context<double>* context) const {
    const VectorX<double> tau_arm = VectorX<double>::LinSpaced(
        plant.num_actuated_dofs(arm_model), 10.0, 1000.0);
    const VectorX<double> tau_gripper = VectorX<double>::LinSpaced(
        plant.num_actuated_dofs(gripper_model), 10.0, 1000.0);
    plant.get_actuation_input_port(arm_model).FixValue(context, tau_arm);
    plant.get_actuation_input_port(gripper_model)
        .FixValue(context, tau_gripper);
  }

 private:
  std::vector<ModelInstanceIndex> LoadIiwaWithGripper(
      int robot_number, MultibodyPlant<double>* plant) const {
    DRAKE_DEMAND(plant != nullptr);
    const char kArmFilePath[] =
        "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_no_collision.urdf";

    const char kWsg50FilePath[] =
        "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf";

    Parser parser(plant);
    ModelInstanceIndex arm_model =
        parser.AddModelFromFile(FindResourceOrThrow(kArmFilePath),
                                "robot_" + std::to_string(robot_number));

    // Add the gripper.
    ModelInstanceIndex gripper_model =
        parser.AddModelFromFile(FindResourceOrThrow(kWsg50FilePath),
                                "gripper_" + std::to_string(robot_number));

    const auto& base_body = plant->GetBodyByName("base", arm_model);
    const auto& end_effector = plant->GetBodyByName("iiwa_link_7", arm_model);
    const auto& gripper_body = plant->GetBodyByName("body", gripper_model);
    plant->WeldFrames(plant->world_frame(), base_body.body_frame());
    plant->WeldFrames(end_effector.body_frame(), gripper_body.body_frame());

    return {arm_model, gripper_model};
  }

  void AddInReflectedInertia(MultibodyPlant<double>* plant,
                             const std::vector<ModelInstanceIndex>& models,
                             const VectorX<double>& rotor_inertias,
                             const VectorX<double>& gear_ratios) const {
    DRAKE_DEMAND(plant != nullptr);
    int local_joint_index = 0;
    for (JointActuatorIndex index(0); index < plant->num_actuators(); ++index) {
      JointActuator<double>& joint_actuator =
          plant->get_mutable_joint_actuator(index);
      if (std::count(models.begin(), models.end(),
                     joint_actuator.model_instance()) > 0) {
        joint_actuator.set_default_rotor_inertia(
            rotor_inertias(local_joint_index));
        joint_actuator.set_default_gear_ratio(gear_ratios(local_joint_index));
        local_joint_index++;
      }
    }
  }

  // Set arbitrary state, though within joint limits.
  void SetArbitraryState(const MultibodyPlant<double>& plant,
                         Context<double>* context) {
    for (JointIndex joint_index(0); joint_index < plant.num_joints();
         ++joint_index) {
      const Joint<double>& joint = plant.get_joint(joint_index);
      // This model only has weld, prismatic, and revolute joints.
      if (joint.type_name() == "revolute") {
        const RevoluteJoint<double>& revolute_joint =
            dynamic_cast<const RevoluteJoint<double>&>(joint);
        // Arbitrary position within position limits.
        const double ql = revolute_joint.position_lower_limit();
        const double qu = revolute_joint.position_upper_limit();
        const double w = joint_index / kNumJoints;  // Number in (0,1).
        const double q = w * ql + (1.0 - w) * qu;   // q in (ql, qu)
        revolute_joint.set_angle(context, q);
        // Arbitrary velocity.
        revolute_joint.set_angular_rate(context, 0.5 * joint_index);
      } else if (joint.type_name() == "prismatic") {
        const PrismaticJoint<double>& prismatic_joint =
            dynamic_cast<const PrismaticJoint<double>&>(joint);
        // Arbitrary position within position limits.
        const double ql = prismatic_joint.position_lower_limit();
        const double qu = prismatic_joint.position_upper_limit();
        const double w = joint_index / kNumJoints;  // Number in (0,1).
        const double q = w * ql + (1.0 - w) * qu;   // q in (ql, qu)
        prismatic_joint.set_translation(context, q);
        // Arbitrary velocity.
        prismatic_joint.set_translation_rate(context, 0.5 * joint_index);
      }
    }
  }

 protected:
  const int kNumJoints = 9;
  const double kTimeStep{0.015};
  const VectorXd kRotorInertias{VectorXd::LinSpaced(kNumJoints, 0.1, 12.0)};
  const VectorXd kGearRatios{VectorXd::LinSpaced(kNumJoints, 1.5, 100.0)};
  const double kCouplerGearRatio{-1.5};
  const double kCouplerOffset{3.1};
  MultibodyPlant<double> plant_{kTimeStep};
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

// This test verifies that the linear dynamics matrix is properly computed
// according to A = ∂m/∂v = (M + dt⋅D).
TEST_F(KukaIiwaArmTests, CalcLinearDynamicsMatrix) {
  SetSingleRobotModel();
  const std::vector<MatrixXd> A =
      CompliantContactManagerTest::CalcLinearDynamicsMatrix(*manager_,
                                                            *context_);
  const int nv = plant_.num_velocities();
  MatrixXd Adense = MatrixXd::Zero(nv, nv);
  const MultibodyTreeTopology& topology =
      CompliantContactManagerTest::topology(*manager_);
  for (TreeIndex t(0); t < topology.num_trees(); ++t) {
    const int tree_start = topology.tree_velocities_start(t);
    const int tree_nv = topology.num_tree_velocities(t);
    Adense.block(tree_start, tree_start, tree_nv, tree_nv) = A[t];
  }
  const MatrixXd Aexpected =
      CalcLinearDynamicsMatrixIncludingImplicitDampingContribution();
  EXPECT_TRUE(
      CompareMatrices(Adense, Aexpected, kEps, MatrixCompareType::relative));
}

// This test verifies that the computation of free motion velocities v*
// correctly include the effect of damping implicitly.
TEST_F(KukaIiwaArmTests, CalcFreeMotionVelocities) {
  SetSingleRobotModel();
  const VectorXd v_star = CompliantContactManagerTest::CalcFreeMotionVelocities(
      *manager_, *context_);

  MultibodyForces<double> forces(plant_);
  CompliantContactManagerTest::CalcNonContactForces(*manager_, *context_,
                                                    &forces);
  const VectorXd zero_vdot = VectorXd::Zero(plant_.num_velocities());
  const VectorXd k0 = -plant_.CalcInverseDynamics(*context_, zero_vdot, forces);

  // A = M + dt*D
  const MatrixXd A =
      CalcLinearDynamicsMatrixIncludingImplicitDampingContribution();
  const VectorXd a = A.ldlt().solve(k0);
  const VectorXd& v0 = plant_.GetVelocities(*context_);
  const VectorXd v_star_expected = v0 + plant_.time_step() * a;

  EXPECT_TRUE(CompareMatrices(v_star, v_star_expected, 5.0 * kEps,
                              MatrixCompareType::relative));
}

// This unit test simply verifies that the manager is loading acceleration
// kinematics with the proper results. The correctness of the computations we
// rely on in this test (computation of accelerations) are tested elsewhere.
TEST_F(KukaIiwaArmTests, CalcAccelerationKinematicsCache) {
  SetSingleRobotModel();
  const VectorXd& v0 = plant_.GetVelocities(*context_);
  ContactSolverResults<double> contact_results;
  manager_->CalcContactSolverResults(*context_, &contact_results);
  const VectorXd a_expected =
      (contact_results.v_next - v0) / plant_.time_step();
  std::vector<SpatialAcceleration<double>> A_WB_expected(plant_.num_bodies());
  plant_.CalcSpatialAccelerationsFromVdot(*context_, a_expected,
                                          &A_WB_expected);

  // Verify CompliantContactManager loads the acceleration kinematics with the
  // proper results.
  AccelerationKinematicsCache<double> ac(
      CompliantContactManagerTest::topology(*manager_));
  manager_->CalcAccelerationKinematicsCache(*context_, &ac);
  EXPECT_TRUE(CompareMatrices(ac.get_vdot(), a_expected));
  for (BodyIndex b(0); b < plant_.num_bodies(); ++b) {
    const auto& body = plant_.get_body(b);
    EXPECT_TRUE(ac.get_A_WB(body.node_index()).IsApprox(A_WB_expected[b]));
  }
}

TEST_F(KukaIiwaArmTests, LimitConstraints) {
  SetSingleRobotModel();
  // Arbitrary selection of how positions and velocities are initialized.
  std::vector<InitializePositionAt> limits_specification(
      kNumJoints, InitializePositionAt::WellWithinLimits);
  limits_specification[0] = InitializePositionAt::BelowLowerLimit;
  limits_specification[1] = InitializePositionAt::AboveUpperLimit;
  limits_specification[2] =
      InitializePositionAt::AboveLowerLimitThoughPredictionBelow;
  limits_specification[3] =
      InitializePositionAt::BelowUpperLimitThoughPredictionAbove;
  limits_specification[4] = InitializePositionAt::WellWithinLimits;
  limits_specification[5] = InitializePositionAt::BelowLowerLimit;
  limits_specification[6] = InitializePositionAt::AboveUpperLimit;
  limits_specification[7] = InitializePositionAt::WellWithinLimits;
  limits_specification[8] = InitializePositionAt::WellWithinLimits;

  // Three joints are WellWithinLimits.
  const int kNumJointsWithLimits = 6;
  const int kNumConstraintEquations = 6;
  SetArbitraryStateWithLimitsSpecification(plant_, limits_specification,
                                           context_.get());

  const std::vector<DiscreteContactPair<double>>& discrete_pairs =
      CompliantContactManagerTest::EvalDiscreteContactPairs(*manager_,
                                                            *context_);
  const int num_contacts = discrete_pairs.size();
  // We are assuming there is no contact. Assert this.
  ASSERT_EQ(num_contacts, 0);

  const ContactProblemCache<double>& problem_cache =
      CompliantContactManagerTest::EvalContactProblemCache(*manager_,
                                                           *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // This model has no contact. We expect the number of constraints and
  // equations be consistent with limits_specification defined above.
  // Recall the model has one additional constraint to model the coupler between
  // the gripper fingers.
  EXPECT_EQ(problem.num_constraints(), kNumJointsWithLimits + 1);
  EXPECT_EQ(problem.num_constraint_equations(), kNumConstraintEquations + 1);

  // In this model we clearly have single tree, the arm with its gripper.
  const int tree_expected = 0;

  int num_constraints = 0;  // count number of constraints visited.
  // The manager adds limit constraints in the order joints are specified.
  // Therefore we verify the limit constrant for each joint.
  for (JointIndex joint_index(0); joint_index < plant_.num_joints();
       ++joint_index) {
    const Joint<double>& joint = plant_.get_joint(joint_index);
    if (joint.num_velocities() == 1) {
      const int v_index = joint.velocity_start();
      const InitializePositionAt limit_spec = limits_specification[v_index];

      if (limit_spec != InitializePositionAt::WellWithinLimits) {
        // Get limit constraint for the specific joint.
        const auto* constraint =
            dynamic_cast<const SapLimitConstraint<double>*>(
                &problem.get_constraint(num_constraints++));
        // Since the spec is not WellWithinLimits, we expect a constraint added.
        ASSERT_NE(constraint, nullptr);

        // Limit constraints always apply to a single tree in the multibody
        // forest.
        EXPECT_EQ(constraint->num_cliques(), 1);

        // There is a single tree in this model, the arm with gripper.
        EXPECT_EQ(constraint->first_clique(), tree_expected);

        EXPECT_EQ(constraint->clique_dof(), v_index);

        // Each constraints acts on the same tree (the arm+gripper) with a total
        // of kNumJoint DOFs in that tree.
        EXPECT_EQ(constraint->first_clique_jacobian().cols(), kNumJoints);

        // Verify the plant's state is consistent with the constraint's state.
        const double q0 = joint.GetOnePosition(*context_);
        EXPECT_EQ(constraint->position(), q0);

        // N.B. Default values implemented in
        // CompliantContactManager::AddLimitConstraints(), keep these values in
        // sync.
        const SapLimitConstraint<double>::Parameters& params =
            constraint->parameters();
        EXPECT_EQ(params.stiffness(), 1.0e12);
        EXPECT_EQ(params.dissipation_time_scale(), plant_.time_step());
        EXPECT_EQ(params.beta(), 0.1);

        const bool lower_limit_expected =
            limit_spec == InitializePositionAt::BelowLowerLimit ||
            limit_spec ==
                InitializePositionAt::AboveLowerLimitThoughPredictionBelow;
        const bool upper_limit_expected =
            limit_spec == InitializePositionAt::AboveUpperLimit ||
            limit_spec ==
                InitializePositionAt::BelowUpperLimitThoughPredictionAbove;

        const int expected_num_equations =
            lower_limit_expected && upper_limit_expected ? 2 : 1;
        EXPECT_EQ(constraint->num_constraint_equations(),
                  expected_num_equations);
        EXPECT_EQ(constraint->first_clique_jacobian().rows(),
                  expected_num_equations);

        const double kInf = std::numeric_limits<double>::infinity();
        const double ql_expected =
            lower_limit_expected ? joint.position_lower_limits()[0] : -kInf;
        const double qu_expected =
            upper_limit_expected ? joint.position_upper_limits()[0] : kInf;

        EXPECT_EQ(params.lower_limit(), ql_expected);
        EXPECT_EQ(params.upper_limit(), qu_expected);
      }
    }
  }
  EXPECT_EQ(num_constraints, kNumJointsWithLimits);
}

// This joint is used to verify the support of multi-DOF joints with/without
// joint limits. In particular, CompliantContactManager does not support limits
// for constraints with more than 1 DOF and therefore we expect the manager to
// throw an exception when building the problem.
// The implementation for this joint is incomplete. Only the strictly necessary
// overrides for the unit tests in this file are implemented.
template <typename T>
class MultiDofJointWithLimits final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultiDofJointWithLimits)

  // Arbitrary number of DOFs, though larger than one for these tests.
  static constexpr int kNumDofs = 3;

  // The constructor allows to specify finite joint limits.
  MultiDofJointWithLimits(const Frame<T>& frame_on_parent,
                          const Frame<T>& frame_on_child,
                          double pos_lower_limit, double pos_upper_limit)
      : Joint<T>("MultiDofJointWithLimits", frame_on_parent, frame_on_child,
                 VectorX<double>::Zero(kNumDofs),
                 VectorX<double>::Constant(kNumDofs, pos_lower_limit),
                 VectorX<double>::Constant(kNumDofs, pos_upper_limit),
                 VectorX<double>::Constant(
                     kNumDofs, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     kNumDofs, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     kNumDofs, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     kNumDofs, std::numeric_limits<double>::infinity())) {
    DRAKE_DEMAND(pos_lower_limit <= pos_upper_limit);
  }

  const std::string& type_name() const override {
    static const never_destroyed<std::string> name{"MultiDofJointWithLimits"};
    return name.access();
  }

 private:
  // Make MultiDofJointWithLimits templated on every other scalar type a friend
  // of MultiDofJointWithLimits<T> so that CloneToScalar<ToAnyOtherScalar>() can
  // access private members of MultiDofJointWithLimits<T>.
  template <typename>
  friend class MultiDofJointWithLimits;

  int do_get_num_velocities() const override { return kNumDofs; }
  int do_get_num_positions() const override { return kNumDofs; }
  // Dummy implementation, knowing our unit tests below have a single joint of
  // this type.
  int do_get_velocity_start() const override { return 0; }
  int do_get_position_start() const override { return 0; }

  std::unique_ptr<typename Joint<T>::BluePrint> MakeImplementationBlueprint()
      const override {
    auto blue_print = std::make_unique<typename Joint<T>::BluePrint>();
    // The only restriction here relevant for these tests is that we provide a
    // mobilizer with kNumDofs postions and velocities, so that indexes are
    // consistent during MultibodyPlant::Finalize().
    auto revolute_mobilizer = std::make_unique<internal::SpaceXYZMobilizer<T>>(
        this->frame_on_parent(), this->frame_on_child());
    blue_print->mobilizers_.push_back(std::move(revolute_mobilizer));
    return blue_print;
  }

  // We do not need an implementation for the methods below since the unit tests
  // do not exercise them. We mark them as "unreachable".

  void DoAddInOneForce(const Context<T>&, int, const T&,
                       MultibodyForces<T>*) const override {
    DRAKE_UNREACHABLE();
  }
  void DoAddInDamping(const Context<T>&, MultibodyForces<T>*) const override {
    DRAKE_UNREACHABLE();
  }
  std::string do_get_position_suffix(int) const override {
    DRAKE_UNREACHABLE();
  }
  std::string do_get_velocity_suffix(int) const override {
    DRAKE_UNREACHABLE();
  }
  void do_set_default_positions(const VectorX<double>&) override {
    DRAKE_UNREACHABLE();
  }
  const T& DoGetOnePosition(const Context<T>&) const override {
    DRAKE_UNREACHABLE();
  }
  const T& DoGetOneVelocity(const Context<T>&) const override {
    DRAKE_UNREACHABLE();
  }
  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override {
    DRAKE_UNREACHABLE();
  }
  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override {
    DRAKE_UNREACHABLE();
  }
  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override {
    DRAKE_UNREACHABLE();
  }
};

// Verify that CompliantContactManager throws when the model contains multi-DOF
// joints with finite limits.
GTEST_TEST(CompliantContactManager, ThrowForUnsupportedJoints) {
  MultibodyPlant<double> plant(1.0e-3);
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& body =
      plant.AddRigidBody("DummyBody", SpatialInertia<double>::MakeUnitary());
  plant.AddJoint(std::make_unique<MultiDofJointWithLimits<double>>(
      plant.world_frame(), body.body_frame(), -1.0, 2.0));
  plant.Finalize();
  auto owned_contact_manager =
      std::make_unique<CompliantContactManager<double>>();
  CompliantContactManager<double>* contact_manager =
      owned_contact_manager.get();
  plant.SetDiscreteUpdateManager(std::move(owned_contact_manager));
  auto context = plant.CreateDefaultContext();

  // Dummy v* and problem.
  const VectorXd v_star = Vector3d::Zero();
  SapContactProblem<double> problem(plant.time_step());

  // We verify the manager throws for the right reasons.
  DRAKE_EXPECT_THROWS_MESSAGE(CompliantContactManagerTest::AddLimitConstraints(
                                  *contact_manager, *context, v_star, &problem),
                              "Limits for joints with more than one degree of "
                              "freedom are not supported(.|\n)*");
}

// Verify that CompliantContactManager allows multi-DOF joints whenever these do
// not specify finite limits.
GTEST_TEST(CompliantContactManager,
           VerifyMultiDofJointsWithoutLimitsAreSupported) {
  MultibodyPlant<double> plant(1.0e-3);
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& body =
      plant.AddRigidBody("DummyBody", SpatialInertia<double>::MakeUnitary());
  const double kInf = std::numeric_limits<double>::infinity();
  plant.AddJoint(std::make_unique<MultiDofJointWithLimits<double>>(
      plant.world_frame(), body.body_frame(), -kInf, kInf));
  plant.Finalize();
  auto owned_contact_manager =
      std::make_unique<CompliantContactManager<double>>();
  CompliantContactManager<double>* contact_manager =
      owned_contact_manager.get();
  plant.SetDiscreteUpdateManager(std::move(owned_contact_manager));
  auto context = plant.CreateDefaultContext();

  const VectorXd v_star = Vector3d::Zero();
  SapContactProblem<double> problem(plant.time_step());
  EXPECT_NO_THROW(CompliantContactManagerTest::AddLimitConstraints(
      *contact_manager, *context, v_star, &problem));

  // No limit constraints are added since the only one joint in the model has
  // no limits.
  EXPECT_EQ(problem.num_constraints(), 0);
}

// This test verifies that the manager properly added holonomic constraints for
// the coupler constraints specified in the MultibodyPlant model.
TEST_F(KukaIiwaArmTests, CouplerConstraints) {
  // Only SAP supports the modeling of constraints.
  plant_.set_discrete_contact_solver(DiscreteContactSolver::kSap);

  // Load two robot models.
  std::vector<ModelInstanceIndex> arm_gripper1 = SetUpArmModel(1, &plant_);
  std::vector<ModelInstanceIndex> arm_gripper2 = SetUpArmModel(2, &plant_);

  // For testing purposes, we'll add a coupler constraint between joints in two
  // different arms.
  const Joint<double>& arm1_joint3 =
      plant_.GetJointByName("iiwa_joint_3", arm_gripper1[0]);
  const Joint<double>& arm2_joint6 =
      plant_.GetJointByName("iiwa_joint_6", arm_gripper2[0]);
  ConstraintIndex constraint_index = plant_.AddCouplerConstraint(
      arm1_joint3, arm2_joint6, kCouplerGearRatio, kCouplerOffset);
  EXPECT_EQ(constraint_index, ConstraintIndex(2));

  plant_.Finalize();

  // There should be three coupler constraints: one for each gripper and a third
  // one between the two arms.
  EXPECT_EQ(plant_.num_constraints(), 3);

  // Set manager using the experimental API so that we can bring it to scope.
  auto owned_contact_manager =
      std::make_unique<CompliantContactManager<double>>();
  manager_ = owned_contact_manager.get();
  plant_.SetDiscreteUpdateManager(std::move(owned_contact_manager));

  context_ = plant_.CreateDefaultContext();
  SetArbitraryNonZeroActuation(plant_, arm_gripper1[0], arm_gripper1[1],
                               context_.get());
  SetArbitraryNonZeroActuation(plant_, arm_gripper2[0], arm_gripper2[1],
                               context_.get());

  // Specify a state in which all kNumJoints are within limits so that we know
  // the contact problem has no limit constraints.
  std::vector<InitializePositionAt> limits_specification(
      2 * kNumJoints, InitializePositionAt::WellWithinLimits);
  SetArbitraryStateWithLimitsSpecification(plant_, limits_specification,
                                           context_.get());

  // We are assuming there is no contact. Assert this.
  const std::vector<DiscreteContactPair<double>>& discrete_pairs =
      CompliantContactManagerTest::EvalDiscreteContactPairs(*manager_,
                                                            *context_);
  const int num_contacts = discrete_pairs.size();
  ASSERT_EQ(num_contacts, 0);

  const ContactProblemCache<double>& problem_cache =
      CompliantContactManagerTest::EvalContactProblemCache(*manager_,
                                                           *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // This model has no contact and the configuration is set to be within joint
  // limits. Therefore we expect the problem to have the three couple
  // constraints we added to the model.
  EXPECT_EQ(problem.num_constraints(), 3);
  EXPECT_EQ(problem.num_constraint_equations(), 3);

  std::vector<std::pair<JointIndex, JointIndex>> coupler_joints;
  // Coupler on first robot's gripper.
  coupler_joints.push_back({
      plant_.GetJointByName("left_finger_sliding_joint", arm_gripper1[1])
          .index(),
      plant_.GetJointByName("right_finger_sliding_joint", arm_gripper1[1])
          .index(),
  });

  // Coupler on second robot's gripper.
  coupler_joints.push_back({
      plant_.GetJointByName("left_finger_sliding_joint", arm_gripper2[1])
          .index(),
      plant_.GetJointByName("right_finger_sliding_joint", arm_gripper2[1])
          .index(),
  });

  // Coupler between the two robots.
  coupler_joints.push_back({
      plant_.GetJointByName("iiwa_joint_3", arm_gripper1[0]).index(),
      plant_.GetJointByName("iiwa_joint_6", arm_gripper2[0]).index(),
  });

  // Verify each of the coupler constraints.
  for (int i = 0; i < 3; ++i) {
    const auto* constraint =
        dynamic_cast<const SapHolonomicConstraint<double>*>(
            &problem.get_constraint(i));

    // Verify it is a SapHolonomicConstraint as expected.
    ASSERT_NE(constraint, nullptr);

    // There are two cliques in this model, one for each robot arm.
    const int num_cliques = i == 2 ? 2 : 1;
    EXPECT_EQ(constraint->num_cliques(), num_cliques);
    const int first_clique = i == 1 ? 1 : 0;
    EXPECT_EQ(constraint->first_clique(), first_clique);
    if (i == 2) {
      // constraint between the two robots.
      EXPECT_EQ(constraint->second_clique(), 1);
    }

    const Joint<double>& joint0 = plant_.get_joint(coupler_joints[i].first);
    const Joint<double>& joint1 = plant_.get_joint(coupler_joints[i].second);

    // Verify the value of the constraint function.
    const double q0 = joint0.GetOnePosition(*context_);
    const double q1 = joint1.GetOnePosition(*context_);
    const Vector1d g0_expected(q0 - kCouplerGearRatio * q1 - kCouplerOffset);
    const VectorXd& g0 = constraint->constraint_function();
    EXPECT_EQ(g0, g0_expected);

    if (i < 2) {
      // For the grippers, fingers are the last two DOFs in their tree.
      const int left_index = 7;
      const int right_index = 8;
      const MatrixXd J_expected =
          (VectorXd::Unit(kNumJoints, left_index) -
           kCouplerGearRatio * VectorXd::Unit(kNumJoints, right_index))
              .transpose();
      EXPECT_EQ(constraint->first_clique_jacobian(), J_expected);
    } else {
      // The third constraint couples the two robot arms.
      const MatrixXd J0_expected =
          VectorXd::Unit(kNumJoints, 2 /* third joint. */).transpose();
      const MatrixXd J1_expected =
          -kCouplerGearRatio *
          VectorXd::Unit(kNumJoints, 5 /* sixth joint. */).transpose();
      EXPECT_EQ(constraint->first_clique_jacobian(), J0_expected);
      EXPECT_EQ(constraint->second_clique_jacobian(), J1_expected);
    }

    // N.B. Default values implemented in
    // CompliantContactManager::AddCouplerConstraints(), keep these values in
    // sync.
    const Vector1d kInfinity =
        Vector1d::Constant(std::numeric_limits<double>::infinity());
    const SapHolonomicConstraint<double>::Parameters& params =
        constraint->parameters();
    EXPECT_EQ(params.impulse_lower_limits(), -kInfinity);
    EXPECT_EQ(params.impulse_upper_limits(), kInfinity);
    EXPECT_EQ(params.stiffnesses(), kInfinity);
    EXPECT_EQ(params.relaxation_times(),
              Vector1d::Constant(plant_.time_step()));
    EXPECT_EQ(params.beta(), 0.1);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
