#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/kinematics_vector.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::Box;
using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::SceneGraphInspector;
using drake::geometry::Sphere;
using drake::geometry::VolumeMesh;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::fem::FemState;
using drake::systems::Context;
using drake::systems::Simulator;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace multibody {
namespace internal {

/* Deformable body parameters.  */
constexpr double kYoungsModulus = 1e5;      // unit: N/m²
constexpr double kPoissonsRatio = 0.4;      // unitless.
constexpr double kMassDensity = 1e3;        // unit: kg/m³
constexpr double kStiffnessDamping = 0.01;  // unit: s
/* Time step (seconds). */
constexpr double kDt = 2.5e-3;
/* Contact parameters. */
const double kSlopeAngle = M_PI / 12.0;  // unit: radian
/* The friction coefficient has to be greater than or equal to tan(θ) to hold
 objects on a slope with an incline angle θ in stiction.
 Here we choose 0.4 > tan(π/12) ≈ 0.27. */
const CoulombFriction<double> kFriction{0.4, 0.4};

/* Sets up a deformable simulation with a deformable octahedron centered at the
 origin of the world frame and a rigid slope welded to the world body. The setup
 is used to test the discrete updates for deformable bodies in a simulator.
 Run:
   bazel run //tools:meldis -- --open-window
   bazel run //multibody/plant:deformable_integration_test
 to visualize the test. */
class DeformableIntegrationTest : public ::testing::Test {
 protected:
  /* Sets up a scene with a deformable cube sitting on the ground. */
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);

    DeformableModel<double>& deformable_model =
        plant_->mutable_deformable_model();
    body_id_ = RegisterDeformableOctahedron(&deformable_model, "deformable");
    model_ = &deformable_model;
    // N.B. Deformables are only supported with the SAP solver.
    // Thus for testing we choose one arbitrary contact approximation that uses
    // the SAP solver.
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    /* Register a rigid geometry that serves as an inclined plane. */
    ProximityProperties proximity_prop;
    geometry::AddContactMaterial({}, {}, kFriction, &proximity_prop);
    // TODO(xuchenhan-tri): Modify this when resolution hint is no longer used
    //  as the trigger for contact with deformable bodies.
    proximity_prop.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kRezHint, 1.0);
    const RigidTransformd X_WG(RollPitchYawd(kSlopeAngle, 0, 0),
                               Vector3d(0, 0, 0));
    const Box box(10, 10, 1);
    ground_collision_id_ = plant_->RegisterCollisionGeometry(
        plant_->world_body(), X_WG, box, "ground_collision", proximity_prop);
    IllustrationProperties illustration_props;
    illustration_props.AddProperty("phong", "diffuse",
                                   Vector4d(0.1, 0.8, 0.1, 0.8));
    plant_->RegisterVisualGeometry(plant_->world_body(), X_WG, box,
                                   "ground_visual", illustration_props);
    /* Set the initial position of the deformable body to be 0.7m above the
     ground. */
    plant_->mutable_deformable_model()
        .GetMutableBody(body_id_)
        .set_default_pose(RigidTransformd(Vector3d(0, 0, 0.7)));
    plant_->Finalize();

    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(contact_manager));
    driver_ = manager_->deformable_driver();
    DRAKE_DEMAND(driver_ != nullptr);
    /* Connect visualizer. Useful for when this test is used for debugging. */
    geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph_);

    diagram_ = builder.Build();
  }

  /* Calls DeformableDriver::EvalFemState(). */
  const FemState<double>& EvalFemState(const Context<double>& context,
                                       DeformableBodyIndex index) const {
    return driver_->EvalFemState(context, index);
  }

  /* Computes the reference volume of the registered deformable body. */
  double CalcDeformableReferenceVolume() const {
    const SceneGraphInspector<double>& inspector =
        scene_graph_->model_inspector();
    const GeometryId g_id = model_->GetGeometryId(body_id_);
    const VolumeMesh<double>* reference_mesh = inspector.GetReferenceMesh(g_id);
    DRAKE_DEMAND(reference_mesh != nullptr);
    return reference_mesh->CalcVolume();
  }

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  const DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  unique_ptr<systems::Diagram<double>> diagram_{nullptr};
  DeformableBodyId body_id_;
  GeometryId ground_collision_id_;

 private:
  DeformableBodyId RegisterDeformableOctahedron(DeformableModel<double>* model,
                                                std::string name) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(0.1), std::move(name));
    ProximityProperties props;
    geometry::AddContactMaterial({}, {}, kFriction, &props);
    geometry->set_proximity_properties(std::move(props));
    fem::DeformableBodyConfig<double> body_config;
    body_config.set_youngs_modulus(kYoungsModulus);
    body_config.set_poissons_ratio(kPoissonsRatio);
    body_config.set_mass_density(kMassDensity);
    body_config.set_stiffness_damping_coefficient(kStiffnessDamping);
    /* Make the resolution hint large enough so that we get an octahedron. */
    constexpr double kRezHint = 10.0;
    DeformableBodyId id = model->RegisterDeformableBody(std::move(geometry),
                                                        body_config, kRezHint);
    /* Verify that the geometry has 7 vertices and is indeed an octahedron. */
    const SceneGraphInspector<double>& inspector =
        scene_graph_->model_inspector();
    GeometryId g_id = model->GetGeometryId(id);
    const VolumeMesh<double>* mesh_G = inspector.GetReferenceMesh(g_id);
    DRAKE_DEMAND(mesh_G != nullptr);
    DRAKE_DEMAND(mesh_G->num_vertices() == 7);
    return id;
  }
};

namespace {

/* The deformable octahedron free-falls and lands on the rigid slope. Due to the
 large enough friction coefficient, the deformable body eventually reaches a
 static steady state. Verify the contact force applied by the ground to the
 deformable octahedron balances gravity in the steady state. */
TEST_F(DeformableIntegrationTest, SteadyState) {
  Simulator<double> simulator(*diagram_);
  /* Run simulation for long enough to reach steady state. */
  simulator.AdvanceTo(5.0);

  /* Verify the system has reached steady state. */
  const Context<double>& diagram_context = simulator.get_context();
  const Context<double>& plant_context =
      plant_->GetMyContextFromRoot(diagram_context);
  const FemState<double>& fem_state =
      EvalFemState(plant_context, DeformableBodyIndex(0));
  constexpr double kVelocityThreshold = 2e-5;      // unit: m/s.
  constexpr double kAccelerationThreshold = 2e-6;  // unit: m/s².
  const VectorXd& v = fem_state.GetVelocities();
  EXPECT_TRUE(CompareMatrices(v, VectorXd::Zero(v.size()), kVelocityThreshold));
  const VectorXd& a = fem_state.GetAccelerations();
  EXPECT_TRUE(
      CompareMatrices(a, VectorXd::Zero(a.size()), kAccelerationThreshold));

  /* Computes the total contact force on the body. */
  const ContactSolverResults<double>& contact_solver_results =
      manager_->EvalContactSolverResults(plant_context);
  /* The contact force at each contact point C expressed in the world frame. */
  const VectorXd& f_C_W = contact_solver_results.tau_contact;
  /* The contact force applied to the deformable body A expressed in the world
   frame. */
  Vector3d f_A_W = Vector3d::Zero();
  /* All contact forces are in the world frame, so we add them up to get the
   force on the body. */
  for (int i = 0; i < f_C_W.size(); i += 3) {
    f_A_W += f_C_W.segment<3>(i);
  }
  /* Computes the total gravitational force on the body. */
  const double volume = CalcDeformableReferenceVolume();
  const Vector3d expected_contact_force =
      volume * kMassDensity * (-plant_->gravity_field().gravity_vector());
  /* Verify the contact force balances gravity. */
  constexpr double kForceThreshold = 1e-5;  // unit: N.
  EXPECT_TRUE(CompareMatrices(expected_contact_force, f_A_W, kForceThreshold));

  /* Verifies the contact results agree with the contact solver results. */
  ContactResults<double> contact_results;
  manager_->CalcContactResults(plant_context, &contact_results);
  ASSERT_EQ(contact_results.num_deformable_contacts(), 1);
  const DeformableContactInfo<double>& contact_info =
      contact_results.deformable_contact_info(0);
  EXPECT_EQ(contact_info.id_A(), model_->GetGeometryId(body_id_));
  EXPECT_EQ(contact_info.id_B(), ground_collision_id_);

  const Vector3d p_WC = contact_info.contact_mesh().centroid();
  /* Accumulate the expected spatial force on the deformable body A at the
   centroid of the contact patch C by shifting the generalized force on each
   deformable vertex given by the contact solver. */
  SpatialForce<double> F_Ac_W_expected;
  F_Ac_W_expected.SetZero();
  GeometryId deformable_geometry_id = model_->GetGeometryId(body_id_);
  const VectorXd& vertex_positions =
      plant_->get_deformable_body_configuration_output_port()
          .template Eval<geometry::GeometryConfigurationVector<double>>(
              plant_context)
          .value(deformable_geometry_id);
  const int num_vertices = vertex_positions.size() / 3;
  /* The deformable mesh is an octahedron and the only stable configuration on a
   plane is with one face lying on the plane. In that configuration, all
   vertices except the center vertex are participating in contact and thus we
   expect the number of participating dofs to be equal to the total number of
   dofs in the deformable model minus 3. We know that the internal vertex is
   vertex zero. The the permutation is the mapping
   (1, 2, 3, 4, 5, 6) -> (0, 1, 2, 3, 4, 5), i.e., we can get the position of
   the i-th vertex *after* the permutation by looking at the i+1-th vertex
   position *before* the permutation.  */
  ASSERT_EQ(contact_solver_results.v_next.size(), 3 * (num_vertices - 1));
  for (int i = 0; i < num_vertices - 1; ++i) {
    const Vector3d& p_WV = vertex_positions.segment<3>(3 * (i + 1));
    const Vector3d p_VC_W = p_WC - p_WV;
    F_Ac_W_expected +=
        SpatialForce<double>(Vector3d::Zero(), f_C_W.segment<3>(3 * i))
            .Shift(p_VC_W);
  }
  const double kTol = 32.0 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(contact_info.F_Ac_W().translational(),
                              F_Ac_W_expected.translational(), kTol));
  EXPECT_TRUE(CompareMatrices(contact_info.F_Ac_W().rotational(),
                              F_Ac_W_expected.rotational(), kTol));
  /* Verify that adding deformable contact didn't mess up the generalized
   contact force port for rigid bodies. */
  EXPECT_NO_THROW(
      plant_->get_generalized_contact_forces_output_port(ModelInstanceIndex(0))
          .Eval(plant_context));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
