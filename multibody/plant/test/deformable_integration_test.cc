#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/drake_visualizer.h"
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
using std::move;
using std::unique_ptr;

namespace drake {
namespace multibody {
namespace internal {

/* Provides access to a selection of private functions in
 CompliantContactManager for testing purposes. */
class CompliantContactManagerTester {
 public:
  static const DeformableDriver<double>* deformable_driver(
      const CompliantContactManager<double>& manager) {
    return manager.deformable_driver_.get();
  }
};

/* Deformable body parameters.  */
constexpr double kYoungsModulus = 1e5;       // unit: N/m²
constexpr double kPoissonsRatio = 0.4;       // unitless.
constexpr double kMassDensity = 1e3;         // unit: kg/m³
constexpr double kStiffnessDamping = 0.01;   // unit: s
/* Time step (seconds). */
constexpr double kDt = 1e-2;
/* Contact parameters. */
const double kSlopeAngle = M_PI / 12.0;             // unit: radian
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

    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    body_id_ =
        RegisterDeformableOctahedron(deformable_model.get(), "deformable");
    model_ = deformable_model.get();
    plant_->AddPhysicalModel(move(deformable_model));
    plant_->set_discrete_contact_solver(DiscreteContactSolver::kSap);

    /* Register a rigid geometry that serves as an inclined plane. */
    ProximityProperties proximity_prop;
    geometry::AddContactMaterial({}, {}, kFriction, &proximity_prop);
    // TODO(xuchenhan-tri): Modify this when resolution hint is no longer used
    //  as the trigger for contact with deformable bodies.
    proximity_prop.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kRezHint, 1.0);
    const RigidTransformd X_WG(RollPitchYawd(kSlopeAngle, 0, 0),
                               Vector3d(0, 0, -0.7));
    const Box box(10, 10, 1);
    plant_->RegisterCollisionGeometry(plant_->world_body(), X_WG, box,
                                      "ground_collision", proximity_prop);
    IllustrationProperties illustration_props;
    illustration_props.AddProperty("phong", "diffuse",
                                   Vector4d(0.1, 0.8, 0.1, 0.8));
    plant_->RegisterVisualGeometry(plant_->world_body(), X_WG, box,
                                   "ground_visual", illustration_props);
    plant_->Finalize();

    auto contact_manager = make_unique<CompliantContactManager<double>>();
    manager_ = contact_manager.get();
    plant_->SetDiscreteUpdateManager(move(contact_manager));
    driver_ = CompliantContactManagerTester::deformable_driver(*manager_);
    /* Connect visualizer. Useful for when this test is used for debugging. */
    geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph_);

    builder.Connect(model_->vertex_positions_port(),
                    scene_graph_->get_source_configuration_port(
                        plant_->get_source_id().value()));

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
  DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  unique_ptr<systems::Diagram<double>> diagram_{nullptr};
  DeformableBodyId body_id_;

 private:
  DeformableBodyId RegisterDeformableOctahedron(DeformableModel<double>* model,
                                                std::string name) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(0.1), move(name));
    ProximityProperties props;
    geometry::AddContactMaterial({}, {}, kFriction, &props);
    geometry->set_proximity_properties(move(props));
    fem::DeformableBodyConfig<double> body_config;
    body_config.set_youngs_modulus(kYoungsModulus);
    body_config.set_poissons_ratio(kPoissonsRatio);
    body_config.set_mass_density(kMassDensity);
    body_config.set_stiffness_damping_coefficient(kStiffnessDamping);
    /* Make the resolution hint large enough so that we get an octahedron. */
    constexpr double kRezHint = 10.0;
    DeformableBodyId id =
        model->RegisterDeformableBody(move(geometry), body_config, kRezHint);
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
  constexpr double kVelocityThreshold = 2e-5;  // unit: m/s.
  constexpr double kAccelerationThreshold = 1e-6;  // unit: m/s².
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
  /* The contact force applied to the deformable body B expressed in the world
   frame. */
  Vector3d f_B_W = Vector3d::Zero();
  /* All contact forces are in the world frame, so we add them up to get the
   force on the body. */
  for (int i = 0; i < f_C_W.size(); i += 3) {
    f_B_W += f_C_W.segment<3>(i);
  }
  /* Computes the total gravitational force on the body. */
  const double volume = CalcDeformableReferenceVolume();
  const Vector3d expected_contact_force =
      volume * kMassDensity * (-plant_->gravity_field().gravity_vector());
  /* Verify the contact force balances gravity. */
  constexpr double kForceThreshold = 1e-5;  // unit: N.
  EXPECT_TRUE(CompareMatrices(expected_contact_force, f_B_W, kForceThreshold));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
