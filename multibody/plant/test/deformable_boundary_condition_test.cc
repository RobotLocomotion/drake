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
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
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
constexpr double kRadius = 0.1;             // unit: m
constexpr double kYoungsModulus = 2e3;      // unit: N/m²
constexpr double kPoissonsRatio = 0.4;      // unitless
constexpr double kMassDensity = 1e3;        // unit: kg/m³
constexpr double kStiffnessDamping = 0.01;  // unit: s
/* Time step (seconds). */
constexpr double kDt = 1e-2;

/* Sets up a deformable simulation with a deformable ball centered at the origin
 of the world frame. The top half of the ball is subject to wall boundary
 condition with the infinite wall being a top half space (z >= 0). The test
 verifies that the boundary condition is properly applied and it prevents the
 ball from free falling.
 Run:
   bazel run //tools:meldis -- --open-window
   bazel run //multibody/plant:deformable_boundary_condition_test
 to visualize the test. */
class DeformableIntegrationTest : public ::testing::Test {
 protected:
  /* Sets up a scene with a deformable cube sitting on the ground. */
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);

    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    body_id_ = RegisterDeformableBall(deformable_model.get(), "deformable");
    deformable_model->SetWallBoundaryCondition(
        body_id_, p_WQ_, n_W_);
    model_ = deformable_model.get();
    plant_->AddPhysicalModel(move(deformable_model));
    plant_->set_discrete_contact_solver(DiscreteContactSolver::kSap);

    /* Register a visual geometry for the "wall". */
    constexpr double box_height = 0.4;
    const Box box(0.5, 0.5, box_height);
    const RigidTransformd X_WG(Vector3d(0, 0, box_height / 2));
    IllustrationProperties illustration_props;
    illustration_props.AddProperty("phong", "diffuse",
                                   Vector4d(0.1, 0.8, 0.1, 0.8));
    plant_->RegisterVisualGeometry(plant_->world_body(), X_WG, box,
                                   "wall_visual", illustration_props);
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

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  DeformableModel<double>* model_{nullptr};
  const CompliantContactManager<double>* manager_{nullptr};
  const DeformableDriver<double>* driver_{nullptr};
  unique_ptr<systems::Diagram<double>> diagram_{nullptr};
  DeformableBodyId body_id_;
  /* A point on the surface of the wall. */
  const Vector3d p_WQ_{0.01, 0.02, -0.01};
  /* Outward normal of the wall. */
  const Vector3d n_W_{0.1, 0.2, -1};

 private:
  DeformableBodyId RegisterDeformableBall(DeformableModel<double>* model,
                                          std::string name) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(kRadius), move(name));
    ProximityProperties props;
    const CoulombFriction<double> kFriction{0.4, 0.4};
    geometry::AddContactMaterial({}, {}, kFriction, &props);
    geometry->set_proximity_properties(move(props));
    fem::DeformableBodyConfig<double> body_config;
    body_config.set_youngs_modulus(kYoungsModulus);
    body_config.set_poissons_ratio(kPoissonsRatio);
    body_config.set_mass_density(kMassDensity);
    body_config.set_stiffness_damping_coefficient(kStiffnessDamping);
    constexpr double kRezHint = kRadius;
    DeformableBodyId id =
        model->RegisterDeformableBody(move(geometry), body_config, kRezHint);
    return id;
  }
};

namespace {

/* The deformable ball would free-fall without boundary condition. We verify
 that the ball is not free falling by
  1. checking that it reaches a steady state, and
  2. verifying that it stays close to the initial configuration. */
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
  constexpr double kVelocityThreshold = 1e-5;      // unit: m/s.
  constexpr double kAccelerationThreshold = 1e-5;  // unit: m/s².
  const VectorXd& v = fem_state.GetVelocities();
  EXPECT_TRUE(CompareMatrices(v, VectorXd::Zero(v.size()), kVelocityThreshold));
  const VectorXd& a = fem_state.GetAccelerations();
  EXPECT_TRUE(
      CompareMatrices(a, VectorXd::Zero(a.size()), kAccelerationThreshold));

  /* Verify that vertices under the boundary condition didn't move and those not
   under the boundary condition stay close to the wall due to internal forces.
  */
  const VectorXd& q = fem_state.GetPositions();
  const VectorXd& reference_positions = model_->GetReferencePositions(body_id_);
  const int num_vertices = q.size() / 3;
  for (int i = 0; i < num_vertices; ++i) {
    const Vector3d reference_position = reference_positions.segment<3>(3 * i);
    const Vector3d current_position = q.segment<3>(3 * i);
    if (n_W_.dot(reference_position - p_WQ_) <= 0.0) {
      /* The vertex should stay unmoved if it's inside the wall. */
      EXPECT_TRUE(CompareMatrices(current_position, reference_position));
    } else {
      /* Otherwise, it would fall to be lower than the reference position in the
       z-direction under gravity but still stay close to the reference position
       due to internal forces. */
      EXPECT_LT(current_position.z(), reference_position.z());
      EXPECT_LT((current_position - reference_position).norm(), 0.5 * kRadius);
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
