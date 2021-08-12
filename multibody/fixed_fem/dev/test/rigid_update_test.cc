#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

constexpr double kGravity = -9.81;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::SceneGraph;
using systems::Context;

/* Makes a finalized MultibodyPlant model of a ball falling into a plane using
 the given `contact_solver`. Sets a DiscreteUpdateManager for the plant if the
 `use_manager` flag is on. The given `scene_graph` is used to manage geometries
 and must be non-null. */
std::unique_ptr<MultibodyPlant<double>> MakePlant(
    std::unique_ptr<contact_solvers::internal::ContactSolver<double>>
        contact_solver,
    bool use_manager, SceneGraph<double>* scene_graph) {
  EXPECT_NE(scene_graph, nullptr);
  constexpr double kBouncingBallDt = 1e-3;  // s
  constexpr double kBallRadius = 0.05;      // m
  constexpr double kBallMass = 0.1;         // kg
  constexpr double kElasticModulus = 5e4;   // Pa
  constexpr double kDissipation = 5;        // s/m
  const CoulombFriction<double> kFriction(0.3, 0.3);
  std::unique_ptr<MultibodyPlant<double>> plant =
      examples::multibody::bouncing_ball::MakeBouncingBallPlant(
          kBouncingBallDt, kBallRadius, kBallMass, kElasticModulus,
          kDissipation, kFriction, kGravity * Vector3d::UnitZ(), true, false,
          scene_graph);
  if (use_manager) {
    auto deformable_model =
        std::make_unique<DeformableModel<double>>(plant.get());
    plant->AddPhysicalModel(std::move(deformable_model));
  }
  plant->Finalize();
  if (use_manager) {
    auto deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>(
            std::move(contact_solver));
    plant->SetDiscreteUpdateManager(std::move(deformable_rigid_manager));
  } else {
    plant->SetContactSolver(std::move(contact_solver));
  }
  return plant;
}

/* Sets up a discrete simulation with a rigid sphere in contact with a rigid
 ground and runs the simulation until `kFinalTime` and returns the final
 discrete states. The given `contact_solver` is used to solve the rigid
 contacts. If `use_manager` is true, use DiscreteUpdateManager to perform the
 discrete updates. Otherwise, use the MultibodyPlant discrete updates. */
VectorXd CalcFinalState(
    std::unique_ptr<contact_solvers::internal::ContactSolver<double>>
        contact_solver,
    bool use_manager) {
  DRAKE_DEMAND(contact_solver != nullptr);
  systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph<double>>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double>& plant = *builder.AddSystem(
      MakePlant(std::move(contact_solver), use_manager, &scene_graph));
  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  /* Wire up the plant and the scene graph. */
  DRAKE_DEMAND(plant.get_source_id().has_value());
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  auto diagram = builder.Build();

  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  /* Set non-trivial initial pose and velocity. */
  const math::RotationMatrixd R_WB(
      math::RollPitchYawd(Vector3<double>{100, 200, 300}));
  constexpr double kZ0 = 0.05;  // Initial ball height [m].
  const math::RigidTransformd X_WB(R_WB, Vector3d(0.0, 0.0, kZ0));
  plant.SetFreeBodyPose(&plant_context, plant.GetBodyByName("Ball"), X_WB);
  const SpatialVelocity<double> V_WB(Vector3d(100, 200, 300),
                                     Vector3d(1.5, 1.6, 1.7));
  plant.SetFreeBodySpatialVelocity(&plant_context, plant.GetBodyByName("Ball"),
                                   V_WB);

  /* Builds the simulator and simulate to final time. */
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  constexpr double kFinalTime = 0.1;
  simulator.AdvanceTo(kFinalTime);
  const Context<double>& final_plant_context =
      plant.GetMyContextFromRoot(simulator.get_context());
  return final_plant_context.get_discrete_state().get_vector().get_value();
}

/* Updates the states with MultibodyPlant and DeformableRigidManager
 respectively. Without any deformable dofs, the results should be equivalent. */
GTEST_TEST(RigidUpdateTest, PlantUpdateIsEquivalentToManagerUpdate) {
  const VectorXd final_state_with_manager = CalcFinalState(
      std::make_unique<contact_solvers::internal::PgsSolver<double>>(), true);
  const VectorXd final_state_without_manager = CalcFinalState(
      std::make_unique<contact_solvers::internal::PgsSolver<double>>(), false);
  EXPECT_TRUE(
      CompareMatrices(final_state_with_manager, final_state_without_manager));
  /* Sanity check that the final state is not NaN. */
  EXPECT_FALSE(final_state_with_manager.hasNaN());
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
