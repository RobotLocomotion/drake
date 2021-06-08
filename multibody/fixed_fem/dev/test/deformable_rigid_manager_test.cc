#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/* Deformable body parameters. These parameters (with the exception of
 kMassDamping) are dummy in the sense that they do not affect the result of
 the test as long as they are valid. */
const double kYoungsModulus = 1.23;
const double kPoissonRatio = 0.456;
const double kDensity = 0.789;
/* The mass damping coefficient is set to zero so that the free fall test
 (AdvanceOneTimeStep) produces an easy analytical solution. */
const double kMassDamping = 0.0;
const double kStiffnessDamping = 0.02;
/* Time step. */
const double kDt = 0.0123;
const double kGravity = -9.81;
/* Number of vertices in the box mesh (see below). */
constexpr int kNumVertices = 8;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::SceneGraph;
using systems::Context;

class DeformableRigidManagerTest : public ::testing::Test {
 protected:
  /* Builds a deformable model with a single deformable body and adds it to
   MultibodyPlant. Then sets a DeformableRigidManager as the discrete update
   manager for the MultibodyPlant. */
  void SetUp() override {
    auto deformable_model = std::make_unique<DeformableModel<double>>(&plant_);
    deformable_model->RegisterDeformableBody(MakeBoxTetMesh(), "box",
                                             MakeDeformableBodyConfig());
    deformable_model_ = &plant_.AddPhysicalModel(std::move(deformable_model));
    plant_.Finalize();
    auto deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>();
    deformable_rigid_manager_ = &plant_.set_discrete_update_manager(
        std::move(deformable_rigid_manager));
  }

  /* Make a box and subdivide it into 6 tetrahedra. */
  static geometry::VolumeMesh<double> MakeBoxTetMesh() {
    const double length = 1;
    geometry::Box box(length, length, length);
    geometry::VolumeMesh<double> mesh =
        geometry::internal::MakeBoxVolumeMesh<double>(box, length);
    DRAKE_DEMAND(mesh.num_elements() == 6);
    DRAKE_DEMAND(mesh.num_vertices() == kNumVertices);
    return mesh;
  }

  /* Create a dummy DeformableBodyConfig. */
  static DeformableBodyConfig<double> MakeDeformableBodyConfig() {
    DeformableBodyConfig<double> config;
    config.set_youngs_modulus(kYoungsModulus);
    config.set_poisson_ratio(kPoissonRatio);
    config.set_mass_damping_coefficient(kMassDamping);
    config.set_stiffness_damping_coefficient(kStiffnessDamping);
    config.set_mass_density(kDensity);
    config.set_material_model(MaterialModel::kLinear);
    return config;
  }

  MultibodyPlant<double> plant_{kDt};
  const DeformableModel<double>* deformable_model_;
  const DeformableRigidManager<double>* deformable_rigid_manager_;
};

namespace {
/* Verifies that the DeformableRigidManager calculates the expected displacement
 for a deformable object under free fall over one time step. */
TEST_F(DeformableRigidManagerTest, CalcDiscreteValue) {
  auto context = plant_.CreateDefaultContext();
  auto simulator = systems::Simulator<double>(plant_, std::move(context));
  const auto initial_positions =
      deformable_model_->get_vertex_positions_output_port()
          .Eval<std::vector<VectorXd>>(simulator.get_context());
  EXPECT_EQ(initial_positions.size(), 1);
  EXPECT_EQ(initial_positions[0].size(), kNumVertices * 3);
  simulator.AdvanceTo(kDt);
  const auto current_positions =
      deformable_model_->get_vertex_positions_output_port()
          .Eval<std::vector<VectorXd>>(simulator.get_context());
  EXPECT_EQ(current_positions.size(), 1);
  EXPECT_EQ(current_positions[0].size(), kNumVertices * 3);

  /* The factor of 0.25 seems strange but is correct. For the default mid-point
   rule used by DynamicElasticityModel,
        x = xₙ + dt ⋅ vₙ + dt² ⋅ (0.25 ⋅ a + 0.25 ⋅ aₙ).
   In this test case vₙ and aₙ are both 0, so x - xₙ is given by
   0.25 ⋅ a ⋅ dt². */
  const Vector3<double> expected_displacement(0, 0,
                                              0.25 * kGravity * kDt * kDt);
  const double kTol = 1e-14;
  for (int i = 0; i < kNumVertices; ++i) {
    const Vector3<double> displacement =
        current_positions[0].segment<3>(3 * i) -
        initial_positions[0].segment<3>(3 * i);
    EXPECT_TRUE(CompareMatrices(displacement, expected_displacement, kTol));
  }
}

// TODO(xuchenhan-tri): Update the unit test once the
//  CalcAccelerationKinematicsCache() method is implemented.
TEST_F(DeformableRigidManagerTest, CalcAccelerationKinematicsCache) {
  auto context = plant_.CreateDefaultContext();
  EXPECT_THROW(plant_.get_generalized_acceleration_output_port().Eval(*context),
               std::exception);
}

/* Makes a finalized MultibodyPlant model of a ball falling into a plane. Sets a
 DiscreteUpdateManager for the plant if the `use_manager` flag is on. The given
 `scene_graph` is used to manage geometries and must be non-null. */
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
        std::make_unique<DeformableRigidManager<double>>();
    DeformableRigidManager<double>* deformable_rigid_manager_ptr =
        &plant->set_discrete_update_manager(
            std::move(deformable_rigid_manager));
    if (contact_solver != nullptr) {
      deformable_rigid_manager_ptr->set_contact_solver(
          std::move(contact_solver));
    }
  } else {
    if (contact_solver != nullptr) {
      plant->set_contact_solver(std::move(contact_solver));
    }
  }
  return plant;
}

/* Sets up a discrete simulation with a rigid sphere in contact with a rigid
 ground and runs the simulation until `kFinalTime` and returns the final
 discrete states. The given `contact_solver` is used to solve the rigid
 contacts. If `contact_solver == nullptr`, then the TAMSI solver is used. If
 `use_manager` is true, use DiscreteUpdateManager to perform the discrete
 updates. Otherwise, use the MultibodyPlant discrete updates. */
VectorXd CalcFinalState(
    std::unique_ptr<contact_solvers::internal::ContactSolver<double>>
        contact_solver,
    bool use_manager) {
  systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph<double>>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double>& plant = *builder.AddSystem(
      MakePlant(std::move(contact_solver), use_manager, &scene_graph));
  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  /* Wire up the plant and the scene graph. */
  DRAKE_DEMAND(!!plant.get_source_id());
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

/* Given a scene with no deformable body, verify that the simulation results
 obtained through the DeformableRigidManager is the same as the one obtained
 without when no contact solver is assigned. */
GTEST_TEST(RigidUpdateTest, TamsiSolver) {
  const VectorXd final_state_with_manager = CalcFinalState(nullptr, true);
  const VectorXd final_state_without_manager = CalcFinalState(nullptr, false);
  EXPECT_TRUE(
      CompareMatrices(final_state_with_manager, final_state_without_manager));
  /* Sanity check that the final state is not NaN. */
  EXPECT_FALSE(final_state_with_manager.hasNaN());
}

/* Similar to the test above but uses a contact solver instead of the TAMSI
 solver to solve rigid contacts. */
GTEST_TEST(RigidUpdateTest, ContactSolver) {
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
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
