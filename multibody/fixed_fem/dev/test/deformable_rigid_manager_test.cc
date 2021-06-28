#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/rotation_matrix.h"
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
constexpr double kYoungsModulus = 1.23;
constexpr double kPoissonRatio = 0.456;
constexpr double kDensity = 0.789;
/* The mass damping coefficient is set to zero so that the free fall test
 (AdvanceOneTimeStep) produces an easy analytical solution. */
constexpr double kMassDamping = 0.0;
constexpr double kStiffnessDamping = 0.02;
/* Time step. */
constexpr double kDt = 0.0123;
constexpr double kGravity = -9.81;
/* Number of vertices in the box mesh (see below). */
constexpr int kNumVertices = 8;
/* Contact parameters. */
constexpr double kContactStiffness = 5e4;
constexpr double kContactDissipation = 5;
const CoulombFriction kFriction{0.3, 0.2};

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::GeometryId;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using geometry::SurfaceMesh;
using multibody::contact_solvers::internal::BlockSparseMatrix;
using systems::Context;

geometry::Box MakeUnitCube() { return geometry::Box(1.0, 1.0, 1.0); }

/* Makes a unit cube and subdivide it into 6 tetrahedra. */
geometry::VolumeMesh<double> MakeUnitCubeTetMesh() {
  geometry::VolumeMesh<double> mesh =
      geometry::internal::MakeBoxVolumeMesh<double>(MakeUnitCube(), 1.0);
  DRAKE_DEMAND(mesh.num_elements() == 6);
  DRAKE_DEMAND(mesh.num_vertices() == kNumVertices);
  return mesh;
}

/* Returns a proximity property with arbitrary elastic modulus, dissipation, and
 friction. */
ProximityProperties MakeProximityProperties() {
  ProximityProperties proximity_properties;
  geometry::AddContactMaterial({}, kContactDissipation, kContactStiffness,
                               kFriction, &proximity_properties);
  return proximity_properties;
}

/* Create a dummy DeformableBodyConfig. */
DeformableBodyConfig<double> MakeDeformableBodyConfig() {
  DeformableBodyConfig<double> config;
  config.set_youngs_modulus(kYoungsModulus);
  config.set_poisson_ratio(kPoissonRatio);
  config.set_mass_damping_coefficient(kMassDamping);
  config.set_stiffness_damping_coefficient(kStiffnessDamping);
  config.set_mass_density(kDensity);
  config.set_material_model(MaterialModel::kLinear);
  return config;
}

class DeformableRigidManagerTest : public ::testing::Test {
 protected:
  /* Builds a deformable model with a single deformable body and adds it to
   MultibodyPlant. Then sets a DeformableRigidManager as the discrete update
   manager for the MultibodyPlant. */
  void SetUp() override {
    auto deformable_model = std::make_unique<DeformableModel<double>>(&plant_);
    deformable_model->RegisterDeformableBody(MakeUnitCubeTetMesh(), "box",
                                             MakeDeformableBodyConfig(),
                                             MakeProximityProperties());
    deformable_model_ = deformable_model.get();
    plant_.AddPhysicalModel(std::move(deformable_model));
    /* Add a collision geometry. */
    plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
    plant_.RegisterCollisionGeometry(
        plant_.world_body(), math::RigidTransform<double>(), MakeUnitCube(),
        "collision", MakeProximityProperties());
    plant_.Finalize();
    auto deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>();
    deformable_rigid_manager_ = deformable_rigid_manager.get();
    plant_.SetDiscreteUpdateManager(std::move(deformable_rigid_manager));
    deformable_rigid_manager_->RegisterCollisionObjects(scene_graph_);
  }

  /* Verifies that there exists one and only one collision object registered
   with the DeformableRigidManager under test and return its geometry id. */
  void get_collision_geometry(GeometryId* geometry_id) const {
    const std::vector<std::vector<GeometryId>> collision_geometries =
        deformable_rigid_manager_->collision_geometries();
    ASSERT_EQ(collision_geometries.size(), 1);
    ASSERT_EQ(collision_geometries[0].size(), 1);
    *geometry_id = collision_geometries[0][0];
  }

  const std::vector<geometry::VolumeMesh<double>>& EvalDeformableMeshes(
      const systems::Context<double>& context) const {
    deformable_rigid_manager_->UpdateDeformableVertexPositions(context);
    return deformable_rigid_manager_->deformable_meshes_;
  }

  /* Returns the CollisionObjects owned by the DeformableRigidManager under
   test. */
  const internal::CollisionObjects<double> get_collision_objects() const {
    return deformable_rigid_manager_->collision_objects_;
  }

  /* Fowards the call to
   DeformableRigidManager<double>::CalcDeformableRigidContactPair(). */
  internal::DeformableRigidContactPair<double> CalcDeformableRigidContactPair(
      GeometryId rigid_id, SoftBodyIndex deformable_id) const {
    return deformable_rigid_manager_->CalcDeformableRigidContactPair(
        rigid_id, deformable_id);
  }

  /* Sets the collision object in `deformable_rigid_manager_` with `id` to the
   given `pose_in_world`. */
  void SetCollisionObjectPoseInWorld(GeometryId id,
                                     math::RigidTransformd pose_in_world) {
    deformable_rigid_manager_->collision_objects_.set_pose_in_world(
        id, pose_in_world);
  }

  SceneGraph<double> scene_graph_;
  MultibodyPlant<double> plant_{kDt};
  const DeformableModel<double>* deformable_model_;
  DeformableRigidManager<double>* deformable_rigid_manager_;
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
   In this test case vₙ and aₙ are both 0, so x - xₙ is given by 0.25 ⋅ a ⋅ dt².
  */
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
        deformable_rigid_manager.get();
    plant->SetDiscreteUpdateManager(std::move(deformable_rigid_manager));
    if (contact_solver != nullptr) {
      deformable_rigid_manager_ptr->SetContactSolver(std::move(contact_solver));
    }
  } else {
    if (contact_solver != nullptr) {
      plant->SetContactSolver(std::move(contact_solver));
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

/* Verifies that RegisterCollisionGeometry registers the rigid objects from
 MultibodyPlant in DeformableRigidManager as intended. */
TEST_F(DeformableRigidManagerTest, RegisterCollisionGeometry) {
  const internal::CollisionObjects<double>& collision_objects =
      get_collision_objects();
  GeometryId id;
  get_collision_geometry(&id);
  /* Verify the surface mesh is as expected. */
  const SurfaceMesh<double> expected_surface_mesh =
      geometry::internal::ConvertVolumeToSurfaceMesh(MakeUnitCubeTetMesh());
  EXPECT_TRUE(expected_surface_mesh.Equal(collision_objects.mesh(id)));
  /* Verify proximity property is as expected. */
  const CoulombFriction<double> mu = collision_objects.proximity_properties(id)
                                         .GetProperty<CoulombFriction<double>>(
                                             geometry::internal::kMaterialGroup,
                                             geometry::internal::kFriction);
  EXPECT_EQ(mu, kFriction);
}

// TODO(xuchenhan-tri): Add a unit test for UpdateCollisionObjectPoses() once
//  PR#15123 is merged.

/* Verifies that deformable vertex positions gets updated as expected. */
TEST_F(DeformableRigidManagerTest, UpdateDeformableVertexPositions) {
  auto context = plant_.CreateDefaultContext();
  auto simulator = systems::Simulator<double>(plant_, std::move(context));
  simulator.AdvanceTo(kDt);
  const std::vector<geometry::VolumeMesh<double>>&
      reference_configuration_meshes =
          deformable_model_->reference_configuration_meshes();
  DRAKE_DEMAND(reference_configuration_meshes.size() == 1);
  const std::vector<geometry::VolumeMesh<double>>& deformed_meshes =
      EvalDeformableMeshes(simulator.get_context());
  DRAKE_DEMAND(deformed_meshes.size() == 1);
  DRAKE_DEMAND(deformed_meshes[0].num_vertices() ==
               reference_configuration_meshes[0].num_vertices());
  DRAKE_DEMAND(deformed_meshes[0].num_elements() ==
               reference_configuration_meshes[0].num_elements());
  /* Verify that the elements of the deformed mesh is the same as the elements
   of the initial mesh. */
  for (geometry::VolumeElementIndex i(0); i < deformed_meshes[0].num_elements();
       ++i) {
    EXPECT_EQ(deformed_meshes[0].element(i),
              reference_configuration_meshes[0].element(i));
  }

  /* Verify that the vertices of the mesh is as expected. */
  const auto current_positions =
      deformable_model_->get_vertex_positions_output_port()
          .Eval<std::vector<VectorXd>>(simulator.get_context());
  EXPECT_EQ(current_positions.size(), 1);
  EXPECT_EQ(current_positions[0].size(), deformed_meshes[0].num_vertices() * 3);
  for (geometry::VolumeVertexIndex i(0); i < deformed_meshes[0].num_vertices();
       ++i) {
    const Vector3<double> p_WV = current_positions[0].segment<3>(3 * i);
    EXPECT_TRUE(CompareMatrices(p_WV, deformed_meshes[0].vertex(i).r_MV()));
  }
}

/* Verifies that the CalcDeformableRigidContactPair() method produces expected
 results. */
TEST_F(DeformableRigidManagerTest, CalcDeformableRigidContactPair) {
  const internal::CollisionObjects<double>& collision_objects =
      get_collision_objects();
  const std::vector<GeometryId> rigid_ids = collision_objects.geometry_ids();
  /* Verifies that there exists a unique rigid collision object. */
  EXPECT_EQ(rigid_ids.size(), 1);
  /* Shifts the rigid box to the -y direction so that the contact looks like
                                    +Z
                                     |
                                     |
               rigid box             |      deformable box
                     ----------+--+--+-------
                     |         |  ●  |      |
                     |         |  |  |      |
              -Y-----+---------+--+--+------+-------+Y
                     |         |  |  |      |
                     |         |  ●  |      |
                     ----------+--+--+-------
                                     |
                                     |
                                     |
                                    -Z
   where the "●"s denote representative contact points. */
  const auto X_DR = math::RigidTransformd(Vector3d(0, -0.75, 0));
  SetCollisionObjectPoseInWorld(rigid_ids[0], X_DR);
  /* Calculates the contact pair between the only rigid geometry and the only
   deformable geometry. */
  const SoftBodyIndex deformable_id(0);
  const internal::DeformableRigidContactPair<double> contact_pair =
      CalcDeformableRigidContactPair(rigid_ids[0], deformable_id);
  /* Verifies that the geometry ids are as expected. */
  EXPECT_EQ(contact_pair.rigid_id, rigid_ids[0]);
  EXPECT_EQ(contact_pair.deformable_id, deformable_id);
  /* Verifies that the contact parameters are as expected. */
  auto [expected_stiffness, expected_dissipation] =
      multibody::internal::CombinePointContactParameters(
          kContactStiffness, kContactStiffness, kContactDissipation,
          kContactDissipation);
  EXPECT_EQ(contact_pair.stiffness, expected_stiffness);
  EXPECT_EQ(contact_pair.dissipation, expected_dissipation);
  const CoulombFriction<double> expected_mu =
      CalcContactFrictionFromSurfaceProperties(kFriction, kFriction);
  EXPECT_EQ(contact_pair.friction, expected_mu.dynamic_friction());

  /* Verifies that the contact surface is as expected. */
  const DeformableContactSurface<double> expected_contact_surface =
      ComputeTetMeshTriMeshContact<double>(
          deformable_model_->reference_configuration_meshes()[0],
          collision_objects.mesh(rigid_ids[0]), X_DR);
  EXPECT_EQ(contact_pair.num_contact_points(),
            expected_contact_surface.num_polygons());
  const int num_contacts = expected_contact_surface.num_polygons();
  for (int i = 0; i < num_contacts; ++i) {
    const auto& expected_polygon_data =
        expected_contact_surface.polygon_data(i);
    const auto& calculated_polygon_data =
        contact_pair.contact_surface.polygon_data(i);
    EXPECT_EQ(expected_polygon_data.area, calculated_polygon_data.area);
    EXPECT_TRUE(CompareMatrices(expected_polygon_data.unit_normal,
                                calculated_polygon_data.unit_normal));
    EXPECT_TRUE(CompareMatrices(expected_polygon_data.centroid,
                                calculated_polygon_data.centroid));
    EXPECT_TRUE(CompareMatrices(expected_polygon_data.b_centroid,
                                calculated_polygon_data.b_centroid));
    EXPECT_EQ(expected_polygon_data.tet_index,
              calculated_polygon_data.tet_index);
  }

  /* Verifies the calculated rotation matrices map contact normals from world
   frame to contact frame ({0,0,1}). */
  constexpr double kTol = std::numeric_limits<double>::epsilon();
  EXPECT_EQ(contact_pair.R_CWs.size(), num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    EXPECT_TRUE(CompareMatrices(
        contact_pair.R_CWs[i] *
            contact_pair.contact_surface.polygon_data(i).unit_normal,
        Vector3d(0, 0, 1), kTol));
  }
}
}  // namespace

class DeformableRigidContactJacobianTest : public ::testing::Test {
 protected:
  /* Set up a scene with three rigid boxes and one deforamble cube.
   In particular, unit cube A is deformable and boxes B, C, and D are rigid and
   have size (2, 0.5, 2). */
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    auto deformable_model = std::make_unique<DeformableModel<double>>(plant_);
    A_ = deformable_model->RegisterDeformableBody(
        MakeUnitCubeTetMesh(), "box_A", MakeDeformableBodyConfig(),
        MakeProximityProperties());
    plant_->AddPhysicalModel(std::move(deformable_model));
    /* Add the rigid bodies. */
    const UnitInertia<double> G_Rcm =
        UnitInertia<double>::SolidBox(2.0, 0.5, 2.0);
    const SpatialInertia<double> M_Rcm(1.0, Vector3<double>::Zero(), G_Rcm);
    B_ = plant_->AddRigidBody("box_B", M_Rcm).index();
    C_ = plant_->AddRigidBody("box_C", M_Rcm).index();
    D_ = plant_->AddRigidBody("box_D", M_Rcm).index();
    collision_geometry_B_ = plant_->RegisterCollisionGeometry(
        plant_->get_body(B_), math::RigidTransform<double>(),
        geometry::Box(2, 0.5, 2), "B", MakeProximityProperties());
    collision_geometry_C_ = plant_->RegisterCollisionGeometry(
        plant_->get_body(C_), math::RigidTransform<double>(),
        geometry::Box(2, 0.5, 2), "C", MakeProximityProperties());
    // The geometry and the inertia is seemingly incompatible for box D, but we
    // are not using the inertia information anyway.
    collision_geometry_D_ = plant_->RegisterCollisionGeometry(
        plant_->get_body(D_), math::RigidTransform<double>(),
        geometry::Box(2, 2, 0.5), "D", MakeProximityProperties());
    plant_->Finalize();

    auto deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>();
    deformable_rigid_manager_ = deformable_rigid_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(deformable_rigid_manager));
    deformable_rigid_manager_->RegisterCollisionObjects(*scene_graph_);

    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
  }

  /* Calls DeformableRigidManager::CalcContactJacobian() and returns the contact
   jacobian as a dense matrix. */
  MatrixXd CalcContactJacobian(const Context<double>& context) const {
    const BlockSparseMatrix<double> Jc =
        deformable_rigid_manager_->CalcContactJacobian(context);
    return Jc.MakeDenseMatrix();
  }

  /* Calculates the rigid-rigid contact jacobian. */
  MatrixXd CalcRigidRigidContactJacobian(const Context<double>& context) const {
    return deformable_rigid_manager_->EvalContactJacobians(context).Jc;
  }

  int CalcNumRigidRigidContactPoints(const Context<double>& context) const {
    const std::vector<multibody::internal::DiscreteContactPair<double>>&
        rigid_contact_pairs =
            deformable_rigid_manager_->EvalDiscreteContactPairs(context);
    return rigid_contact_pairs.size();
  }

  /* Returns the CollisionObjects owned by the DeformableRigidManager under
   test. */
  const internal::CollisionObjects<double>& get_collision_objects() const {
    return deformable_rigid_manager_->collision_objects_;
  }

  const std::vector<geometry::VolumeMesh<double>>& get_deformable_meshes()
      const {
    return deformable_rigid_manager_->deformable_meshes_;
  }

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  const DeformableRigidManager<double>* deformable_rigid_manager_{nullptr};
  SoftBodyIndex A_;
  BodyIndex B_;
  BodyIndex C_;
  BodyIndex D_;
  GeometryId collision_geometry_B_;
  GeometryId collision_geometry_C_;
  GeometryId collision_geometry_D_;
  std::unique_ptr<systems::Diagram<double>> diagram_{nullptr};
  std::unique_ptr<Context<double>> diagram_context_{nullptr};
};

namespace {

/* Set up the scene so that the bodies A, B, C, and D look like:

                            +Z
                  B          |          C
                    -------  |   -------
                    |     |  |   |     |
                    |  ---+--+---+---  |
                    |  |  ●  |   ●  |  |
                    |  |  |  |A  |  |  |
              -Y----+--+--+--+---+--+--+----+Y
                    |  |  |  |   |  |  |
                    |  |  ●  |   ●  |  |
                    |  ---+--+---+---  |
                    |     |  |   |     |
               -----+-----+--+---+-----+-----
               |    -------  |   -------  D |
               --------------+---+-----+-----
                             |
                             |
                            -Z
 A is axis-aligned and centered at origin. B and C are axis-aligned with size
 (2, 0.5, 2). They are centered at (0, -0.5, 0) and (0, 0.5, 0) respectively. D
 is axis-aligned with size (2, 2, 0.5) and is centered at (0, 0, -1). There
 exist rigid-deformable contacts between A and B and between A and C. The "●"
 represents a characteristic rigid-deformable contact point. There exist
 rigid-rigid contacts between B and D and between C and D. */
TEST_F(DeformableRigidContactJacobianTest, ContactJacobian) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Set poses of the rigid bodies. */
  const Vector3d p_WB(0, -0.5, 0);
  const Vector3d p_WC(0, 0.5, 0);
  const Vector3d p_WD(0, 0, -1);
  const math::RigidTransformd X_WB(p_WB);
  const math::RigidTransformd X_WC(p_WC);
  const math::RigidTransformd X_WD(p_WD);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(B_), X_WB);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(C_), X_WC);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(D_), X_WD);

  /* Calculates the contact jacobian. */
  const MatrixXd Jc = CalcContactJacobian(plant_context);

  const std::vector<geometry::VolumeMesh<double>>& deformable_meshes =
      get_deformable_meshes();
  ASSERT_EQ(deformable_meshes.size(), 1);
  const auto& mesh_A = deformable_meshes[0];
  const internal::CollisionObjects<double>& collision_objects =
      get_collision_objects();
  /* The expected contact surfaces. */
  const DeformableContactSurface<double> contact_surface_AB =
      ComputeTetMeshTriMeshContact(
          mesh_A, collision_objects.mesh(collision_geometry_B_),
          collision_objects.pose_in_world(collision_geometry_B_));
  const DeformableContactSurface<double> contact_surface_AC =
      ComputeTetMeshTriMeshContact(
          mesh_A, collision_objects.mesh(collision_geometry_C_),
          collision_objects.pose_in_world(collision_geometry_C_));
  /* Number of contact points between A and B. */
  const int nc_AB = contact_surface_AB.num_polygons();
  /* Number of contact points between A and C. */
  const int nc_AC = contact_surface_AC.num_polygons();
  const int nc_rigid_deformable = nc_AB + nc_AC;
  /* Number of rigid-rigid contact points between B and D and between C and D.
   */
  const int nc_rigid_rigid = CalcNumRigidRigidContactPoints(plant_context);
  const int nc = nc_rigid_deformable + nc_rigid_rigid;
  /* The number of rows of the contact jacobian should be 3 times the number of
   contact points. */
  EXPECT_EQ(Jc.rows(), 3 * nc);

  const int nv_rigid = plant_->num_velocities();
  /* The contact participating deformable dofs should be a multiple of 3 and
   non-negative. */
  const int nv_deformable = Jc.cols() - nv_rigid;
  EXPECT_EQ(nv_deformable % 3, 0);
  EXPECT_GE(nv_deformable, 0);

  /* The contact jacobian should exhibit the following sparsity pattern:
                    rigid dofs                  participating deformable dofs
           ____________________________________________________________________
  rigid   |                               |                                    |
  rigid   |       Jc_rigid_rigid          |                  0                 |
  contact |                               |                                    |
          | ------------------------------------------------------------------ |
  rigid   |                               |                                    |
  deform  | Jc_rigid_deformable_wrt_rigid | Jc_rigid_deforamble_wrt_deformable |
  -able   | ______________________________|____________________________________|
  contact                                                                     */

  const auto& Jc_rigid_rigid = Jc.topLeftCorner(nc_rigid_rigid * 3, nv_rigid);
  /* The rigid-rigid block should be the same as the contact jacobian calculated
   by MultibodyPlant. */
  EXPECT_TRUE(CompareMatrices(Jc_rigid_rigid,
                              CalcRigidRigidContactJacobian(plant_context)));

  const auto& zero_block = Jc.topRightCorner(nc_rigid_rigid * 3, nv_deformable);
  EXPECT_TRUE(CompareMatrices(
      zero_block, MatrixXd::Zero(nc_rigid_rigid * 3, nv_deformable)));

  /* We verify the correctness of the deformable rigid blocks of the contact
   jacobian by calculating the contact velocities by multiplying the contact
   jacobian with arbitrary generalized velocities and verify the contact
   velocities against pen-and-paper calculation. */
  const auto& Jc_rigid_deformable = Jc.bottomRows(3 * nc_rigid_deformable);

  /* Set arbitrary spatial velocities for the rigid bodies. */
  const SpatialVelocity<double> V_WB(Vector3d(0.12, 0.23, 0.34),
                                     Vector3d(0.21, 0.32, 0.43));
  const SpatialVelocity<double> V_WC(Vector3d(0.1, 0.2, 0.3),
                                     Vector3d(0.4, 0.5, 0.6));
  const SpatialVelocity<double> V_WD(Vector3d(1.1, 1.2, 1.3),
                                     Vector3d(1.4, 1.5, 1.6));
  plant_->SetFreeBodySpatialVelocity(&plant_context, plant_->get_body(B_),
                                     V_WB);
  plant_->SetFreeBodySpatialVelocity(&plant_context, plant_->get_body(C_),
                                     V_WC);
  plant_->SetFreeBodySpatialVelocity(&plant_context, plant_->get_body(D_),
                                     V_WD);
  /* Set arbitrary (but uniform for simplicity) deformable velocities. */
  VectorXd deformable_velocities(nv_deformable);
  const Vector3d v_WA(1.11, 2.22, 3.33);
  for (int i = 0; i < nv_deformable / 3; ++i) {
    deformable_velocities.template segment<3>(3 * i) = v_WA;
  }
  /* Collect all velocities. */
  const int nv = nv_rigid + nv_deformable;
  VectorXd generalized_velocities(nv);
  generalized_velocities << plant_->GetVelocities(plant_context),
      deformable_velocities;

  const VectorXd contact_velocities =
      Jc_rigid_deformable * generalized_velocities;
  EXPECT_EQ(contact_velocities.size(), 3 * nc_rigid_deformable);

  /* The normal and tangential components of the contact velocities. */
  VectorXd vn(nc_rigid_deformable);
  VectorXd vt(2 * nc_rigid_deformable);
  using contact_solvers::internal::ExtractNormal;
  using contact_solvers::internal::ExtractTangent;
  ExtractNormal(contact_velocities, &vn);
  ExtractTangent(contact_velocities, &vt);

  /* Verifies the normal and tangential components of the contact velocities
   are as expected.
   @param v_WD  the velocity at the deformable contact point in world frame.
   @param V_WR  the spatial velocity at the rigid contact point in world frame.
   @param X_WR  pose of the rigid body in world frame.
   @param polygon_data  the contact polygon data.
   @param contact_index the index of the contact point among all deformable
                        rigid contacts. */
  auto verify_contact_velocity =
      [&vn, &vt](const Vector3d& v_WD, const SpatialVelocity<double>& V_WR,
                 const math::RigidTransformd& X_WR,
                 const ContactPolygonData<double>& polygon_data,
                 int contact_index) {
        /* The position of the contact point Rq on the rigid body R measured in
         and expressed in the world frame. */
        const Vector3d& p_WRq = polygon_data.centroid;
        /* The contact_point measured in R frame and expressed in world frame.
         */
        const Vector3d p_RoRq_W = p_WRq - X_WR.translation();
        /* Translation velocity of the contact point in the world frame. */
        const Vector3d v_WRq = V_WR.Shift(p_RoRq_W).translational();
        /* Translation velocity of the contact point in the deformable frame.
         */
        const Vector3d v_DRq = v_WD - v_WRq;
        /* Normal componenet of the contact velocity. */
        const double vn_DRq = v_DRq.dot(polygon_data.unit_normal);
        EXPECT_DOUBLE_EQ(vn(contact_index), vn_DRq);
        /* Tangent component of the contact velocity. */
        const Vector3d vt_DRq = v_DRq - vn_DRq * polygon_data.unit_normal;
        EXPECT_DOUBLE_EQ(vt.template segment<2>(2 * contact_index).norm(),
                         vt_DRq.norm());
      };
  for (int i = 0; i < nc_AB; ++i) {
    const int contact_index = i;
    const ContactPolygonData<double>& data = contact_surface_AB.polygon_data(i);
    verify_contact_velocity(v_WA, V_WB, X_WB, data, contact_index);
  }
  for (int i = 0; i < nc_AC; ++i) {
    const int contact_index = nc_AB + i;
    const ContactPolygonData<double>& data = contact_surface_AC.polygon_data(i);
    verify_contact_velocity(v_WA, V_WC, X_WC, data, contact_index);
  }
}

/* Verify that the calculated contact jacobian is the same as that from
 MultibodyPlant when no deformable body is in contact. */
TEST_F(DeformableRigidContactJacobianTest, RigidOnlyContactJacobian) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Set poses of the rigid bodies similar to the test above, but shift all
   rigid bodies in the -z direction so that the deformable body is not in
   contact. */
  const Vector3d p_WB(0, -0.5, 5);
  const Vector3d p_WC(0, 0.5, 5);
  const Vector3d p_WD(0, 0, 4);
  const math::RigidTransformd X_WB(p_WB);
  const math::RigidTransformd X_WC(p_WC);
  const math::RigidTransformd X_WD(p_WD);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(B_), X_WB);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(C_), X_WC);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(D_), X_WD);

  const MatrixXd Jc = CalcContactJacobian(plant_context);
  /* Two contact points, one between B and D, one between C and D. */
  const int nc = 2;
  EXPECT_EQ(Jc.rows(), 3 * nc);
  EXPECT_EQ(Jc.cols(), plant_->num_velocities());
  EXPECT_TRUE(
      CompareMatrices(Jc, CalcRigidRigidContactJacobian(plant_context)));
}

/* Verify that the contact jacobian is empty when there is no contact. */
TEST_F(DeformableRigidContactJacobianTest, EmptyContactJacobian) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Move the bodies far away from each other so that there is no contact at
   all. */
  const Vector3d p_WB(0, -5, 0);
  const Vector3d p_WC(0, 5, 0);
  const Vector3d p_WD(0, 0, -6);
  const math::RigidTransformd X_WB(p_WB);
  const math::RigidTransformd X_WC(p_WC);
  const math::RigidTransformd X_WD(p_WD);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(B_), X_WB);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(C_), X_WC);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(D_), X_WD);
  const MatrixXd Jc = CalcContactJacobian(plant_context);
  EXPECT_EQ(Jc.rows(), 0);
  EXPECT_EQ(Jc.cols(), 0);
}

}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
