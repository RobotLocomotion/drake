#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/fixed_fem/dev/matrix_utilities.h"
#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace fem {
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
constexpr double kDummySignedDistance = -0.058;
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
using geometry::TriangleSurfaceMesh;
using geometry::VolumeMesh;
using geometry::VolumeMeshFieldLinear;
using math::RigidTransformd;
using multibody::contact_solvers::internal::BlockSparseMatrix;
using multibody::contact_solvers::internal::PgsSolver;
using multibody::internal::DiscreteContactPair;
using systems::Context;

geometry::Box MakeUnitCube() { return geometry::Box(1.0, 1.0, 1.0); }

/* Makes a unit cube with the given `pose` in world and subdivide it into 6
 tetrahedra. */
VolumeMesh<double> MakeUnitCubeTetMesh(
    RigidTransformd pose = RigidTransformd()) {
  VolumeMesh<double> mesh =
      geometry::internal::MakeBoxVolumeMesh<double>(MakeUnitCube(), 1.0);
  DRAKE_DEMAND(mesh.num_elements() == 6);
  DRAKE_DEMAND(mesh.num_vertices() == kNumVertices);

  std::vector<geometry::VolumeElement> elements = mesh.tetrahedra();
  std::vector<Vector3d> vertices;
  for (const auto& v : mesh.vertices()) {
    vertices.emplace_back(pose * v);
  }
  return {std::move(elements), std::move(vertices)};
}

internal::ReferenceDeformableGeometry<double> MakeUnitCubeDeformableGeometry(
    RigidTransformd pose = RigidTransformd()) {
  auto mesh = std::make_unique<VolumeMesh<double>>(MakeUnitCubeTetMesh(pose));
  /* This doesn't satisfy the requirement of the approximated signed-distance
   field. In particular, it doesn't evaluate to zero on the boundary of the
   mesh. But we are using our clear-box knowledge here to test interpolation of
   the phi0 (and would like a nonzero value) and thus don't really care about
   whether it's a valid approximation. */
  std::vector<double> dummy_signed_distances(kNumVertices,
                                             kDummySignedDistance);
  auto mesh_field = std::make_unique<VolumeMeshFieldLinear<double, double>>(
      std::move(dummy_signed_distances), mesh.get(), false);
  return {std::move(mesh), std::move(mesh_field)};
}

/* Returns a proximity property with the given point contact stiffness,
 dissipation, and friction. */
ProximityProperties MakeProximityProperties(double stiffness,
                                            double dissipation,
                                            const CoulombFriction<double>& mu) {
  ProximityProperties proximity_properties;
  geometry::AddContactMaterial(dissipation, stiffness, mu,
                               &proximity_properties);
  return proximity_properties;
}

/* Returns a proximity property with an arbitrary set of default point contact
 stiffness, dissipation, and friction. */
ProximityProperties MakeDefaultProximityProperties() {
  return MakeProximityProperties(kContactStiffness, kContactDissipation,
                                 kFriction);
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
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    auto deformable_model = std::make_unique<DeformableModel<double>>(plant_);
    deformable_model->RegisterDeformableBody(MakeUnitCubeDeformableGeometry(),
                                             "box", MakeDeformableBodyConfig(),
                                             MakeDefaultProximityProperties());
    deformable_model_ = deformable_model.get();
    plant_->AddPhysicalModel(std::move(deformable_model));
    /* Add a collision geometry. Make sure it is *not* in contact with the
     deformable cube. */
    plant_->RegisterCollisionGeometry(
        plant_->world_body(), math::RigidTransform<double>(Vector3d(0, -2, 0)),
        MakeUnitCube(), "collision", MakeDefaultProximityProperties());
    plant_->Finalize();
    auto deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>(
            std::make_unique<PgsSolver<double>>());
    deformable_rigid_manager_ = deformable_rigid_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(deformable_rigid_manager));
    deformable_rigid_manager_->RegisterCollisionObjects(*scene_graph_);

    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
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

  const std::vector<geometry::internal::DeformableVolumeMesh<double>>&
  EvalDeformableMeshes(const systems::Context<double>& context) const {
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
      GeometryId rigid_id, DeformableBodyIndex deformable_id) const {
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

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  const DeformableModel<double>* deformable_model_;
  DeformableRigidManager<double>* deformable_rigid_manager_;
  std::unique_ptr<systems::Diagram<double>> diagram_{nullptr};
  std::unique_ptr<Context<double>> diagram_context_{nullptr};
};

namespace {
/* Verifies that the DeformableRigidManager calculates the expected
 displacement for a deformable object under free fall over one time step. */
TEST_F(DeformableRigidManagerTest, CalcDiscreteValue) {
  auto simulator =
      systems::Simulator<double>(*diagram_, std::move(diagram_context_));
  const auto& plant_context =
      plant_->GetMyContextFromRoot(simulator.get_context());
  const auto initial_positions =
      deformable_model_->get_vertex_positions_output_port()
          .Eval<std::vector<VectorXd>>(plant_context);
  EXPECT_EQ(initial_positions.size(), 1);
  EXPECT_EQ(initial_positions[0].size(), kNumVertices * 3);
  simulator.AdvanceTo(kDt);
  const auto current_positions =
      deformable_model_->get_vertex_positions_output_port()
          .Eval<std::vector<VectorXd>>(plant_context);
  EXPECT_EQ(current_positions.size(), 1);
  EXPECT_EQ(current_positions[0].size(), kNumVertices * 3);

  /* For the default Newmark scheme used by DynamicElasticityModel,
       x = xₙ + dt ⋅ vₙ + dt² ⋅ 0.5 ⋅ a.
   In this test case vₙ is 0, so x - xₙ is given by 0.5 ⋅ a ⋅ dt². */
  const Vector3<double> expected_displacement(0, 0, 0.5 * kGravity * kDt * kDt);
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
  const auto& plant_context = plant_->GetMyContextFromRoot(*diagram_context_);
  EXPECT_THROW(
      plant_->get_generalized_acceleration_output_port().Eval(plant_context),
      std::exception);
}

/* Verifies that RegisterCollisionGeometry registers the rigid objects from
 MultibodyPlant in DeformableRigidManager as intended. */
TEST_F(DeformableRigidManagerTest, RegisterCollisionGeometry) {
  const internal::CollisionObjects<double>& collision_objects =
      get_collision_objects();
  GeometryId id;
  get_collision_geometry(&id);
  /* Verify the surface mesh is as expected. */
  const TriangleSurfaceMesh<double> expected_surface_mesh =
      geometry::ConvertVolumeToSurfaceMesh(MakeUnitCubeTetMesh());
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
  auto simulator =
      systems::Simulator<double>(*diagram_, std::move(diagram_context_));
  const auto& plant_context =
      plant_->GetMyContextFromRoot(simulator.get_context());
  simulator.AdvanceTo(kDt);
  const std::vector<internal::ReferenceDeformableGeometry<double>>&
      reference_configuration_geometries =
          deformable_model_->reference_configuration_geometries();
  DRAKE_DEMAND(reference_configuration_geometries.size() == 1);
  const std::vector<geometry::internal::DeformableVolumeMesh<double>>&
      deformed_meshes = EvalDeformableMeshes(plant_context);
  DRAKE_DEMAND(deformed_meshes.size() == 1);
  DRAKE_DEMAND(deformed_meshes[0].mesh().num_vertices() ==
               reference_configuration_geometries[0].mesh().num_vertices());
  DRAKE_DEMAND(deformed_meshes[0].mesh().num_elements() ==
               reference_configuration_geometries[0].mesh().num_elements());
  /* Verify that the elements of the deformed mesh is the same as the elements
   of the initial mesh. */
  for (int i = 0; i < deformed_meshes[0].mesh().num_elements(); ++i) {
    EXPECT_EQ(deformed_meshes[0].mesh().element(i),
              reference_configuration_geometries[0].mesh().element(i));
  }

  /* Verify that the vertices of the mesh is as expected. */
  const auto current_positions =
      deformable_model_->get_vertex_positions_output_port()
          .Eval<std::vector<VectorXd>>(plant_context);
  EXPECT_EQ(current_positions.size(), 1);
  EXPECT_EQ(current_positions[0].size(),
            deformed_meshes[0].mesh().num_vertices() * 3);
  for (int i = 0; i < deformed_meshes[0].mesh().num_vertices(); ++i) {
    const Vector3<double> p_WV = current_positions[0].segment<3>(3 * i);
    EXPECT_TRUE(
        CompareMatrices(p_WV, deformed_meshes[0].mesh().vertex(i)));
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
  const DeformableBodyIndex deformable_id(0);
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
          geometry::internal::DeformableVolumeMesh<double>(
              deformable_model_->reference_configuration_geometries()[0]
                  .mesh()),
          collision_objects.mesh(rigid_ids[0]),
          collision_objects.bvh(rigid_ids[0]), X_DR);
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

constexpr double kStiffnessA = 1e4;
constexpr double kStiffnessB = 2e4;
constexpr double kStiffnessC = 3e4;
constexpr double kStiffnessD = 4e4;

constexpr double kDampingA = 1.4;
constexpr double kDampingB = 2.4;
constexpr double kDampingC = 3.4;
constexpr double kDampingD = 4.4;

const CoulombFriction kFrictionA = {0.14, 0.13};
const CoulombFriction kFrictionB = {0.24, 0.23};
const CoulombFriction kFrictionC = {0.34, 0.33};
const CoulombFriction kFrictionD = {0.44, 0.43};

/* Unit test for contact data from DeformableRigidManager fed to the contact
 solver. */
class DeformableRigidContactDataTest : public ::testing::Test {
 protected:
  /* Set up a scene with three rigid boxes and one deforamble cube.
   In particular, unit cube A is deformable and boxes B, C, and D are rigid and
   have size (2, 0.5, 2). */
  void SetUp() override { Initialize(false); }

  void Initialize(bool add_dirichlet_bc) {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    auto deformable_model = std::make_unique<DeformableModel<double>>(plant_);
    A_ = deformable_model->RegisterDeformableBody(
        MakeUnitCubeDeformableGeometry(), "box_A", MakeDeformableBodyConfig(),
        MakeProximityProperties(kStiffnessA, kDampingA, kFrictionA));
    if (add_dirichlet_bc) {
      /* Set up the boundary condition so that one and only one vertex (at
       position (-0.5, -0.5, -0.5)) is under dirichlet boundary conditions. */
      deformable_model->SetWallBoundaryCondition(A_, Vector3d(-0.5, -0.5, -0.5),
                                                 Vector3d(1, 1, 1));
      /* Verify our clear box knowledge that the vertex at position (-0.5,
       -0.5, -0.5) has vertex index 0 and thus dof index 0, 1, 2. */
      const FemModelBase<double>& fem_model = deformable_model->fem_model(A_);
      const DirichletBoundaryCondition<double>* dirichlet_bc =
          fem_model.dirichlet_boundary_condition();
      ASSERT_NE(dirichlet_bc, nullptr);
      const std::map<DofIndex, VectorXd>& bc_map = dirichlet_bc->get_bcs();
      ASSERT_EQ(bc_map.size(), 3);
      for (DofIndex dof_index(0); dof_index < 3; ++dof_index) {
        ASSERT_NE(bc_map.find(dof_index), bc_map.end());
      }
    }
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
        geometry::Box(2, 0.5, 2), "B",
        MakeProximityProperties(kStiffnessB, kDampingB, kFrictionB));
    collision_geometry_C_ = plant_->RegisterCollisionGeometry(
        plant_->get_body(C_), math::RigidTransform<double>(),
        geometry::Box(2, 0.5, 2), "C",
        MakeProximityProperties(kStiffnessC, kDampingC, kFrictionC));
    // The geometry and the inertia is seemingly incompatible for box D, but we
    // are not using the inertia information anyway.
    collision_geometry_D_ = plant_->RegisterCollisionGeometry(
        plant_->get_body(D_), math::RigidTransform<double>(),
        geometry::Box(2, 2, 0.5), "D",
        MakeProximityProperties(kStiffnessD, kDampingD, kFrictionD));
    plant_->Finalize();

    auto deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>(
            std::make_unique<PgsSolver<double>>());
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

  /* Forwards the call to DeformableRigidManager::CalcContactPointData() and
   returns the result as a return value. */
  internal::ContactPointData<double> CalcContactPointData(
      const Context<double>& context) const {
    internal::ContactPointData<double> data;
    deformable_rigid_manager_->CalcContactPointData(context, &data);
    return data;
  }

  /* Forwards the call to DeformableRigidManager::EvalDiscreteContactPairs(). */
  const std::vector<DiscreteContactPair<double>>& EvalDiscreteContactPairs(
      const Context<double>& context) const {
    return deformable_rigid_manager_->EvalDiscreteContactPairs(context);
  }

  int CalcNumRigidRigidContactPoints(const Context<double>& context) const {
    const std::vector<multibody::internal::DiscreteContactPair<double>>&
        rigid_contact_pairs = EvalDiscreteContactPairs(context);
    return rigid_contact_pairs.size();
  }

  /* Forwards the call to MultibodyPlant::CalcCombinedFrictionCoefficients()
   through DeformableRigidManager. */
  std::vector<CoulombFriction<double>> CalcCombinedFrictionCoefficients(
      const Context<double>& context,
      const std::vector<DiscreteContactPair<double>>& contact_pairs) const {
    return deformable_rigid_manager_->CalcCombinedFrictionCoefficients(
        context, contact_pairs);
  }

  /* Given the `context`, calculate the contact surface between deformable body
   A and the rigid geometry with the given GeometryId. */
  DeformableContactSurface<double> CalcDeformableRigidContactSurface(
      const Context<double>& context, GeometryId rigid_id) const {
    deformable_rigid_manager_->UpdateDeformableVertexPositions(context);
    deformable_rigid_manager_->UpdateCollisionObjectPoses(context);
    const std::vector<geometry::internal::DeformableVolumeMesh<double>>&
        deformable_meshes = deformable_rigid_manager_->deformable_meshes_;
    EXPECT_EQ(deformable_meshes.size(), 1);
    const auto& mesh_A = deformable_meshes[0];
    const internal::CollisionObjects<double>& collision_objects =
        deformable_rigid_manager_->collision_objects_;
    return ComputeTetMeshTriMeshContact(
        mesh_A, collision_objects.mesh(rigid_id),
        collision_objects.bvh(rigid_id),
        collision_objects.pose_in_world(rigid_id));
  }

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  const DeformableRigidManager<double>* deformable_rigid_manager_{nullptr};
  DeformableBodyIndex A_;
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
TEST_F(DeformableRigidContactDataTest, ContactJacobian) {
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

  const DeformableContactSurface<double> contact_surface_AB =
      CalcDeformableRigidContactSurface(plant_context, collision_geometry_B_);
  const DeformableContactSurface<double> contact_surface_AC =
      CalcDeformableRigidContactSurface(plant_context, collision_geometry_C_);
  /* Number of contact points between A and B and between A and C. */
  const int nc_AB = contact_surface_AB.num_polygons();
  const int nc_AC = contact_surface_AC.num_polygons();
  const int nc_rigid_deformable = nc_AB + nc_AC;
  /* Number of rigid-rigid contact points between B and D and between C and D.
   */
  const int nc_rigid_rigid = CalcNumRigidRigidContactPoints(plant_context);
  ASSERT_GT(nc_rigid_rigid, 0);
  ASSERT_GT(nc_rigid_deformable, 0);
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

/* Verify that the contact jacobian with Dirichlet boundary condition present is
 the same as that when there is no boundary condition except with appropriate
 columns zeroed out. */
TEST_F(DeformableRigidContactDataTest, ContactJacobianWithBoundaryCondition) {
  /* Calculate the contact jacobian *without* boundary condition. */
  Initialize(false);
  Context<double>& plant_without_bc_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Set poses of the rigid bodies. */
  const Vector3d p_WB(0, -0.5, 0);
  const Vector3d p_WC(0, 0.5, 0);
  const Vector3d p_WD(0, 0, -1);
  const math::RigidTransformd X_WB(p_WB);
  const math::RigidTransformd X_WC(p_WC);
  const math::RigidTransformd X_WD(p_WD);
  plant_->SetFreeBodyPose(&plant_without_bc_context, plant_->get_body(B_),
                          X_WB);
  plant_->SetFreeBodyPose(&plant_without_bc_context, plant_->get_body(C_),
                          X_WC);
  plant_->SetFreeBodyPose(&plant_without_bc_context, plant_->get_body(D_),
                          X_WD);
  MatrixXd Jc_without_bc = CalcContactJacobian(plant_without_bc_context);

  /* Calculate the contact jacobian *with* boundary condition. */
  Initialize(true);
  Context<double>& plant_with_bc_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  plant_->SetFreeBodyPose(&plant_with_bc_context, plant_->get_body(B_), X_WB);
  plant_->SetFreeBodyPose(&plant_with_bc_context, plant_->get_body(C_), X_WC);
  plant_->SetFreeBodyPose(&plant_with_bc_context, plant_->get_body(D_), X_WD);
  const MatrixXd Jc_with_bc = CalcContactJacobian(plant_with_bc_context);

  /* Jc_without_bc should be the same as Jc_with_bc except that the columns
   corresponding to the deformable dofs under bc (the 0th, 1st, and 2nd
   deformable dofs) are zeroed out. */
  EXPECT_FALSE(CompareMatrices(Jc_with_bc, Jc_without_bc));
  const int nv_rigid = plant_->num_velocities();
  for (int deformable_dof = 0; deformable_dof < 3; ++deformable_dof) {
    Jc_without_bc.col(nv_rigid + deformable_dof).setZero();
  }
  EXPECT_TRUE(CompareMatrices(Jc_with_bc, Jc_without_bc));
}

/* Uses the same set up as the ContactJacobian test and verifies that
 CalcContactPointData correctly assembles the data from both rigid-rigid
 contacts and deformable-rigid contacts. Note that in this test, we exploit our
 glass box knowledge that the computation is handled by some other, previously
 tested code, and we only test that the data is correctly assembled. */
TEST_F(DeformableRigidContactDataTest, ContactData) {
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

  const DeformableContactSurface<double> contact_surface_AB =
      CalcDeformableRigidContactSurface(plant_context, collision_geometry_B_);
  const DeformableContactSurface<double> contact_surface_AC =
      CalcDeformableRigidContactSurface(plant_context, collision_geometry_C_);
  /* Number of contact points between A and B and between A and C. */
  const int nc_AB = contact_surface_AB.num_polygons();
  const int nc_AC = contact_surface_AC.num_polygons();
  const int nc_rigid_deformable = nc_AB + nc_AC;
  /* Number of rigid-rigid contact points between B and D and between C and D.
   */
  const int nc_rigid_rigid = CalcNumRigidRigidContactPoints(plant_context);
  ASSERT_GT(nc_rigid_rigid, 0);
  ASSERT_GT(nc_rigid_deformable, 0);

  /* Verifies that the contact point data is as expected. */
  const internal::ContactPointData<double> contact_point_data =
      CalcContactPointData(plant_context);
  const VectorXd& mu = contact_point_data.mu;
  const VectorXd& phi0 = contact_point_data.phi0;
  const VectorXd& stiffness = contact_point_data.stiffness;
  const VectorXd& damping = contact_point_data.damping;

  /* Verifies that the friction data is as expected. */
  /* rigid vs. rigid friction. */
  const auto& mu_rigid_rigid = mu.head(nc_rigid_rigid);
  const std::vector<DiscreteContactPair<double>>& rigid_contact_pairs =
      EvalDiscreteContactPairs(plant_context);
  const std::vector<CoulombFriction<double>> rigid_frictions =
      CalcCombinedFrictionCoefficients(plant_context, rigid_contact_pairs);
  ASSERT_EQ(rigid_frictions.size(), nc_rigid_rigid);
  VectorXd mu_rigid_rigid_expected(nc_rigid_rigid);
  for (int i = 0; i < nc_rigid_rigid; ++i) {
    mu_rigid_rigid_expected(i) = rigid_frictions[i].dynamic_friction();
  }
  EXPECT_TRUE(CompareMatrices(mu_rigid_rigid, mu_rigid_rigid_expected));

  /* A vs. B friction. */
  const auto& mu_AB = mu.segment(nc_rigid_rigid, nc_AB);
  const VectorXd mu_AB_expected =
      CalcContactFrictionFromSurfaceProperties(kFrictionA, kFrictionB)
          .dynamic_friction() *
      VectorXd::Ones(nc_AB);
  EXPECT_TRUE(CompareMatrices(mu_AB, mu_AB_expected));

  /* A vs. C friction. */
  const auto& mu_AC = mu.tail(nc_AC);
  const VectorXd mu_AC_expected =
      CalcContactFrictionFromSurfaceProperties(kFrictionA, kFrictionC)
          .dynamic_friction() *
      VectorXd::Ones(nc_AC);
  EXPECT_TRUE(CompareMatrices(mu_AC, mu_AC_expected));

  /* Verifies that the phi0 data is as expected. */
  /* rigid vs. rigid phi0. */
  const auto& phi0_rigid_rigid = phi0.head(nc_rigid_rigid);
  ASSERT_EQ(rigid_contact_pairs.size(), nc_rigid_rigid);
  VectorXd phi0_rigid_rigid_expected(nc_rigid_rigid);
  for (int i = 0; i < nc_rigid_rigid; ++i) {
    phi0_rigid_rigid_expected(i) = rigid_contact_pairs[i].phi0;
  }
  EXPECT_TRUE(CompareMatrices(phi0_rigid_rigid, phi0_rigid_rigid_expected));

  /* rigid vs. deformable penetration distance. */
  /* The phi0 data is interpolated from a constant field with value
   kDummySignedDistance. */
  EXPECT_TRUE(CompareMatrices(
      phi0.tail(nc_rigid_deformable),
      kDummySignedDistance * VectorXd::Ones(nc_rigid_deformable),
      std::numeric_limits<double>::epsilon()));

  /* Verifies that the stiffness and damping data is as expected. */
  const auto [k_AB, d_AB] = multibody::internal::CombinePointContactParameters(
      kStiffnessA, kStiffnessB, kDampingA, kDampingB);
  const auto [k_AC, d_AC] = multibody::internal::CombinePointContactParameters(
      kStiffnessA, kStiffnessC, kDampingA, kDampingC);

  /* rigid vs. rigid stiffness and damping. */
  const auto& stiffness_rigid_rigid = stiffness.head(nc_rigid_rigid);
  VectorXd stiffness_rigid_rigid_expected(nc_rigid_rigid);
  for (int i = 0; i < nc_rigid_rigid; ++i) {
    stiffness_rigid_rigid_expected(i) = rigid_contact_pairs[i].stiffness;
  }
  EXPECT_TRUE(
      CompareMatrices(stiffness_rigid_rigid, stiffness_rigid_rigid_expected));
  const auto& damping_rigid_rigid = damping.head(nc_rigid_rigid);
  VectorXd damping_rigid_rigid_expected(nc_rigid_rigid);
  for (int i = 0; i < nc_rigid_rigid; ++i) {
    damping_rigid_rigid_expected(i) = rigid_contact_pairs[i].damping;
  }
  EXPECT_TRUE(
      CompareMatrices(damping_rigid_rigid, damping_rigid_rigid_expected));

  /* A vs. B stiffness and damping. */
  const auto& stiffness_AB = stiffness.segment(nc_rigid_rigid, nc_AB);
  const auto& damping_AB = damping.segment(nc_rigid_rigid, nc_AB);
  EXPECT_TRUE(CompareMatrices(stiffness_AB, k_AB * VectorXd::Ones(nc_AB)));
  EXPECT_TRUE(CompareMatrices(damping_AB, d_AB * VectorXd::Ones(nc_AB)));

  /* A vs. C stiffness and damping. */
  const auto& stiffness_AC = stiffness.tail(nc_AC);
  const auto& damping_AC = damping.tail(nc_AC);
  EXPECT_TRUE(CompareMatrices(stiffness_AC, k_AC * VectorXd::Ones(nc_AC)));
  EXPECT_TRUE(CompareMatrices(damping_AC, d_AC * VectorXd::Ones(nc_AC)));
}

/* Verify that the calculated contact jacobian is the same as that from
 MultibodyPlant when no deformable body is in contact. */
TEST_F(DeformableRigidContactDataTest, RigidContactOnly) {
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

/* Verify that the contact jacobian and the contact data are empty when there
 is no contact. */
TEST_F(DeformableRigidContactDataTest, NoContact) {
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

  /* Verify that the contact point data is empty. */
  const internal::ContactPointData<double> contact_point_data =
      CalcContactPointData(plant_context);
  EXPECT_EQ(contact_point_data.mu.size(), 0);
  EXPECT_EQ(contact_point_data.phi0.size(), 0);
  EXPECT_EQ(contact_point_data.stiffness.size(), 0);
  EXPECT_EQ(contact_point_data.damping.size(), 0);
}

}  // namespace

/* Unit test for dynamics data from DeformableRigidManager fed to the contact
 solver. */
class DeformableRigidDynamicsDataTest : public ::testing::Test {
 protected:
  /* Set up a scene with two deforamble bodies and one rigid cube.
   In particular, unit cube A and octahedron B are deformable and unit cube
   C is rigid. */
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);
    /* Add two deformable bodies. */
    auto deformable_model = std::make_unique<DeformableModel<double>>(plant_);
    A_ = deformable_model->RegisterDeformableBody(
        MakeUnitCubeDeformableGeometry(RigidTransformd(Vector3d(0, 2, 0))),
        "cube_A", MakeDeformableBodyConfig(),
        MakeProximityProperties(kStiffnessA, kDampingA, kFrictionA));
    B_ = deformable_model->RegisterDeformableBody(
        MakeOctahedronDeformableGeometry<double>(), "octahedron_B",
        MakeDeformableBodyConfig(),
        MakeProximityProperties(kStiffnessB, kDampingB, kFrictionB));
    plant_->AddPhysicalModel(std::move(deformable_model));
    /* Add the rigid cube. */
    C_ = plant_->AddRigidBody("cube_C", MakeSpatialInertiaForBox()).index();
    collision_geometry_C_ = plant_->RegisterCollisionGeometry(
        plant_->get_body(C_), math::RigidTransform<double>(),
        geometry::Box(1.0, 1.0, 1.0), "C",
        MakeProximityProperties(kStiffnessC, kDampingC, kFrictionC));
    plant_->Finalize();

    auto owned_deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>(
            std::make_unique<PgsSolver<double>>());
    deformable_rigid_manager_ = owned_deformable_rigid_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_deformable_rigid_manager));
    deformable_rigid_manager_->RegisterCollisionObjects(*scene_graph_);

    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
  }

  static SpatialInertia<double> MakeSpatialInertiaForBox() {
    const UnitInertia<double> G_Rcm =
        UnitInertia<double>::SolidBox(1.0, 1.0, 1.0);
    return SpatialInertia<double>(1.0, Vector3d::Zero(), G_Rcm);
  }

  /* Calls DeformableRigidManager::EvalContactTangentMatrix() and returns the
   tangent matrix for contact as a dense matrix. */
  MatrixXd CalcContactTangentMatrix(const Context<double>& context) const {
    const BlockSparseMatrix<double> A =
        deformable_rigid_manager_->EvalContactTangentMatrix(context);
    return A.MakeDenseMatrix();
  }

  /* Calls DeformableRigidManager::EvalFreeMotionTangentMatrix() and returns the
   free motion tangent matrix of the deformable body with the given `index` as a
   dense matrix. */
  MatrixXd CalcFreeMotionTangentMatrix(const Context<double>& context,
                                       DeformableBodyIndex index) const {
    const Eigen::SparseMatrix<double> tangent_matrix_sparse =
        deformable_rigid_manager_->EvalFreeMotionTangentMatrix(context, index);
    return MatrixXd(tangent_matrix_sparse);
  }

  /* Calls DeformableRigidManager::EvalFreeMotionTangentMatrixSchurComplement()
   and returns the Schur complement of the tangent matrix of the deformable body
   with the given `index` as a dense matrix. */
  const MatrixXd& EvalFreeMotionTangentMatrixSchurComplement(
      const systems::Context<double>& context,
      DeformableBodyIndex index) const {
    const internal::SchurComplement<double>& schur_complement =
        deformable_rigid_manager_->EvalFreeMotionTangentMatrixSchurComplement(
            context, index);
    return schur_complement.get_D_complement();
  }

  /* Calls DeformableRigidManager::EvalFreeMotionRigidVelocities(). */
  const VectorX<double>& EvalFreeMotionRigidVelocities(
      const systems::Context<double>& context) const {
    return deformable_rigid_manager_->EvalFreeMotionRigidVelocities(context);
  }

  /* Calls DeformableRigidManager::EvalFreeMotionParticipatingVelocities(). */
  const VectorX<double>& EvalFreeMotionParticipatingVelocities(
      const systems::Context<double>& context) const {
    return deformable_rigid_manager_->EvalFreeMotionParticipatingVelocities(
        context);
  }

  /* Calls DeformableRigidManager::EvalParticipatingVelocities(). */
  const VectorX<double>& EvalParticipatingVelocities(
      const systems::Context<double>& context) const {
    return deformable_rigid_manager_->EvalParticipatingVelocities(context);
  }

  /* Calls DeformableRigidManager::EvalFreeMotionFemStateBase(). */
  const FemStateBase<double>& EvalFreeMotionFemStateBase(
      const systems::Context<double>& context,
      DeformableBodyIndex index) const {
    return deformable_rigid_manager_->EvalFreeMotionFemStateBase(context,
                                                                 index);
  }

  /* Calls DeformableRigidManager::EvalDeformableRigidContact(). */
  const std::vector<internal::DeformableContactData<double>>&
  EvalDeformableRigidContact(const systems::Context<double>& context) {
    return deformable_rigid_manager_->EvalDeformableRigidContact(context);
  }

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  const DeformableRigidManager<double>* deformable_rigid_manager_{nullptr};
  DeformableBodyIndex A_;
  DeformableBodyIndex B_;
  BodyIndex C_;
  GeometryId collision_geometry_C_;
  std::unique_ptr<systems::Diagram<double>> diagram_{nullptr};
  std::unique_ptr<Context<double>> diagram_context_{nullptr};
};

namespace {

/* Verifies that the Schur complement of the deformable free motion tangent
 matrix is as expected in a simple scenario where we know what the contact
 looks like. */
TEST_F(DeformableRigidDynamicsDataTest,
       FreeMotionTangentMatrixSchurComplement) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Move the rigid cube up so that it intersects the octahedron in the upper
  prism. Recall that the octahedron looks like:
                +Z   -X
                 |   /
              v5 ●  ● v3
                 | /
       v4     v0 |/
  -Y----●--------●------●----+Y
                /|      v2
               / |
           v1 ●  ● v6
             /   |
           +X    |
                -Z
  Therefore, all vertices except v6 are participating in contact. */
  const Vector3d p_WC(0, 0, 1);
  const math::RigidTransformd X_WC(p_WC);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(C_), X_WC);
  const MatrixXd tangent_matrix_schur_complement =
      EvalFreeMotionTangentMatrixSchurComplement(plant_context, B_);
  constexpr int num_participating_vertices = 6;
  EXPECT_EQ(tangent_matrix_schur_complement.rows(),
            3 * num_participating_vertices);
  EXPECT_EQ(tangent_matrix_schur_complement.cols(),
            3 * num_participating_vertices);
  /* Construct the Schur complement by hand and verify that it matches the
   calculated result. Note that here we rely on the fact that the permutation in
   the deformable dofs is identity since it so happens that v6 is the
   non-participating vertex. */
  const MatrixXd tangent_matrix =
      CalcFreeMotionTangentMatrix(plant_context, B_);
  const Eigen::SparseMatrix<double> participating_block =
      tangent_matrix
          .topLeftCorner<3 * num_participating_vertices,
                         3 * num_participating_vertices>()
          .sparseView();
  const Eigen::SparseMatrix<double> non_participating_block =
      tangent_matrix.bottomRightCorner<3, 3>().sparseView();
  const Eigen::SparseMatrix<double> off_diagonal_block =
      tangent_matrix.bottomLeftCorner<3, 3 * num_participating_vertices>()
          .sparseView();
  const internal::SchurComplement<double> expected_schur_complement =
      internal::SchurComplement<double>(participating_block, off_diagonal_block,
                                        non_participating_block);
  EXPECT_TRUE(CompareMatrices(expected_schur_complement.get_D_complement(),
                              tangent_matrix_schur_complement,
                              std::numeric_limits<double>::epsilon()));
}

/* Verifies that the contact tangent matrix is empty when there is no contact.
 */
TEST_F(DeformableRigidDynamicsDataTest, NoContact) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Move the bodies far away from each other so that there is no contact at
   all. */
  const Vector3d p_WC(0, -5, 0);
  const math::RigidTransformd X_WC(p_WC);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(C_), X_WC);
  const MatrixXd tangent_matrix = CalcContactTangentMatrix(plant_context);
  EXPECT_EQ(tangent_matrix.rows(), 0);
  EXPECT_EQ(tangent_matrix.cols(), 0);
  const VectorX<double>& participating_v_star =
      EvalFreeMotionParticipatingVelocities(plant_context);
  EXPECT_EQ(participating_v_star.size(), 0);
  const VectorX<double>& participating_v0 =
      EvalParticipatingVelocities(plant_context);
  EXPECT_EQ(participating_v0.size(), 0);
}

/* Verifies that the contact tangent matrix matches expectation. */
TEST_F(DeformableRigidDynamicsDataTest, TangentMatrix) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Move the rigid body so that A and C are in contact but B and C are not.
   */
  const Vector3d p_WC(0, 2.5, 0);
  const math::RigidTransformd X_WC(p_WC);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(C_), X_WC);
  const MatrixXd tangent_matrix = CalcContactTangentMatrix(plant_context);
  /* Since only A and C are in contact and all vertices of A are participating
   in contact, the dimension of the tangent matrix should be 6 (rigid dofs) +
   the number of dofs in A (3 * kNumVertices). B does not show up in the
   tangent matrix. */
  EXPECT_EQ(tangent_matrix.rows(), 6 + 3 * kNumVertices);
  EXPECT_EQ(tangent_matrix.cols(), 6 + 3 * kNumVertices);
  SpatialInertia<double> M = MakeSpatialInertiaForBox();
  EXPECT_TRUE(CompareMatrices(tangent_matrix.topLeftCorner(6, 6),
                              M.CopyToFullMatrix6()));
  EXPECT_TRUE(
      CompareMatrices(tangent_matrix.topRightCorner(6, 3 * kNumVertices),
                      MatrixXd::Zero(6, 3 * kNumVertices)));
  EXPECT_TRUE(
      CompareMatrices(tangent_matrix.bottomLeftCorner(3 * kNumVertices, 6),
                      MatrixXd::Zero(3 * kNumVertices, 6)));
  /* Since *all* vertices in A are participating in contact, the Schur
   complement of the free motion tangent matrix of A is the free motion
   tangent matrix itself. */
  EXPECT_TRUE(CompareMatrices(
      tangent_matrix.bottomRightCorner(3 * kNumVertices, 3 * kNumVertices),
      CalcFreeMotionTangentMatrix(plant_context, A_)));
}

/* Verifies that the participating free motion velocities match expectation for
 a simple set up where a rigid unit cube intersects with the bottom half of a
 deformable octahedron. See the ASCII illustration below. */
TEST_F(DeformableRigidDynamicsDataTest, ParticipatingFreeMotionVelocities) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  /* Move the rigid cube down so that it intersects the octahedron in the lower
   prism.

   The setup viewed in the YZ plane looks like:
                 +Z
                  |
               v5 ●
                ⁄  | ＼
              ⁄    |   ＼
        v4  ⁄      |  B  ＼  v2
   -Y-----●-------●-------●----+Y
            \     |      ⁄
              \---|----⁄
              | \ |  ⁄ |
              |   ● v6|
              |   |   |
              ----|----  C
                  |
                 -Z
  Recall that the octahedron looks like:
                +Z   -X
                 |   /
              v5 ●  ● v3
                 | /
       v4     v0 |/
  -Y----●--------●------●----+Y
                /|      v2
               / |
           v1 ●  ● v6
             /   |
           +X    |
                -Z
  Therefore, all vertices except v5 are participating in contact. */
  const Vector3d p_WC(0, 0, -1);
  const math::RigidTransformd X_WC(p_WC);
  plant_->SetFreeBodyPose(&plant_context, plant_->get_body(C_), X_WC);
  const VectorXd& rigid_v_star = EvalFreeMotionRigidVelocities(plant_context);
  const VectorXd& deformable_v_star =
      EvalFreeMotionFemStateBase(plant_context, B_).qdot();
  constexpr int kNumRigidDofs = 6;
  EXPECT_EQ(rigid_v_star.size(), kNumRigidDofs);
  EXPECT_EQ(deformable_v_star.size(),
            7 /* number of vertices in the octehedral mesh */ * 3);

  const VectorXd& participating_v_star =
      EvalFreeMotionParticipatingVelocities(plant_context);
  /* v0-v4, and v6 are participating in contact. */
  constexpr int kNumParticipatingVertices = 6;
  constexpr int kNumParticipatingDofs = 3 * kNumParticipatingVertices;
  EXPECT_EQ(participating_v_star.size(), kNumRigidDofs + kNumParticipatingDofs);
  /* Verify rigid velocities for the participating C. */
  EXPECT_TRUE(CompareMatrices(participating_v_star.head<kNumRigidDofs>(),
                              rigid_v_star));
  /* Get the expected participating deformable velocities by permuting the
   vector of deformable velocities for all vertices. */
  const internal::DeformableContactData<double>& contact_data =
      EvalDeformableRigidContact(plant_context)[B_];
  const VectorXd expected_participating_deformable_v_star =
      internal::PermuteBlockVector<double>(
          deformable_v_star, contact_data.permuted_vertex_indexes())
          .head<kNumParticipatingDofs>();
  /* Verify deformable velocities for the participating B matches expectation.
   */
  EXPECT_TRUE(
      CompareMatrices(participating_v_star.tail<kNumParticipatingDofs>(),
                      expected_participating_deformable_v_star));

  const VectorXd& participating_v0 = EvalParticipatingVelocities(plant_context);
  EXPECT_TRUE(CompareMatrices(
      participating_v0, VectorXd::Zero(kNumRigidDofs + kNumParticipatingDofs)));
}

// TODO(xuchenhan-tri): This unit test can be strengthened by leveraging
//  the fixture in multibody_plant_forward_dynamics_test.cc when
//  DeformableRigidManager moves out of dev,
/* Verifies that the free motion rigid velocities match expectation. */
TEST_F(DeformableRigidDynamicsDataTest, FreeMotionRigidVelocities) {
  Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  const Vector3d w_WC(1, 2, 3);
  const Vector3d v_WC(4, 5, 6);
  const SpatialVelocity<double> V_WC(w_WC, v_WC);
  plant_->SetFreeBodySpatialVelocity(&plant_context,
                                     plant_->GetBodyByName("cube_C"), V_WC);
  const VectorXd rigid_v_star = EvalFreeMotionRigidVelocities(plant_context);
  EXPECT_TRUE(CompareMatrices(rigid_v_star.head<3>(), w_WC));
  EXPECT_TRUE(CompareMatrices(
      rigid_v_star.tail<3>(),
      v_WC + plant_->time_step() * plant_->gravity_field().gravity_vector()));
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
