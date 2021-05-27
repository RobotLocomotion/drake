#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
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
constexpr double kBoxLength = 1;
const CoulombFriction kFriction{0.3, 0.2};
using Eigen::VectorXd;
using geometry::GeometryId;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using geometry::SurfaceMesh;

class DeformableRigidManagerTest : public ::testing::Test {
 protected:
  /* Builds a deformable model with a single deformable body and adds it to
   MultibodyPlant. Then sets a DeformableRigidManager as the discrete update
   manager for the MultibodyPlant. */
  void SetUp() override {
    auto deformable_model = std::make_unique<DeformableModel<double>>(&plant_);
    deformable_model->RegisterDeformableBody(MakeBoxTetMesh(), "box",
                                             MakeDeformableBodyConfig(),
                                             MakeProximityProperties());
    deformable_model_ = &plant_.AddPhysicalModel(std::move(deformable_model));
    /* Add a collision geometry. */
    plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
    plant_.RegisterCollisionGeometry(
        plant_.world_body(), math::RigidTransform<double>(), MakeBoxShape(),
        "collision", MakeProximityProperties());
    plant_.Finalize();
    auto deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>();
    deformable_rigid_manager_ = &plant_.set_discrete_update_manager(
        std::move(deformable_rigid_manager));
    deformable_rigid_manager_->RegisterCollisionObjects(scene_graph_);
  }

  static geometry::Box MakeBoxShape() {
    return geometry::Box(kBoxLength, kBoxLength, kBoxLength);
  }

  /* Makes a box and subdivide it into 6 tetrahedra. */
  static geometry::VolumeMesh<double> MakeBoxTetMesh() {
    geometry::VolumeMesh<double> mesh =
        geometry::internal::MakeBoxVolumeMesh<double>(MakeBoxShape(),
                                                      kBoxLength);
    DRAKE_DEMAND(mesh.num_elements() == 6);
    DRAKE_DEMAND(mesh.num_vertices() == kNumVertices);
    return mesh;
  }

  /* Returns a proximity property with default elastic modulus and dissipation
   and an arbitrary friction. */
  static ProximityProperties MakeProximityProperties() {
    ProximityProperties proximity_properties;
    geometry::AddContactMaterial({}, {}, kFriction, &proximity_properties);
    return proximity_properties;
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

// TODO(xuchenhan-tri): Update the unit test once the CalcContactSolverResults()
//  method is implemented.
TEST_F(DeformableRigidManagerTest, CalcContactSolverResults) {
  auto context = plant_.CreateDefaultContext();
  contact_solvers::internal::ContactSolverResults<double> results;
  EXPECT_THROW(
      deformable_rigid_manager_->CalcContactSolverResults(*context, &results),
      std::exception);
}

// TODO(xuchenhan-tri): Update the unit test once the
//  CalcAccelerationKinematicsCache() method is implemented.
TEST_F(DeformableRigidManagerTest, CalcAccelerationKinematicsCache) {
  auto context = plant_.CreateDefaultContext();
  EXPECT_THROW(plant_.get_generalized_acceleration_output_port().Eval(*context),
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
  const SurfaceMesh<double> expected_surface_mesh =
      geometry::internal::MakeBoxSurfaceMesh<double>(MakeBoxShape(),
                                                     kBoxLength);
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

}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
