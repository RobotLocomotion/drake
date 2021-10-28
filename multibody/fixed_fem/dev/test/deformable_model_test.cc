#include "drake/multibody/fixed_fem/dev/deformable_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace fem {
// TODO(xuchenhan-tri): Add a test to verify that the deformable body parameters
//  are properly passed to the FemModel.
/* Deformable body parameters. These parameters are arbitrary and they do not
 affect the result of the test as long as they are valid. */
const double kYoungsModulus = 1.23;
const double kPoissonRatio = 0.456;
const double kDensity = 0.789;
const double kMassDamping = 0.01;
const double kStiffnessDamping = 0.02;
/* Time step. */
const double kDt = 0.0123;
/* Number of vertices in the box mesh (see below). */
constexpr int kNumVertices = 8;

using Eigen::VectorXd;
using geometry::VolumeMesh;
using geometry::VolumeMeshFieldLinear;

class DeformableModelTest : public ::testing::Test {
 protected:
  /* Make a box and subdivide it into 6 tetrahedra. */
  static VolumeMesh<double> MakeBoxTetMesh() {
    const double length = 1;
    geometry::Box box(length, length, length);
    VolumeMesh<double> mesh =
        geometry::internal::MakeBoxVolumeMesh<double>(box, length);
    DRAKE_DEMAND(mesh.num_elements() == 6);
    DRAKE_DEMAND(mesh.num_vertices() == kNumVertices);
    return mesh;
  }

  static internal::ReferenceDeformableGeometry<double>
  MakeBoxDeformableGeometry() {
    auto mesh = std::make_unique<VolumeMesh<double>>(MakeBoxTetMesh());
    /* All vertices of the box mesh lie on the surface. */
    std::vector<double> signed_distances(kNumVertices, 0.0);
    auto mesh_field = std::make_unique<VolumeMeshFieldLinear<double, double>>(
        std::move(signed_distances), mesh.get(), false);
    return {std::move(mesh), std::move(mesh_field)};
  }

  /* Create a dummy DeformableConfig. */
  static DeformableBodyConfig<double> MakeDeformableConfig() {
    DeformableBodyConfig<double> config;
    config.set_youngs_modulus(kYoungsModulus);
    config.set_poisson_ratio(kPoissonRatio);
    config.set_mass_damping_coefficient(kMassDamping);
    config.set_stiffness_damping_coefficient(kStiffnessDamping);
    config.set_mass_density(kDensity);
    config.set_material_model(MaterialModel::kLinear);
    return config;
  }

  /* Creates a dummy proximity property. */
  static geometry::ProximityProperties MakeProximityProps() {
    geometry::ProximityProperties dummy_proximity_props;
    geometry::AddContactMaterial({}, {},
                                 multibody::CoulombFriction<double>(0, 0),
                                 &dummy_proximity_props);
    return dummy_proximity_props;
  }

  /* Add a dummy box shaped deformable body with the given "name". */
  int AddDeformableBox(DeformableModel<double>* deformable_model,
                       std::string name) {
    return deformable_model_->RegisterDeformableBody(
        MakeBoxDeformableGeometry(), std::move(name), MakeDeformableConfig(),
        MakeProximityProps());
  }

  MultibodyPlant<double> plant_{kDt};
  /* The DeformableModel under test. */
  std::unique_ptr<DeformableModel<double>> deformable_model_{
      std::make_unique<DeformableModel<double>>(&plant_)};
};

namespace {
TEST_F(DeformableModelTest, RegisterDeformableBody) {
  AddDeformableBox(deformable_model_.get(), "box");
  EXPECT_EQ(deformable_model_->num_bodies(), 1);
  const std::vector<std::string>& registered_names = deformable_model_->names();
  EXPECT_EQ(registered_names.size(), 1);
  EXPECT_EQ(registered_names[0], "box");
  const std::vector<internal::ReferenceDeformableGeometry<double>>&
      reference_configuration_geometries =
          deformable_model_->reference_configuration_geometries();
  ASSERT_EQ(reference_configuration_geometries.size(), 1);
  EXPECT_TRUE(
      MakeBoxTetMesh().Equal(reference_configuration_geometries[0].mesh()));
}

/* Verifies that registering a deformable body returns the expected body id and
 that registering a body with an existing name throws an exception. */
TEST_F(DeformableModelTest, RegisterDeformableBodyUniqueNameRequirement) {
  EXPECT_EQ(AddDeformableBox(deformable_model_.get(), "box1"),
            DeformableBodyIndex(0));
  /* The returned body index should be the same as the number of deformable
   bodies in the system before the new one is added. */
  EXPECT_EQ(AddDeformableBox(deformable_model_.get(), "box2"),
            DeformableBodyIndex(1));
  EXPECT_EQ(deformable_model_->num_bodies(), 2);
  DRAKE_EXPECT_THROWS_MESSAGE(AddDeformableBox(deformable_model_.get(), "box1"),
                              "RegisterDeformableBody\\(\\): A body with name "
                              "'box1' already exists in the system.");
}

TEST_F(DeformableModelTest, VertexPositionsOutputPort) {
  AddDeformableBox(deformable_model_.get(), "box");
  DeformableModel<double>* deformable_model = deformable_model_.get();
  plant_.AddPhysicalModel(std::move(deformable_model_));
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();
  const std::vector<VectorXd> vertex_positions_vector =
      deformable_model->get_vertex_positions_output_port()
          .Eval<std::vector<VectorXd>>(*context);
  EXPECT_EQ(vertex_positions_vector.size(), 1);
  const VectorXd& vertex_positions = vertex_positions_vector[0];
  EXPECT_EQ(vertex_positions.size(), 3 * kNumVertices);
  const auto& expected_mesh = MakeBoxTetMesh();
  for (int i = 0; i < kNumVertices; ++i) {
    EXPECT_TRUE(CompareMatrices(expected_mesh.vertex(i),
                                vertex_positions.segment<3>(3 * i)));
  }
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
