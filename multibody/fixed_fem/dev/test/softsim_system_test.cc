#include "drake/multibody/fixed_fem/dev/softsim_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/* Deformable body parameters. */
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

class SoftsimSystemTest : public ::testing::Test {
 protected:
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

  /* Create a dummy DeformableConfig. */
  static DeformableBodyConfig<double> MakeDeformableConfig() {
    DeformableBodyConfig<double> config;
    config.set_youngs_modulus(kYoungsModulus);
    config.set_poisson_ratio(kPoissonRatio);
    config.set_mass_damping_coefficient(kMassDamping);
    config.set_stiffness_damping_coefficient(kStiffnessDamping);
    config.set_mass_density(kDensity);
    config.set_material_model(
        DeformableBodyConfig<double>::MaterialModel::Linear);
    return config;
  }

  /* Add a dummy box shaped deformable body with the given "name". */
  int AddDeformableBox(std::string name) {
    return softsim_system_.RegisterDeformableBody(
        MakeBoxTetMesh(), std::move(name), MakeDeformableConfig());
  }

  void SetUp() override { AddDeformableBox("box"); }

  /* The SoftsimSystem under test. */
  SoftsimSystem<double> softsim_system_{kDt};
};

namespace {
TEST_F(SoftsimSystemTest, Construction) {
  EXPECT_EQ(softsim_system_.num_bodies(), 1);
  const std::vector<std::string>& registered_names = softsim_system_.names();
  EXPECT_EQ(registered_names.size(), 1);
  EXPECT_EQ(registered_names[0], "box");
  const std::vector<geometry::VolumeMesh<double>>& initial_meshes =
      softsim_system_.initial_meshes();
  EXPECT_EQ(initial_meshes.size(), 1);
  EXPECT_TRUE(MakeBoxTetMesh().Equal(initial_meshes[0]));
  EXPECT_EQ(softsim_system_.dt(), kDt);
}

/* Verifies that registering a deformable body returns the expected body id and
 that registering a body with an existing name throws an exception. */
TEST_F(SoftsimSystemTest, RegisterDeformableBody) {
  /* The returned body index should be the same as the number of deformable
   bodies in the system before the new one is added. */
  EXPECT_EQ(AddDeformableBox("box2"), 1);
  EXPECT_EQ(softsim_system_.num_bodies(), 2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddDeformableBox("box"), std::exception,
      "A body with name 'box' already exists in the system.");
}

/* Verifies that the SoftsimSystem calculates the expected displacement for a
 deformable object under free fall over one time step. */
TEST_F(SoftsimSystemTest, AdvanceOneTimeStep) {
  auto context = softsim_system_.CreateDefaultContext();
  auto simulator =
      systems::Simulator<double>(softsim_system_, std::move(context));
  const auto& vertex_position_port =
      softsim_system_.get_vertex_positions_output_port();
  const std::vector<VectorX<double>> initial_positions =
      vertex_position_port.Eval<std::vector<VectorX<double>>>(
          simulator.get_context());
  EXPECT_EQ(initial_positions.size(), 1);
  EXPECT_EQ(initial_positions[0].size(), kNumVertices * 3);
  /* Advance a single time step. */
  const double final_time = kDt;
  simulator.AdvanceTo(final_time);
  const std::vector<VectorX<double>> current_positions =
      vertex_position_port.Eval<std::vector<VectorX<double>>>(
          simulator.get_context());
  EXPECT_EQ(current_positions.size(), 1);
  EXPECT_EQ(current_positions[0].size(), kNumVertices * 3);
  /* The factor of 0.25 seems strange but is correct. For the default mid-point
   rule used by DynamicElasticityModel,
        x = xₙ + dt ⋅ vₙ + dt² ⋅ (0.25 ⋅ a + 0.25 ⋅ aₙ).
   In this test case vₙ and aₙ are both 0, so x - xₙ is given by
   0.25 ⋅ a ⋅ dt². */
  const Vector3<double> expected_displacement(
      0, 0, 0.25 * kGravity * final_time * final_time);
  const double kTol = 1e-14;
  for (int i = 0; i < kNumVertices; ++i) {
    const Vector3<double> displacement =
        current_positions[0].segment<3>(3 * i) -
        initial_positions[0].segment<3>(3 * i);
    EXPECT_TRUE(CompareMatrices(displacement, expected_displacement, kTol));
  }
}

}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
