#include "drake/multibody/fixed_fem/dev/softsim_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
// TODO(xuchenhan-tri): Add a test to verify that the deformable body parameters
//  are properly passed to the FemModel.
/* Deformable body parameters. These parameters (with the exception of
 kMassDamping) are dummy in the sense that they do not affect the result of
 the test as long as they are valid. */
const double kYoungsModulus = 1.23;
const double kPoissonRatio = 0.456;
const double kDensity = 0.789;
/* The mass damping coefficient is set to zero so that the free fall test
 (CalcFreeMotion) produces an easy analytical solution. */
const double kMassDamping = 0.0;
const double kStiffnessDamping = 0.02;
/* Time step. */
const double kDt = 0.0123;
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
    config.set_material_model(MaterialModel::kLinear);
    return config;
  }

  /* Forwards the call to CalcFreeMotion() to the SoftsimSystem under test. */
  VectorX<double> CalcFreeMotion(const VectorX<double>& state0,
                                 int deformable_body_index) const {
    return softsim_system_.CalcFreeMotion(state0, deformable_body_index);
  }

  double dt() const { return softsim_system_.dt(); }
  const Vector3<double>& gravity() const { return softsim_system_.gravity(); }

  /* Add a dummy box shaped deformable body with the given "name". */
  SoftBodyIndex AddDeformableBox(std::string name) {
    return softsim_system_.RegisterDeformableBody(
        MakeBoxTetMesh(), std::move(name), MakeDeformableConfig());
  }

  MultibodyPlant<double> mbp_{kDt};
  /* The SoftsimSystem under test. */
  SoftsimSystem<double> softsim_system_{&mbp_};
};

namespace {
TEST_F(SoftsimSystemTest, RegisterDeformableBody) {
  AddDeformableBox("box");
  EXPECT_EQ(softsim_system_.num_bodies(), 1);
  const std::vector<std::string>& registered_names = softsim_system_.names();
  EXPECT_EQ(registered_names.size(), 1);
  EXPECT_EQ(registered_names[0], "box");
  const std::vector<geometry::VolumeMesh<double>>& initial_meshes =
      softsim_system_.initial_meshes();
  EXPECT_EQ(initial_meshes.size(), 1);
  EXPECT_TRUE(MakeBoxTetMesh().Equal(initial_meshes[0]));
  EXPECT_EQ(dt(), kDt);
  EXPECT_TRUE(
      CompareMatrices(gravity(), mbp_.gravity_field().gravity_vector()));
}

/* Verifies that registering a deformable body returns the expected body id and
 that registering a body with an existing name throws an exception. */
TEST_F(SoftsimSystemTest, RegisterDeformableBodyUniqueNameRequirement) {
  EXPECT_EQ(AddDeformableBox("box1"), SoftBodyIndex(0));
  /* The returned body index should be the same as the number of deformable
   bodies in the system before the new one is added. */
  EXPECT_EQ(AddDeformableBox("box2"), SoftBodyIndex(1));
  EXPECT_EQ(softsim_system_.num_bodies(), 2);
  DRAKE_EXPECT_THROWS_MESSAGE(AddDeformableBox("box1"), std::exception,
                              "RegisterDeformableBody\\(\\): A body with name "
                              "'box1' already exists in the system.");
}

/* Verifies that the SoftsimSystem calculates the expected displacement for a
 deformable object under free fall over one time step. */
TEST_F(SoftsimSystemTest, CalcFreeMotion) {
  const SoftBodyIndex box_id = AddDeformableBox("box");
  const std::vector<geometry::VolumeMesh<double>>& meshes =
      softsim_system_.initial_meshes();
  const geometry::VolumeMesh<double>& box_mesh = meshes[box_id];
  const int num_dofs = kNumVertices * 3;
  VectorX<double> q0(num_dofs);
  for (geometry::VolumeVertexIndex i(0); i < box_mesh.num_vertices(); ++i) {
    q0.segment<3>(3 * i) = box_mesh.vertex(i).r_MV();
  }
  VectorX<double> x0 = VectorX<double>::Zero(3 * num_dofs);
  x0.head(num_dofs) = q0;
  const VectorX<double> x = CalcFreeMotion(x0, box_id);
  EXPECT_EQ(x.size(), 3 * num_dofs);
  const VectorX<double> q = x.head(num_dofs);
  /* The factor of 0.25 seems strange but is correct. For the default mid-point
   rule used by DynamicElasticityModel,
        x = xₙ + dt ⋅ vₙ + dt² ⋅ (0.25 ⋅ a + 0.25 ⋅ aₙ).
   In this test case vₙ and aₙ are both 0, so x - xₙ is given by
   0.25 ⋅ a ⋅ dt². */
  const Vector3<double> expected_displacement = 0.25 * kDt * kDt * gravity();
  const double kTol = 1e-14;
  for (int i = 0; i < kNumVertices; ++i) {
    const Vector3<double> displacement =
        q.segment<3>(3 * i) - q0.segment<3>(3 * i);
    EXPECT_TRUE(CompareMatrices(displacement, expected_displacement, kTol));
  }
}

}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
