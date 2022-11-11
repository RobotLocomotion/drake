#include "drake/multibody/tree/rigid_body.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/tree/multibody_tree-inl.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using systems::Context;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test rigid body constructor.
GTEST_TEST(RigidBody, RigidBodyConstructor) {
  // Construct a rigid body with a spatial inertia.
  const double mass = 2;
  const Vector3d p_BoBcm_B(0.4, 0.3, 0.2);
  const UnitInertia<double> G_BBo_B(6, 7, 8);
  const SpatialInertia<double> M_Bo_B(mass, p_BoBcm_B, G_BBo_B);
  const RigidBody<double> B("B", M_Bo_B);

  // Test that after construction, RigidBody class properly returns its default
  // spatial inertia or parts of it.
  // TODO(amcastro-tri): Replace with EXPECT_EQ once operator==() is overloaded.
  EXPECT_EQ(B.default_spatial_inertia().get_mass(), M_Bo_B.get_mass());
  EXPECT_EQ(B.default_spatial_inertia().get_com(), M_Bo_B.get_com());
  EXPECT_EQ(B.default_spatial_inertia().get_unit_inertia().get_moments(),
            M_Bo_B.get_unit_inertia().get_moments());
  EXPECT_EQ(B.default_spatial_inertia().get_unit_inertia().get_products(),
            M_Bo_B.get_unit_inertia().get_products());
  EXPECT_EQ(B.default_mass(), mass);
  EXPECT_EQ(B.default_com(), p_BoBcm_B);
  const UnitInertia<double>& G_BBo_B_default = B.default_unit_inertia();
  EXPECT_EQ(G_BBo_B_default.get_moments(), G_BBo_B.get_moments());
  EXPECT_EQ(G_BBo_B_default.get_products(), G_BBo_B.get_products());

  // Test that RigidBody class properly calculates rotational inertia.
  const RotationalInertia<double> I_BBo_B_expected = mass * G_BBo_B;
  const RotationalInertia<double> I_BBo_B = B.default_rotational_inertia();
  EXPECT_TRUE(I_BBo_B.IsNearlyEqualTo(I_BBo_B_expected, 4.0*kEpsilon));
}

// Test rigid body constructor passing a string name.
GTEST_TEST(RigidBody, RigidBodyConstructorWithName) {
  const std::string kLinkName = "LinkName";
  // For this test the numerical values of the spatial inertia are not
  // important and therefore it is left uninitialized.
  const SpatialInertia<double> M_Bo_B;
  const RigidBody<double> B("LinkName", M_Bo_B);
  EXPECT_EQ(B.name(), kLinkName);
}

// Fixture for a MultibodyTree model with a single rigid body and a Context.
class RigidBodyTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Create an empty model and then add a rigid body.
    auto model = std::make_unique<internal::MultibodyTree<double>>();
    const SpatialInertia<double> M_BBo_B;  // Default constructor is OK here
    rigid_body_ = &model->AddBody<RigidBody>("rigidBody_B", M_BBo_B);

    // Finalize the model and create a default context for this system.
    model->Finalize();
    mbtree_system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model));
    context_ = mbtree_system_->CreateDefaultContext();
  }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> mbtree_system_;
  std::unique_ptr<Context<double>> context_;
  const RigidBody<double>* rigid_body_{nullptr};
};

// Test RigidBody::SetCenterOfMassInBodyFrame(). Reminder: calling this function
// for a rigid body B does not modify I_BBo_B (B's rotational inertia about B's
// body origin Bo) which can cause an invalid I_BBcm_B (B's rotational inertia
// about B's center of mass Bcm).
TEST_F(RigidBodyTest, SetCenterOfMassInBodyFrame) {
  // Construct a multibody plant having a cube B of dimension 2*L with a spatial
  // inertia with all the mass concentrated at Bcm (B's center of mass).
  double mass = 3;     // mass of body B in kilograms.
  const double L = 2;  // x-measure of Bcm's position from Bo in meters.
  Vector3d p_BoBcm_B(L, 0, 0);  // Position from Bo to Bcm, expressed in B.
  const UnitInertia<double> G_BBo_B(0, L*L, L*L);
  SpatialInertia<double> M_BBo_B(mass, p_BoBcm_B, G_BBo_B);
  systems::Context<double>* context_ptr = context_.get();
  rigid_body_->SetSpatialInertiaInBodyFrame(context_ptr, M_BBo_B);

  // Test that the cube has the proper mass, center of mass, inertia properties.
  constexpr double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  SpatialInertia<double> M_BBo_B_test =
      rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  EXPECT_TRUE(CompareMatrices(M_BBo_B_test.CopyToFullMatrix6(),
      M_BBo_B.CopyToFullMatrix6(), kTolerance));

  // Verify rotational inertia is of a single particle at B's center of mass.
  RotationalInertia<double> I_BBo_B_expected(mass, p_BoBcm_B);
  RotationalInertia<double> I_BBo_B = M_BBo_B_test.CalcRotationalInertia();
  EXPECT_TRUE(CompareMatrices(I_BBo_B.CopyToFullMatrix3(),
      I_BBo_B_expected.CopyToFullMatrix3(), kTolerance));

  // Verify B's rotational inertia about Bcm is zero.
  SpatialInertia<double> M_BBcm_B = M_BBo_B.Shift(p_BoBcm_B);
  RotationalInertia<double> I_BBcm_B = M_BBcm_B.CalcRotationalInertia();
  RotationalInertia<double> I_BBcm_B_expected(0, 0, 0);
  EXPECT_TRUE(I_BBcm_B.IsNearlyEqualTo(I_BBcm_B_expected, kTolerance));

  // Change the body's mass to 1 and ensure that it propagates properly.
  mass = 1;
  rigid_body_->SetMass(context_ptr, mass);
  EXPECT_EQ(rigid_body_->get_mass(*context_ptr), mass);
  M_BBo_B_test = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  SpatialInertia<double> M_BBo_B_expected(mass, p_BoBcm_B, G_BBo_B);
  EXPECT_TRUE(CompareMatrices(M_BBo_B_test.CopyToFullMatrix6(),
      M_BBo_B_expected.CopyToFullMatrix6(), kTolerance));

  // Change p_BoBcm_B and ensure mass is unchanged (not surprising).
  // Verify I_BBo_B = mass * G_BBo_B is unchanged.
  p_BoBcm_B = Vector3d(L/2, 0, 0);
  rigid_body_->SetCenterOfMassInBodyFrame(context_ptr, p_BoBcm_B);
  EXPECT_EQ(rigid_body_->get_mass(*context_ptr), mass);
  EXPECT_EQ(rigid_body_->CalcCenterOfMassInBodyFrame(*context_ptr), p_BoBcm_B);
  M_BBo_B_test = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  M_BBo_B_expected = SpatialInertia<double>(mass, p_BoBcm_B, G_BBo_B);
  EXPECT_TRUE(CompareMatrices(M_BBo_B_test.CopyToFullMatrix6(),
      M_BBo_B_expected.CopyToFullMatrix6(), kTolerance));

#if 0
  // Verify I_BBcm_B changes in an intelligible way.
  M_BBcm_B = M_BBo_B.Shift(p_BoBcm_B);
  I_BBcm_B = M_BBcm_B.CalcRotationalInertia();;
  I_BBcm_B_expected = RotationalInertia<double>(mass, p_BoBcm_B);
  EXPECT_TRUE(CompareMatrices(I_BBcm_B.CopyToFullMatrix3(),
      I_BBcm_B_expected.CopyToFullMatrix3(), kTolerance));
#endif
}

}  // namespace
}  // namespace multibody
}  // namespace drake
