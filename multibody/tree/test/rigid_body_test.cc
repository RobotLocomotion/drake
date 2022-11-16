#include "drake/multibody/tree/rigid_body.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
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
  const SpatialInertia<double> M_BBo_B_test =
      rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  EXPECT_TRUE(CompareMatrices(M_BBo_B_test.CopyToFullMatrix6(),
      M_BBo_B.CopyToFullMatrix6(), kTolerance));

  // Verify rotational inertia is of a single particle at B's center of mass.
  RotationalInertia<double> I_BBo_B_expected(mass, p_BoBcm_B);
  RotationalInertia<double> I_BBo_B = M_BBo_B.CalcRotationalInertia();
  EXPECT_TRUE(CompareMatrices(I_BBo_B.CopyToFullMatrix3(),
      I_BBo_B_expected.CopyToFullMatrix3(), kTolerance));

  // Verify B's unit inertia about Bcm is zero.
  SpatialInertia<double> M_BBcm_B = M_BBo_B.Shift(p_BoBcm_B);
  UnitInertia<double> G_BBcm_B = M_BBcm_B.get_unit_inertia();
  UnitInertia<double> G_BBcm_B_expected(0, 0, 0);
  EXPECT_TRUE(G_BBcm_B.IsNearlyEqualTo(G_BBcm_B_expected, kTolerance));

  // Change the body's mass to 1 and ensure it propagates properly, meaning the
  // mass changes, p_BoBcm_B is unchanged, G_BBo_B and G_BBcm_B are unchanged.
  mass = 1;
  rigid_body_->SetMass(context_ptr, mass);
  EXPECT_EQ(rigid_body_->get_mass(*context_), mass);
  M_BBo_B = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  SpatialInertia<double> M_BBo_B_expected(mass, p_BoBcm_B, G_BBo_B);
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
      M_BBo_B_expected.CopyToFullMatrix6(), kTolerance));
  M_BBcm_B = M_BBo_B.Shift(p_BoBcm_B);
  G_BBcm_B = M_BBcm_B.get_unit_inertia();
  EXPECT_TRUE(G_BBcm_B.IsNearlyEqualTo(G_BBcm_B_expected, kTolerance));

  // Change p_BoBcm_B and ensure it propagates albeit in a weird way, with
  // unchanged mass, changed p_Bo_Bcm_B, unchanged G_BBo_B, changed G_BBcm_B.
  p_BoBcm_B = Vector3d(L/2, 0, 0);  // Now p_BoBcm_B = (1, 0, 0).
  rigid_body_->SetCenterOfMassInBodyFrame(context_ptr, p_BoBcm_B);
  const Vector3d p_BoBcm_B_calculated =
      rigid_body_->CalcCenterOfMassInBodyFrame(*context_);
  EXPECT_EQ(p_BoBcm_B_calculated, p_BoBcm_B);      // Position vector changes.
  EXPECT_EQ(rigid_body_->get_mass(*context_), mass);  // Mass is unchanged.
  M_BBo_B = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  const UnitInertia<double> G_BBo_B_test = M_BBo_B.get_unit_inertia();
  EXPECT_TRUE(CompareMatrices(G_BBo_B_test.CopyToFullMatrix3(),
      G_BBo_B.CopyToFullMatrix3(), kTolerance));  // G_BBo_B is unchanged.

  // Ensure M_BBo_B has proper mass, proper p_BoBcm_B, and unchanged G_BBo_B.
  M_BBo_B_expected = SpatialInertia<double>(mass, p_BoBcm_B, G_BBo_B);
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
      M_BBo_B_expected.CopyToFullMatrix6(), kTolerance));

  // Verify G_BBcm_B changes in a strange way if p_BBcm_B is changed!
  // G_BBcm_B is _not_ the unit inertia of a particle at B's center of mass.
  G_BBcm_B = G_BBo_B.ShiftToCenterOfMass(p_BoBcm_B);
  G_BBcm_B_expected = UnitInertia<double>(0, 3, 3);  // By-hand calculation.
  EXPECT_TRUE(CompareMatrices(G_BBcm_B.CopyToFullMatrix3(),
    G_BBcm_B_expected.CopyToFullMatrix3(), kTolerance));

  // Show that calling SetCenterOfMassInBodyFrame() can cause an exception to
  // be thrown due to non-physical mass/inertia properties.
#ifdef DRAKE_ASSERT_IS_ARMED
  p_BoBcm_B = Vector3d(3*L, 0, 0);  // Now p_BoBcm_B = (6, 0, 0).
  rigid_body_->SetCenterOfMassInBodyFrame(context_ptr, p_BoBcm_B);

  // CalcSpatialInertiaInBodyFrame() does not check whether B's inertia
  // properties make sense. However, Shift() does (whether to Bcm or elsewhere).
  M_BBo_B = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  DRAKE_EXPECT_THROWS_MESSAGE(M_BBo_B.Shift(p_BoBcm_B),  // M_BBcm_B is invalid.
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid[^]*");
#endif
}

}  // namespace
}  // namespace multibody
}  // namespace drake
