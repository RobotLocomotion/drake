#include "drake/multibody/multibody_tree/multibody_forcing.h"

#include <limits>
#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using std::numeric_limits;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

class MultibodyForcingTests : public ::testing::Test {
 public:
  // Creates a simple MultibodyTree model so that we can instantiate
  // MultibodyForcing objects for this model.
  void SetUp() override {
    SpatialInertia<double> M;
    const RigidBody<double>& body1 = model_.AddBody<RigidBody>(M);
    const RigidBody<double>& body2 = model_.AddBody<RigidBody>(M);
    model_.AddJoint<RevoluteJoint>(
        "Joint1", model_.get_world_body(), {}, body1, {}, Vector3d::UnitZ());
    model_.AddJoint<RevoluteJoint>(
        "Joint2", body1, {}, body2, {}, Vector3d::UnitZ());
    model_.Finalize();
  }
 protected:
  MultibodyTree<double> model_;
};

// Test constructor that sets forcing to zero.
TEST_F(MultibodyForcingTests, Construction) {
  // Create a forcing object compatible with model:
  MultibodyForcing<double> forcing(model_);

  // Forcing object should be compatible with the original model.
  EXPECT_TRUE(forcing.CheckInvariants(model_));
  EXPECT_EQ(forcing.num_bodies(), model_.get_num_bodies());
  EXPECT_EQ(forcing.num_mobilities(), model_.get_num_velocities());

  EXPECT_TRUE(forcing.generalized_forces() == Vector2d::Zero());

  for (const SpatialForce<double>& F : forcing.body_forces()) {
    EXPECT_TRUE(F.IsApprox(SpatialForce<double>::Zero(), kEpsilon));
  }
}

// A number of unit tests involving non-zero forcing.
TEST_F(MultibodyForcingTests, NonZeroForcing) {
  // Create a non-zero forcing:
  MultibodyForcing<double> forcing1(model_);
  forcing1.mutable_generalized_forces() = Vector2d(1, 2);
  forcing1.mutable_body_forces()[1] =
      SpatialForce<double>(Vector3d(0, 1, 2), Vector3d(3, 4, 5));
  forcing1.mutable_body_forces()[2] =
      SpatialForce<double>(Vector3d(0, 1, 2), Vector3d(3, 4, 5));

  // Create a second non-zero forcing:
  MultibodyForcing<double> forcing2(model_);
  ASSERT_EQ(forcing2.num_bodies(), 3);
  ASSERT_EQ(forcing2.num_mobilities(), 2);
  forcing2.mutable_generalized_forces() = Vector2d(3, 4);
  forcing2.mutable_body_forces()[0] =
      SpatialForce<double>(Vector3d(6, 7, 8), Vector3d(9, 10, 11));
  forcing2.mutable_body_forces()[2] =
      SpatialForce<double>(Vector3d(6, 7, 8), Vector3d(9, 10, 11));

  // Add-in forcing2 to forcing1:
  forcing1.AddInForcing(forcing2);

  // Const aliases:
  const VectorX<double>& generalized_forces = forcing1.generalized_forces();
  const std::vector<SpatialForce<double>>& spatial_forces =
      forcing1.body_forces();

  // Check the results:
  EXPECT_EQ(generalized_forces, Vector2d(4, 6));
  EXPECT_TRUE(CompareMatrices(
      spatial_forces[0].get_coeffs(),
      (Vector6<double>() << 6, 7, 8, 9, 10, 11).finished(),
      kEpsilon, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      spatial_forces[1].get_coeffs(),
      (Vector6<double>() << 0, 1, 2, 3, 4, 5).finished(),
      kEpsilon, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      spatial_forces[2].get_coeffs(),
      (Vector6<double>() << 6, 8, 10, 12, 14, 16).finished(),
      kEpsilon, MatrixCompareType::absolute));

  // Set to zero and assess the result:
  forcing1.SetZero();
  EXPECT_TRUE(forcing1.generalized_forces() == Vector2d::Zero());
  for (const SpatialForce<double>& F : forcing1.body_forces()) {
    EXPECT_TRUE(F.IsApprox(SpatialForce<double>::Zero(), kEpsilon));
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
