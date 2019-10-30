#include "drake/multibody/tree/multibody_forces.h"

#include <array>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/rotational_inertia.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using std::numeric_limits;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

class MultibodyForcesTests : public ::testing::Test {
 public:
  // Creates a simple MultibodyTree model so that we can instantiate
  // MultibodyForces objects for this model.
  void SetUp() override {
    SpatialInertia<double> M;
    const RigidBody<double>& body1 = model_.AddBody<RigidBody>(M);
    const RigidBody<double>& body2 = model_.AddBody<RigidBody>(M);
    model_.AddJoint<RevoluteJoint>("Joint1", model_.world_body(), std::nullopt,
                                   body1, std::nullopt, Vector3d::UnitZ());
    model_.AddJoint<RevoluteJoint>("Joint2", body1, std::nullopt, body2,
                                   std::nullopt, Vector3d::UnitZ());
    model_.Finalize();
  }
 protected:
  internal::MultibodyTree<double> model_;
};

// Test constructor that sets forces to zero.
TEST_F(MultibodyForcesTests, Construction) {
  // We create a multibody forces object within a dirty memory location so that
  // we can assess the zeroing performed by the constructor. That is, the stored
  // forces would not be zero if the constructor had not explicitly zeroed them.
  std::array<char, sizeof(MultibodyForces<double>)> mem;  // memory buffer.
  mem.fill(1);  // fill in with garbage.
  auto forces = new(&mem) MultibodyForces<double>(model_);  // placement new.
  ASSERT_NE(forces, nullptr);

  // Forces object should be compatible with the original model.
  EXPECT_TRUE(forces->CheckHasRightSizeForModel(model_));

  // Test the API to retrieve sizes.
  EXPECT_EQ(forces->num_bodies(), model_.num_bodies());
  EXPECT_EQ(forces->num_velocities(), model_.num_velocities());

  // Assess the constructor did zero the forces.
  EXPECT_TRUE(forces->generalized_forces() == Vector2d::Zero());
  for (const SpatialForce<double>& F : forces->body_forces()) {
    EXPECT_TRUE(F.IsApprox(SpatialForce<double>::Zero(), 0));
  }

  // Since we used a placement new, we must now explicitly call the object's
  // destructor in order to perform a proper cleanup.
  forces->~MultibodyForces<double>();
}

// A number of unit tests involving non-zero forces.
TEST_F(MultibodyForcesTests, NonZeroForces) {
  // Create a non-zero forces:
  MultibodyForces<double> forces1(model_);
  forces1.mutable_generalized_forces() = Vector2d(1, 2);
  forces1.mutable_body_forces()[1] =
      SpatialForce<double>(Vector3d(0, 1, 2), Vector3d(3, 4, 5));
  forces1.mutable_body_forces()[2] =
      SpatialForce<double>(Vector3d(0, 1, 2), Vector3d(3, 4, 5));

  // Create a second non-zero forces:
  MultibodyForces<double> forces2(model_);
  ASSERT_EQ(forces2.num_bodies(), 3);
  ASSERT_EQ(forces2.num_velocities(), 2);
  forces2.mutable_generalized_forces() = Vector2d(3, 4);
  forces2.mutable_body_forces()[0] =
      SpatialForce<double>(Vector3d(6, 7, 8), Vector3d(9, 10, 11));
  forces2.mutable_body_forces()[2] =
      SpatialForce<double>(Vector3d(6, 7, 8), Vector3d(9, 10, 11));

  // Add-in forces2 to forces1:
  forces1.AddInForces(forces2);

  // Const aliases:
  const VectorX<double>& generalized_forces = forces1.generalized_forces();
  const std::vector<SpatialForce<double>>& spatial_forces =
      forces1.body_forces();

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
  forces1.SetZero();
  EXPECT_TRUE(forces1.generalized_forces() == Vector2d::Zero());
  for (const SpatialForce<double>& F : forces1.body_forces()) {
    EXPECT_TRUE(F.IsApprox(SpatialForce<double>::Zero(), kEpsilon));
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
