#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RotationMatrixd;
using systems::Context;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// An articulated body inertia (ABI) test from Example 7.1, Pages 122 - 124 of
// [Featherstone 2008]. The test consists of a cylinder C inside of a box B. The
// cylinder is allowed to rotate about the x-axis and translate along the
// y-axis. We use an intermediate massless body M to emulate a 2 dof joint
// between the box and cylinder.
//
// This will test basic recursion and projection of articulated body inertias.
// One extension from the example from Featherstone is that this test uses a
// gimbal (ball rpy) joint to connect the box with the world (inertial) frame so
// further testing can be done.
//
// [Featherstone 2008] Featherstone, R., Rigid Body Dynamics Algorithms.
//                     Springer 2008

// This first test leaves everything aligned with World, making hand calculation
// of the ABIs easy.
GTEST_TEST(ArticulatedBodyInertiaAlgorithm, FeatherstoneExample) {
  // Create box (B).
  const double Lx = 0.4, Ly = 1.0, Lz = 1.0;
  const double mass_box = 2.0;
  const SpatialInertia<double> M_Bcm =
      SpatialInertia<double>::SolidBoxWithMass(mass_box, Lx, Ly, Lz);

  // Create cylinder (C) in box.
  // Note that the unit inertia of the cylinder is taken about the x-axis.
  const double r = 0.2, L = 0.3;
  const double mass_cylinder = 0.8;
  const SpatialInertia<double> M_Ccm =
      SpatialInertia<double>::SolidCylinderWithMass(mass_cylinder, r, L,
                                                    Vector3d::UnitX());

  // Create an empty model.
  auto tree_owned = std::make_unique<MultibodyTree<double>>();
  auto& tree = *tree_owned;

  // Add box body and gimbal (BallRpy) joint.
  const RigidBody<double>& box_link = tree.AddRigidBody("box", M_Bcm);
  tree.AddJoint<BallRpyJoint>("ball", tree.world_body(), {}, box_link, {});

  // Add a massless body that can rotate about x.
  const RigidBody<double>& massless_link =
      tree.AddRigidBody("massless", SpatialInertia<double>::Zero());
  tree.AddJoint<RevoluteJoint>("revolute", box_link, {}, massless_link, {},
                               Vector3d(1, 0, 0));

  // Add cylinder body and let it translate along y.
  const RigidBody<double>& cylinder_link = tree.AddRigidBody("cylinder", M_Ccm);
  tree.AddJoint<PrismaticJoint>("prismatic", massless_link, {}, cylinder_link,
                                {}, Vector3d(0, 1, 0));

  // Transfer tree to system and get a Context.
  MultibodyTreeSystem<double> system(std::move(tree_owned));
  auto context = system.CreateDefaultContext();

  // Update cache.
  PositionKinematicsCache<double> pc(tree.forest());
  tree.CalcPositionKinematicsCache(*context, &pc);

  // Compute articulated body cache.
  ArticulatedBodyInertiaCache<double> abc(tree.forest());
  tree.CalcArticulatedBodyInertiaCache(*context, &abc);

  // Get expected projected articulated body inertia of cylinder. Only the
  // y translation is projected out.
  Matrix6<double> M_cylinder_mat = M_Ccm.CopyToFullMatrix6();
  Matrix6<double> Pplus_C_W_expected_mat = Matrix6<double>::Zero();
  Pplus_C_W_expected_mat(0, 0) = M_cylinder_mat(0, 0);
  Pplus_C_W_expected_mat(1, 1) = M_cylinder_mat(1, 1);
  Pplus_C_W_expected_mat(2, 2) = M_cylinder_mat(2, 2);
  Pplus_C_W_expected_mat(3, 3) = mass_cylinder;
  Pplus_C_W_expected_mat(5, 5) = mass_cylinder;

  // Compare results.
  const ArticulatedBodyInertia<double>& Pplus_C_W_actual =
      abc.get_Pplus_PB_W(cylinder_link.mobod_index());
  EXPECT_TRUE(CompareMatrices(Pplus_C_W_expected_mat,
                              Pplus_C_W_actual.CopyToFullMatrix6(), kEpsilon));

  Matrix6<double> Pplus_M_W_expected_mat(Pplus_C_W_expected_mat);
  Pplus_M_W_expected_mat(0, 0) = 0.0;  // x inertia projected out

  const ArticulatedBodyInertia<double>& Pplus_M_W_actual =
      abc.get_Pplus_PB_W(massless_link.mobod_index());
  EXPECT_TRUE(CompareMatrices(Pplus_M_W_expected_mat,
                              Pplus_M_W_actual.CopyToFullMatrix6(), kEpsilon));

  // Get expected projected articulated body inertia of the articulated body
  // consisting of the box and cylinder.
  Matrix6<double> Pplus_B_W_expected_mat = Matrix6<double>::Zero();
  Pplus_B_W_expected_mat(3, 3) = mass_box + mass_cylinder;
  Pplus_B_W_expected_mat(4, 4) = mass_box;
  Pplus_B_W_expected_mat(5, 5) = mass_box + mass_cylinder;

  // Compare results.
  const ArticulatedBodyInertia<double>& P_B_W_actual =
      abc.get_Pplus_PB_W(box_link.mobod_index());
  EXPECT_TRUE(CompareMatrices(Pplus_B_W_expected_mat,
                              P_B_W_actual.CopyToFullMatrix6(), kEpsilon));
}

// This helper function projects a body's ABI P across its inboard mobilizer
// with hinge matrix H to get P⁺. (P⁺ is the ABI as seen from the
// inboard side of the inboard joint, that is, with the joint dofs projected
// out.) Both inputs should be expressed in World.
// See section 6.2 (pp. 100-105) in [Jain 2011] for derivation -- this function
// implements Eqn. 6.25. Noting that our H is transposed from Jain's, we have:
//    P⁺ = τbar ⋅ P                (6.25)
//    where τbar = I - G⋅Hᵀ        (6.16)
//             G = P⋅H⋅D⁻¹         (6.13)
//             D = Hᵀ⋅P⋅H          (6.13)
//
// [Jain 2011] Jain, A., Robot and Multibody Dynamics. Springer 2011
template <int dofs>
Matrix6<double> Project(const Matrix6<double>& P,
                        const Eigen::Matrix<double, 6, dofs>& H) {
  const auto D = H.transpose() * P * H;
  const auto G = P * H * D.inverse();
  const auto taubar = Matrix6<double>::Identity() - G * H.transpose();
  return taubar * P;
}

// A similar test to FeatherstoneExample. The main difference is that this
// test uses non-zero generalized positions and a non-square box.
GTEST_TEST(ArticulatedBodyInertiaAlgorithm, ModifiedFeatherstoneExample) {
  // Create box (B).
  const double Lx = 0.5, Ly = 1.2, Lz = 1.6;
  const double mass_box = 2.4;
  const SpatialInertia<double> M_Bcm =
      SpatialInertia<double>::SolidBoxWithMass(mass_box, Lx, Ly, Lz);

  // Create cylinder (C) in box.
  // Note that the unit inertia of the cylinder is taken about the x-axis.
  const double r = 0.3, L = 0.3;
  const double mass_cylinder = 0.6;
  const SpatialInertia<double> M_Ccm =
      SpatialInertia<double>::SolidCylinderWithMass(mass_cylinder, r, L,
                                                    Vector3d::UnitX());

  // Create an empty model.
  auto tree_owned = std::make_unique<MultibodyTree<double>>();
  auto& tree = *tree_owned;

  // Add box body and gimbal (BallRpy) joint.
  const RigidBody<double>& box_link = tree.AddRigidBody("box", M_Bcm);
  const auto& WB_joint =
      tree.AddJoint<BallRpyJoint>("ball", tree.world_body(), {}, box_link, {});

  // Add a massless body that can rotate about x.
  const RigidBody<double>& massless_link =
      tree.AddRigidBody("massless", SpatialInertia<double>::Zero());
  const auto& BM_joint = tree.AddJoint<RevoluteJoint>(
      "revolute", box_link, {}, massless_link, {}, Vector3d(1, 0, 0));

  // Add cylinder body and let it translate along y.
  const RigidBody<double>& cylinder_link = tree.AddRigidBody("cylinder", M_Ccm);
  const auto& MC_joint = tree.AddJoint<PrismaticJoint>(
      "prismatic", massless_link, {}, cylinder_link, {}, Vector3d(0, 1, 0));

  // Transfer tree to system and get a Context.
  MultibodyTreeSystem<double> system(std::move(tree_owned));
  auto context = system.CreateDefaultContext();

  // State of joint connecting the world and box.
  Vector3d q_WB;
  q_WB << 0.0, -M_PI_2, 0.0;
  WB_joint.set_angles(context.get(), q_WB);

  // State of the joints connecting the box and cylinder.
  BM_joint.set_angle(context.get(), M_PI_4);
  MC_joint.set_translation(context.get(), 0.2);

  // Update cache.
  PositionKinematicsCache<double> pc(tree.forest());
  tree.CalcPositionKinematicsCache(*context, &pc);

  // Compute articulated body cache.
  ArticulatedBodyInertiaCache<double> abc(tree.forest());
  tree.CalcArticulatedBodyInertiaCache(*context, &abc);

  // Find the rotation R_WC from World to the Cylinder frame.
  // We have R_WB = -π/2 about y, R_BM = π/4 about x, R_MC = identity.
  const RotationMatrixd R_WB = RotationMatrixd::MakeYRotation(-M_PI_2);
  const RotationMatrixd R_BM = RotationMatrixd::MakeXRotation(M_PI_4);
  const RotationMatrixd R_WM = R_WB * R_BM;
  const RotationMatrixd R_MC;
  const RotationMatrixd R_WC = R_WB * R_BM * R_MC;

  // To rotate spatial 6-vectors & matrices we use 2 diagonal rotation blocks.
  Matrix6<double> R6_WM = Matrix6<double>::Zero();
  R6_WM.block<3, 3>(0, 0) = R_WM.matrix();
  R6_WM.block<3, 3>(3, 3) = R_WM.matrix();
  const Matrix6<double> R6_WC = R6_WM;  // since R_MC=I

  const Matrix6<double> M_Ccm_mat = M_Ccm.CopyToFullMatrix6();
  Matrix6<double> M_Ccm_proj(M_Ccm_mat);  // This is ABI P_C.
  M_Ccm_proj(4, 4) = 0.0;  // Project out the y translation to get Pplus.
  Matrix6<double> Pplus_C_W_expected_mat =
      R6_WC * M_Ccm_proj * R6_WC.transpose();  // Re-express in World.

  // Compare results for the cylinder.
  const ArticulatedBodyInertia<double>& Pplus_C_W_expected =
      ArticulatedBodyInertia<double>(Pplus_C_W_expected_mat);
  const ArticulatedBodyInertia<double>& Pplus_C_W_actual =
      abc.get_Pplus_PB_W(cylinder_link.mobod_index());
  EXPECT_TRUE(CompareMatrices(Pplus_C_W_expected_mat,
                              Pplus_C_W_actual.CopyToFullMatrix6(), kEpsilon));

  // Get expected ABI of the massless body. This is just Pplus_C shifted by
  // the translation amount to get it back to the massless body's frame.
  Matrix6<double> P_M_W_expected_mat(
      Pplus_C_W_expected.Shift(R_WC * Vector3d(0.0, -0.2, 0.0))
          .CopyToFullMatrix6());

  // Verify that we get the right P_M.
  const ArticulatedBodyInertia<double>& P_M_W_actual =
      abc.get_P_B_W(massless_link.mobod_index());
  EXPECT_TRUE(CompareMatrices(P_M_W_expected_mat,
                              P_M_W_actual.CopyToFullMatrix6(), kEpsilon));

  // Need to project P_M to Pplus_M. We'll use the local Project() function
  // to remove the inertia about the massless body's x-axis revolute joint.
  Vector6d H_M_M;
  H_M_M << 1, 0, 0, 0, 0, 0;             // rotates about x
  const Vector6d H_M_W = R6_WM * H_M_M;  // re-express in W
  Matrix6<double> Pplus_M_W_expected_mat = Project(P_M_W_expected_mat, H_M_W);

  const ArticulatedBodyInertia<double> Pplus_M_W_expected(
      Pplus_M_W_expected_mat);
  const ArticulatedBodyInertia<double>& Pplus_M_W_actual =
      abc.get_Pplus_PB_W(massless_link.mobod_index());
  EXPECT_TRUE(CompareMatrices(Pplus_M_W_expected_mat,
                              Pplus_M_W_actual.CopyToFullMatrix6(), kEpsilon));

  // H for the world-box rpy joint is already in World so we don't need to
  // rotate it.
  Eigen::Matrix<double, 6, 3> H_B_W = Eigen::Matrix<double, 6, 3>::Zero();
  H_B_W.block<3, 3>(0, 0) = Matrix3<double>::Identity();

  // The articulated body inertia of the box is the sum of the rotated spatial
  // inertia of B and Pplus_M_W.
  const Matrix6<double> P_B_W_expected_mat =
      Pplus_M_W_expected_mat + M_Bcm.ReExpress(R_WB).CopyToFullMatrix6();

  // Get expected projected articulated body inertia of the articulated body
  // consisting of the box, massless body, and cylinder.
  Matrix6<double> Pplus_B_W_expected_mat = Project(P_B_W_expected_mat, H_B_W);

  // Compare results.
  const ArticulatedBodyInertia<double>& Pplus_B_W_actual =
      abc.get_Pplus_PB_W(box_link.mobod_index());
  EXPECT_TRUE(CompareMatrices(Pplus_B_W_expected_mat,
                              Pplus_B_W_actual.CopyToFullMatrix6(), kEpsilon));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
