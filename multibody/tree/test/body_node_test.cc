#include "drake/multibody/tree/body_node.h"

#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/planar_mobilizer.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {

// Friend access into MultibodyElement, so DummyBody can set its node index.
class MultibodyElementTester {
 public:
  MultibodyElementTester() = delete;
  static void set_index(RigidBody<double>* element, BodyIndex index) {
    // MultibodyTree parameter is null; don't call anything that depends on the
    // tree.
    element->set_parent_tree(nullptr, index);
  }
};

namespace internal {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using math::RotationMatrix;
using systems::Context;

// Friend access into BodyNode.
class BodyNodeTester {
 public:
  BodyNodeTester() = delete;

  static void CallLltFactorization(const BodyNode<double>& node,
                                   const MatrixUpTo6<double>& D_B) {
    math::LinearSolver<Eigen::LLT, MatrixUpTo6<double>> llt_D_B;
    node.CalcArticulatedBodyHingeInertiaMatrixFactorization(D_B, &llt_D_B);
  }
};

namespace {

// TODO(SeanCurtis-TRI): Consider moving class DummyBody into common test
//  utilities so other tests can create mocked bodies. Possibly templatize it.

// Minimal definition of a body that we can use to construct a BodyNode.
class DummyBody : public RigidBody<double> {
 public:
  DummyBody(std::string name, BodyIndex index)
      : RigidBody<double>(std::move(name), ModelInstanceIndex(0),
                          SpatialInertia<double>::NaN()) {
    // We need a body index for the body node test to be happy.
    MultibodyElementTester::set_index(this, index);
  }
};

// Non-abstract definition of a BodyNode.
class DummyBodyNode : public BodyNode<double> {
 public:
  using T = double;

  using BodyNode::BodyNode;

  void CalcPositionKinematicsCache_BaseToTip(
      const FrameBodyPoseCache<T>&, const T*,
      PositionKinematicsCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcAcrossNodeJacobianWrtVExpressedInWorld(
      const FrameBodyPoseCache<T>&, const T*, const PositionKinematicsCache<T>&,
      std::vector<Vector6<T>>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcVelocityKinematicsCache_BaseToTip(
      const T*, const PositionKinematicsCache<T>&,
      const std::vector<Vector6<T>>&, const T*,
      VelocityKinematicsCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcMassMatrixContributionViaWorld_TipToBase(
      const PositionKinematicsCache<T>&, const std::vector<SpatialInertia<T>>&,
      const std::vector<Vector6<T>>&, EigenPtr<MatrixX<T>>) const final {
    DRAKE_UNREACHABLE();
  }

#define DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(Bnv)                      \
  void CalcMassMatrixOffDiagonalBlockViaWorld##Bnv(                         \
      int, const std::vector<Vector6<T>>&, const Eigen::Matrix<T, 6, Bnv>&, \
      EigenPtr<MatrixX<T>>) const final {                                   \
    DRAKE_UNREACHABLE();                                                    \
  }

  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(1)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(2)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(3)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(4)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(5)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(6)

#undef DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD

  void CalcSpatialAcceleration_BaseToTip(
      const FrameBodyPoseCache<T>&, const T*, const PositionKinematicsCache<T>&,
      const T*, const VelocityKinematicsCache<T>*, const T*,
      std::vector<SpatialAcceleration<T>>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcInverseDynamics_TipToBase(const FrameBodyPoseCache<T>&, const T*,
                                     const PositionKinematicsCache<T>&,
                                     const std::vector<SpatialInertia<T>>&,
                                     const std::vector<SpatialForce<T>>*,
                                     const std::vector<SpatialAcceleration<T>>&,
                                     const std::vector<SpatialForce<T>>&,
                                     const Eigen::Ref<const VectorX<T>>&,
                                     std::vector<SpatialForce<T>>*,
                                     EigenPtr<VectorX<T>>) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcArticulatedBodyInertiaCache_TipToBase(
      const systems::Context<T>&, const PositionKinematicsCache<T>&,
      const Eigen::Ref<const MatrixUpTo6<T>>&, const SpatialInertia<T>&,
      const VectorX<T>&, ArticulatedBodyInertiaCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcArticulatedBodyForceCache_TipToBase(
      const systems::Context<T>&, const PositionKinematicsCache<T>&,
      const VelocityKinematicsCache<T>*, const SpatialForce<T>&,
      const ArticulatedBodyInertiaCache<T>&, const SpatialForce<T>&,
      const SpatialForce<T>&, const Eigen::Ref<const VectorX<T>>&,
      const Eigen::Ref<const MatrixUpTo6<T>>&,
      ArticulatedBodyForceCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcArticulatedBodyAccelerations_BaseToTip(
      const systems::Context<T>&, const PositionKinematicsCache<T>&,
      const ArticulatedBodyInertiaCache<T>&,
      const ArticulatedBodyForceCache<T>&,
      const Eigen::Ref<const MatrixUpTo6<T>>&, const SpatialAcceleration<T>&,
      AccelerationKinematicsCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcCompositeBodyInertiaInWorld_TipToBase(
      const PositionKinematicsCache<T>&, const std::vector<SpatialInertia<T>>&,
      std::vector<SpatialInertia<T>>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcSpatialAccelerationBias(
      const FrameBodyPoseCache<T>&, const T*, const PositionKinematicsCache<T>&,
      const T*, const VelocityKinematicsCache<T>&,
      std::vector<SpatialAcceleration<T>>*) const final {
    DRAKE_UNREACHABLE();
  }
};

// This test validates the exception message thrown in
// BodyNode::CalcArticulatedBodyHingeInertiaMatrixFactorization(). There are two
// aspects of the message that are not simply *literal*:
//
//    - Inclusion of body names in their correct roles.
//    - Customization for whether the joint allows translational and/or
//      rotational motion.
//
// To test for both, we assess the *full* error message. This should be the only
// time the full message appears in a test. In all other test code, a small
// indication should be sufficient.
GTEST_TEST(BodyNodeTest, FactorArticulatedBodyHingeInertiaMatrixErrorMessages) {
  // Construct enough of a node so we can invoke the dut with known body names.
  const DummyBody parent("parent", world_index());
  const DummyBody child("child", BodyIndex(1));
  const DummyBodyNode parent_node(nullptr, &parent, nullptr);

  // A 1x1 articulated body hinge inertia matrix.
  MatrixUpTo6<double> one_by_one(1, 1);
  one_by_one(0, 0) = -1;  // This should *definitely* fail.

  const SpanningForest::Mobod dummy_mobod(MobodIndex(0), LinkOrdinal(0));
  {
    // Rotation only.
    const RevoluteMobilizerAxial<double, 2> mobilizer(
        dummy_mobod, parent.body_frame(), child.body_frame());
    const DummyBodyNode body_node(&parent_node, &child, &mobilizer);
    DRAKE_EXPECT_THROWS_MESSAGE(
        BodyNodeTester::CallLltFactorization(body_node, one_by_one),
        "An internal mass matrix associated with the joint that connects body "
        "parent to body child is not positive-definite. Since the joint allows "
        "rotation, ensure body child \\(combined with other outboard bodies\\) "
        "has reasonable non-zero moments of inertia about the joint rotation "
        "axes.");
  }

  {
    // Translation only.
    const PrismaticMobilizerAxial<double, 2> mobilizer(
        dummy_mobod, parent.body_frame(), child.body_frame());
    const DummyBodyNode body_node(&parent_node, &child, &mobilizer);
    DRAKE_EXPECT_THROWS_MESSAGE(
        BodyNodeTester::CallLltFactorization(body_node, one_by_one),
        "An internal mass matrix associated with the joint that connects body "
        "parent to body child is not positive-definite. Since the joint allows "
        "translation, ensure body child \\(combined with other outboard "
        "bodies\\) has a reasonable non-zero mass.");
  }

  {
    // Rotation and translation.
    // N.B. using a 1x1 matrix with a PlanarMobilizer is physically nonsensical,
    // but acceptable for the test; the actual dimension of the mobilizers state
    // space is irrelevant.
    const PlanarMobilizer<double> mobilizer(dummy_mobod, parent.body_frame(),
                                            child.body_frame());
    const DummyBodyNode body_node(&parent_node, &child, &mobilizer);
    // In this case, we don't need to examine the full exception message since
    // the message is a concatenation of the text in the previous messages. We
    // look for evidence of concatenation with "rotation" and "translation".
    DRAKE_EXPECT_THROWS_MESSAGE(
        BodyNodeTester::CallLltFactorization(body_node, one_by_one),
        "An internal mass matrix associated with the joint that connects body "
        "parent to body child is not positive-definite. Since the joint allows "
        "rotation.+ translation.+");
  }
}

// This test seeks to characterize the conditions under which
// BodyNode<T>::CalcArticulatedBodyHingeInertiaMatrixFactorization()
// throws. We use hand-crafted matrices, to probe the boundary of valid and
// invalid matrices. This allows us to make assertions about the quality of the
// exception condition criterion -- we rely on integration tests in
// multibody_plant_forward_dynamics_test.cc to assess if it plays well
// with the matrices produced by the articulated body algorithm.
GTEST_TEST(BodyNodeTest, FactorHingeMatrixThrows) {
  using Tester = BodyNodeTester;
  using Matrix = MatrixUpTo6<double>;

  const DummyBody world("world", world_index());
  const DummyBody body("child", BodyIndex(1));
  const SpanningForest::Mobod dummy_mobod(MobodIndex(0), LinkOrdinal(0));
  const RevoluteMobilizerAxial<double, 2> mobilizer(
      dummy_mobod, world.body_frame(), body.body_frame());
  const DummyBodyNode world_node(nullptr, &world, nullptr);
  const DummyBodyNode body_node(&world_node, &body, &mobilizer);

  // 1x1 hinge matrix.
  Matrix one_by_one(1, 1);

  // Valid 1x1 hinge matrix values - none of these should throw.
  // For a 1x1 hinge matrix, any positive value is valid.
  for (const double value : {1.0e-120, 1.5, 1.0e120}) {
    one_by_one(0, 0) = value;
    EXPECT_NO_THROW(Tester::CallLltFactorization(body_node, one_by_one))
        << "For expected good value: " << value;
  }

  // Values that should throw. Any non-positive value should throw.
  for (const double value : {0.0, -1.0e-15, -1.5, -1.0e10}) {
    one_by_one(0, 0) = value;
    EXPECT_THROW(Tester::CallLltFactorization(body_node, one_by_one),
                 std::exception)
        << "For expected bad value: " << value;
  }

  // We also test a full 6x6 matrix with the expectation that if 1x1 and 6x6
  // behave, so will 2x2, etc.
  //
  // The test uses matrices of the form:
  //
  //  |  K   0  |,  for an arbitrary 3x3 matrix K, and a diagonal 3x3 matrix
  //  |  0  m I |   with the value m.
  //
  // For *valid* 6x6 matrices, K will be positive definite and should not throw.
  // For *invalid* matrices, it can be anything.

  Matrix six_by_six(6, 6);
  six_by_six = MatrixXd::Zero(6, 6);
  const double m = 1.5;
  six_by_six(3, 3) = m;
  six_by_six(4, 4) = m;
  six_by_six(5, 5) = m;

  // We'll define K as R⋅D⋅Rᵀ; the choice of the values on the diagonal matrix D
  // will determine whether K is positive definite or not.
  const RotationMatrix<double> R = RotationMatrix<double>(
      AngleAxisd(M_PI / 7, Vector3d{1, 2, 3}.normalized()));

  auto make_K = [&R](const Vector3d& K_eigen_values) {
    const Eigen::DiagonalMatrix<double, 3> K_principal(K_eigen_values);
    const Matrix3d K = R.matrix() * K_principal * R.matrix().transpose();
    return K;
  };

  // Valid 6x6 hinge matrix values - none of these should throw.
  for (const auto& K_eigen_values :
       {// A very small matrix.
        Vector3d{1.1e-12, 2e-12, 3e-12},
        // A mid-sized matrix.
        Vector3d{1.1, 2, 3},
        // A matrix with a not-too-bad condition number still passes.
        // (Contrast this with the failiing test below.)
        // TODO(Mitiguy) Improve robusness of this test (see TODO below).
        Vector3d{Vector3d{1.1, 2e12, 3e17}}}) {
    six_by_six.block<3, 3>(0, 0) = make_K(K_eigen_values);
    EXPECT_NO_THROW(Tester::CallLltFactorization(body_node, six_by_six))
        << fmt::format("For expected bad eigenvalues: {}",
                       fmt_eigen(K_eigen_values.transpose()));
  }

  // N.B. There are more ways the matrix could cause a throw. This approach
  // guarantees symmetric matrices, but not necessarily positive definite.
  for (const auto& K_eigen_values :
       {// Negative eigenvalue.
        Vector3d{1.1e-12, -2e-12, 3e-12},
        // A near-zero matrix (not really singular because of numerical
        // round-off when re-expressing K_principal).
        Vector3d{0, 1e-9, 3},
        // A matrix with a too-bad condition number; it throws.
        // TODO(Mitiguy) Improve robusness of these tests. These last two sets
        //  of values are particularly brittle as we are near the edge of a
        //  numerical "cliff". A single call to LLT is not a reliable way to
        //  check if a matrix is near singular. Consider using an tolerance to
        //  discern that we are sufficiently far from the cliff's edge.
        //  Note: The brittle tests herein were chosen because they worked --
        //  based on the computer hardware available in CI testing.
        Vector3d{Vector3d{1.1, 2e12, 3e18}}}) {
    six_by_six.block<3, 3>(0, 0) = make_K(K_eigen_values);
    EXPECT_THROW(Tester::CallLltFactorization(body_node, six_by_six),
                 std::exception)
        << fmt::format("For expected bad eigenvalues: {}",
                       fmt_eigen(K_eigen_values.transpose()));
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
