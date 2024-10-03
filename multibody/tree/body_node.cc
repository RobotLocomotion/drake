#include "drake/multibody/tree/body_node.h"

#include <sstream>
#include <string>

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
BodyNode<T>::~BodyNode() = default;

template <typename T>
void BodyNode<T>::CalcAcrossMobilizerBodyPoses_BaseToTip(
    const FrameBodyPoseCache<T>& frame_body_pose_cache,
    PositionKinematicsCache<T>* pc) const {
  // RigidBody for this node.
  const RigidBody<T>& body_B = body();

  // RigidBody for this node's parent, or the parent body P.
  const RigidBody<T>& body_P = parent_body();

  // Inboard/Outboard frames of this node's mobilizer.
  const Frame<T>& frame_F = inboard_frame();
  DRAKE_ASSERT(frame_F.body().index() == body_P.index());
  const Frame<T>& frame_M = outboard_frame();
  DRAKE_ASSERT(frame_M.body().index() == body_B.index());

  // Input (const):
  // - X_PF
  // - X_MB
  // - X_FM(q_B)
  // - X_WP(q(W:P)), where q(W:P) includes all positions in the kinematics
  //                 path from parent body P to the world W.
  const math::RigidTransform<T>& X_MB =
      frame_M.get_X_FB(frame_body_pose_cache);  // F==M
  const math::RigidTransform<T>& X_PF =
      frame_F.get_X_BF(frame_body_pose_cache);  // B==P
  const math::RigidTransform<T>& X_FM = pc->get_X_FM(mobod_index());
  const math::RigidTransform<T>& X_WP = pc->get_X_WB(inboard_mobod_index());

  // Output (updating a cache entry):
  // - X_PB(q_B)
  // - X_WB(q(W:P), q_B)
  math::RigidTransform<T>& X_PB = pc->get_mutable_X_PB(mobod_index());
  math::RigidTransform<T>& X_WB = pc->get_mutable_X_WB(mobod_index());
  Vector3<T>& p_PoBo_W = pc->get_mutable_p_PoBo_W(mobod_index());

  // TODO(amcastro-tri): Consider logic for the common case B = M.
  //  In that case X_FB = X_FM as suggested by setting X_MB = Identity.
  const math::RigidTransform<T> X_FB = X_FM * X_MB;

  X_PB = X_PF * X_FB;
  X_WB = X_WP * X_PB;

  // Compute shift vector p_PoBo_W from the parent origin to the body origin.
  const Vector3<T>& p_PoBo_P = X_PB.translation();
  const math::RotationMatrix<T>& R_WP = X_WP.rotation();
  p_PoBo_W = R_WP * p_PoBo_P;
}

// This method computes the total force Ftot_BBo on body B that must be
// applied for it to incur in a spatial acceleration A_WB. Mathematically:
//   Ftot_BBo = M_B_W * A_WB + b_Bo
// where b_Bo contains the velocity dependent gyroscopic terms (see Eq. 2.26,
// p. 27, in A. Jain's book). The above balance is performed at the origin
// Bo of the body frame B, which does not necessarily need to coincide with
// the body center of mass.
// Notes:
//   1. Ftot_BBo = b_Bo when A_WB = 0.
//   2. b_Bo = 0 when w_WB = 0.
//   3. b_Bo.translational() = 0 when Bo = Bcm (p_BoBcm = 0).
//      When Fb_Bo_W_cache is nullptr velocities are considered to be zero.
//      Therefore, from (2), the bias term is assumed to be zero and is not
//      computed.
template <typename T>
void BodyNode<T>::CalcBodySpatialForceGivenItsSpatialAcceleration(
    const std::vector<SpatialInertia<T>>& M_B_W_cache,
    const std::vector<SpatialForce<T>>* Fb_Bo_W_cache,
    const SpatialAcceleration<T>& A_WB, SpatialForce<T>* Ftot_BBo_W_ptr) const {
  DRAKE_DEMAND(Ftot_BBo_W_ptr != nullptr);

  // Output spatial force applied on mobilized body B, at Bo, measured in W.
  SpatialForce<T>& Ftot_BBo_W = *Ftot_BBo_W_ptr;

  // RigidBody for this node.
  const RigidBody<T>& body_B = body();

  // Mobilized body B spatial inertia about Bo expressed in world W.
  const SpatialInertia<T>& M_B_W = M_B_W_cache[body_B.mobod_index()];

  // Equations of motion for a rigid body written at a generic point Bo not
  // necessarily coincident with the body's center of mass. This corresponds
  // to Eq. 2.26 (p. 27) in A. Jain's book.
  Ftot_BBo_W = M_B_W * A_WB;

  // If velocities are zero, then Fb_Bo_W is zero and does not contribute.
  if (Fb_Bo_W_cache != nullptr) {
    // Dynamic bias for body B.
    const SpatialForce<T>& Fb_Bo_W = (*Fb_Bo_W_cache)[body_B.mobod_index()];
    Ftot_BBo_W += Fb_Bo_W;
  }
}

template <typename T>
void BodyNode<T>::CalcArticulatedBodyHingeInertiaMatrixFactorization(
    const MatrixUpTo6<T>& D_B,
    math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>* llt_D_B) const {
  DRAKE_THROW_UNLESS(llt_D_B != nullptr);

  // N.B. This code is tested in two ways:
  //  - body_node_test.cc: test the precision of the throwing condition based on
  //    the given hinge matrix, D_B. I.e., is this "correct"?
  //  - multibody_plant_forward_dynamics_test.cc: test integration with the
  //    larger multibody ecosystem (see the TestHingeInertiaMatrix fixture) to
  //    confirm that bad models/configurations get caught in this net. In
  //    other words, does this test serve its intended purpose?

  // Compute the LLT factorization of D_B as llt_D_B.
  // Note: Eigen benchmarks for various matrix factorizations are here:
  // https://eigen.tuxfamily.org/dox/group__DenseDecompositionBenchmark.html
  // As compared to factoring with LLT, the benchmark (as of October 2022)
  // reports other methods of factoring small 8x8 matrices are slower by:
  // LDLT 1.3x, PartialPivLU 1.5x, FullPivLU = 1.9x, HouseholderQR 3.5x,
  // CompleteOrthogonalDecomposition 4.3x, FullPivHouseholderQR 4.3x,
  // JacobiSVD 18.6x, BDCSVD 19.7x.
  *llt_D_B = math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>(
      MatrixUpTo6<T>(D_B.template selfadjointView<Eigen::Lower>()));

  // Ensure D_B (the articulated body hinge inertia matrix) is positive definite
  // (which means that all its eigenvalues are positive). If it is not positive
  // definite, the matrix may be non-physical, e.g., it has a zero moment of
  // inertia for an axis about which rotation is permitted.
  // Example: The 1x1 hinge matrix D_B = [3.3] is positive definite whereas
  // D_B = [0] or D_B = [-1E-22] are not positive definite.
  // If factorization fails, throw an assertion.
  // TODO(Mitiguy) Improve robustness of the test for a positive definite D_B.
  //  LLT factorization is not a reliable test of near-singular matrices.
  //  If LLT fails, we _know_ that D_B is not reliably positive-definite.
  //  However, if LLT succeeds, D_B may be near-singular. It would be ideal to
  //  have a fast check for near-singular cases. One mathematically rigorous
  //  technique that was tested was to do a 2ⁿᵈ LLT factorization on (D_B - ε I)
  //  where ε is a prudently chosen small number and I is the identity matrix.
  //  If the LLT of (D_B - ε I) is not positive definite and the LLT of D_B is
  //  positive definite, we _know_ D_B is near singular.
  if (llt_D_B->eigen_linear_solver().info() != Eigen::Success) {
    // Create a meaningful message that helps the user as much as possible.
    const Mobilizer<T>& mobilizer = *mobilizer_;
    const RigidBody<T>& inboard_body = mobilizer.inboard_body();
    const RigidBody<T>& outboard_body = mobilizer.outboard_body();
    const std::string& inboard_body_name = inboard_body.name();
    const std::string& outboard_body_name = outboard_body.name();
    std::stringstream message;
    message << "An internal mass matrix associated with the joint that "
               "connects body "
            << inboard_body_name << " to body " << outboard_body_name
            << " is not positive-definite.";
    if (mobilizer.can_rotate()) {
      message << " Since the joint allows rotation, ensure body "
              << outboard_body_name
              << " (combined with other outboard bodies) "
                 "has reasonable non-zero moments of inertia about the joint "
                 "rotation axes.";
    }
    if (mobilizer.can_translate()) {
      message << " Since the joint allows translation, ensure body "
              << outboard_body_name
              << " (combined with other outboard bodies) "
                 "has a reasonable non-zero mass.";
    }
    throw std::runtime_error(message.str());
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::BodyNode);
