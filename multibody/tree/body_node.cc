#include "drake/multibody/tree/body_node.h"

#include <sstream>
#include <string>

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
BodyNode<T>::~BodyNode() = default;

// Notation used below
//
// `this` is mobilized body B, with body frame B. We're given B's spatial
// inertia M_BBo_W, taken about B's origin Bo and expressed in world frame W.
// Our goal is to calculate B's composite body inertia I_BBo_W, taken about Bo
// but including all the bodies outboard of B as though they were welded in
// their current configuration.
//
// Below we're going to treat B as the "parent" and work with its immediately
// outboard bodies as "children" C. Because this is an inward sweep, we already
// know I_CCo_W, each child's composite body inertia, taken about the child's
// body frame origin Co, and expressed in W. We need to shift those to Bo before
// summing them up.
template <typename T>
void BodyNode<T>::CalcCompositeBodyInertiaInWorld_TipToBase(
    const PositionKinematicsCache<T>& pc,  // for p_BoCo_W
    const std::vector<SpatialInertia<T>>& M_BBo_W_all,
    std::vector<SpatialInertia<T>>* I_BBo_W_all) const {
  const MobodIndex index = mobod_index();
  DRAKE_ASSERT(index != world_mobod_index());
  DRAKE_ASSERT(I_BBo_W_all != nullptr);

  // This mobod's spatial inertia (given).
  const SpatialInertia<T>& M_BBo_W = M_BBo_W_all[index];
  // This mobod's composite body inertia (to be calculated).
  SpatialInertia<T>& I_BBo_W = (*I_BBo_W_all)[index];

  I_BBo_W = M_BBo_W;  // Start with B's spatial inertia.
  // Then add in each child's composite body inertia, 76 flops per.
  for (const BodyNode<T>* child : child_nodes()) {
    const MobodIndex child_node_index = child->mobod_index();
    const SpatialInertia<T>& I_CCo_W = (*I_BBo_W_all)[child_node_index];
    // Body B is the parent "P" here and C is the body "B" (in pc).
    const Vector3<T>& p_BoCo_W = pc.get_p_PoBo_W(child_node_index);
    I_BBo_W += I_CCo_W.Shift(-p_BoCo_W);  // i.e., by p_CoBo, 76 flops
  }
}

// Notation used below
//
// `this` is mobilized body B. It's mobilizer's inboard frame (the "M" frame) is
// denoted Mb. We're given B's spatial inertia M_BMb_Mb (taken about Mb's origin
// Mbo and expressed in Mb). Our goal is to calculate B's composite body inertia
// I_BMb_Mb, taken about Mbo but including all the bodies outboard of B as
// though they were welded in their current configuration.
//
// Below we're going to treat B as the "parent" and work with its immediately
// outboard bodies as "children" C. Because this is an inward sweep, we
// already know I_CMc_Mc, each child's composite body inertia, taken about
// the child's inboard frame Mc's origin Mco, and expressed in Mc. We need to
// shift those to Mbo and re-express in Mb before summing them up.
template <typename T>
void BodyNode<T>::CalcCompositeBodyInertiaInM_TipToBase(
    const FrameBodyPoseCache<T>& frame_body_pose_cache,  // for M_BMb_Mb
    const PositionKinematicsCacheInM<T>& pcm,            // for X_MbMc(q)
    std::vector<SpatialInertia<T>>* I_BMb_Mb_all) const {
  const MobodIndex index = mobod_index();
  DRAKE_ASSERT(index != world_mobod_index());
  DRAKE_ASSERT(I_BMb_Mb_all != nullptr);

  // This mobod's spatial inertia (given).
  const SpatialInertia<T>& M_BMb_Mb = frame_body_pose_cache.get_M_BMo_M(index);
  // This mobod's composite body inertia (to be calculated).
  SpatialInertia<T>& I_BMb_Mb = (*I_BMb_Mb_all)[index];

  I_BMb_Mb = M_BMb_Mb;  // Start with B's spatial inertia.
  // Then add in each child's composite body inertia, 148 flops per.
  for (const BodyNode<T>* child : child_nodes()) {
    const MobodIndex child_node_index = child->mobod_index();
    const SpatialInertia<T>& I_CMc_Mc =  // Child's composite body inertia.
        (*I_BMb_Mb_all)[child_node_index];
    // Body B is the parent here.
    const math::RigidTransform<T>& X_MbMc = pcm.get_X_MpM(child_node_index);
    // Shift the child's composite body inertia to B's Mb frame origin Mbo, but
    // still expressed in Mc (so Mx==Mc here).
    const Vector3<T> p_McMb_Mb = -X_MbMc.translation();  // 3 flops
    SpatialInertia<T> I_CMb_Mx = I_CMc_Mc.Shift(p_McMb_Mb);  // 37 flops
    I_CMb_Mx.ReExpressInPlace(X_MbMc.rotation());  // Now Mx==Mb, 72 flops
    I_BMb_Mb += I_CMb_Mx;  // 36 flops
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
