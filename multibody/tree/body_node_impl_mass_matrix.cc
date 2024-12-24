/* clang-format off */
// NOLINTNEXTLINE(build/include)
#include "drake/multibody/tree/body_node_impl.h"
/* clang-format on */

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/curvilinear_mobilizer.h"
#include "drake/multibody/tree/planar_mobilizer.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/multibody/tree/rpy_ball_mobilizer.h"
#include "drake/multibody/tree/rpy_floating_mobilizer.h"
#include "drake/multibody/tree/screw_mobilizer.h"
#include "drake/multibody/tree/universal_mobilizer.h"
#include "drake/multibody/tree/weld_mobilizer.h"

/* This is a continuation of body_node_impl.cc, split in order to balance
compilation times. The many template instantiations take a long time to
compile. */

namespace drake {
namespace multibody {
namespace internal {

template <typename T, template <typename> class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::CalcMassMatrixContribution_TipToBase(
    const PositionKinematicsCache<T>& pc,
    const std::vector<SpatialInertia<T>>& Mc_B_W_cache,
    const std::vector<Vector6<T>>& H_PB_W_cache, EigenPtr<MatrixX<T>> M) const {
  // Welds only contribute to composite mass properties. Their effect on the
  // mass matrix gets included when we get to the composite's non-weld inboard
  // mobilizer.
  if constexpr (kNv != 0) {
    // This node's 6x6 composite body inertia.
    const SpatialInertia<T>& Mc_C_W = Mc_B_W_cache[mobod_index()];

    // Across-mobilizer 6 x cnv hinge matrix, from C's parent Cp to C.
    const auto H_CpC_W = get_H(H_PB_W_cache);  // 6 x kNv fixed size Map.

    // The composite body algorithm considers the system at rest, when
    // generalized velocities are zero.
    // Now if we consider this node's generalized accelerations as the matrix
    // vm_dot = Iₘ, the identity matrix in ℝᵐˣᵐ, the spatial acceleration A_WC
    // is in ℝ⁶ˣᵐ. That is, we are considering each case in which all
    // generalized accelerations are zero but the m-th generalized
    // acceleration for this node equals one.
    // This node's spatial acceleration can be written as:
    //   A_WC = Φᵀ(p_CpC) * A_WCp + Ac_WC + Ab_CpC_W + H_CpC_W * vm_dot
    // where A_WCp is the spatial acceleration of the parent node's body Cp,
    // Ac_WC include the centrifugal and Coriolis terms, and Ab_CpC_W is the
    // spatial acceleration bias of the hinge Jacobian matrix H_CpC_W.
    // Now, since all generalized accelerations but vm_dot are zero, then
    // A_WCp is zero.  Since the system is at rest, Ac_WC and Ab_CpC_W are
    // zero.
    // Therefore, for vm_dot = Iₘ, we have that A_WC = H_CpC_W.
    const auto& A_WC = H_CpC_W;  // 6 x cnv, fixed-size Map.

    // If we consider the closed system composed of the composite body held by
    // its mobilizer, the Newton-Euler equations state:
    //   Fm_CCo_W = Mc_C_W * A_WC + Fb_C_W
    // where Fm_CCo_W is the spatial force at this node's mobilizer.
    // Since the system is at rest, we have Fb_C_W = 0 and thus:
    const Eigen::Matrix<T, 6, kNv> Fm_CCo_W = Mc_C_W * A_WC;  // 6 x cnv.

    const int composite_start_in_v = mobilizer().velocity_start_in_v();

    // Diagonal block corresponding to current node (mobod_index).
    M->template block<kNv, kNv>(composite_start_in_v, composite_start_in_v) +=
        H_CpC_W.transpose() * Fm_CCo_W;

    // We recurse the tree inwards from C all the way to the root. We define
    // the frames:
    //  - B:  the frame for the current node, body_node.
    //  - Bc: B's child node frame, child_node.
    //  - P:  B's parent node frame.
    const BodyNode<T>* child_node = this;  // Child starts at frame C.
    const BodyNode<T>* body_node = this->parent_body_node();  // Inboard body.
    Eigen::Matrix<T, 6, kNv> Fm_CBo_W = Fm_CCo_W;             // 6 x cnv

    while (body_node->mobod_index() != world_mobod_index()) {
      const Vector3<T>& p_BoBc_W = pc.get_p_PoBo_W(child_node->mobod_index());
      // In place rigid shift of the spatial force in each column of
      // Fm_CBo_W, from Bc to Bo. Before this computation, Fm_CBo_W actually
      // stores Fm_CBc_W from the previous recursion. At the end of this
      // computation, Fm_CBo_W stores the spatial force on composite body C,
      // shifted to Bo, and expressed in the world W. That is, we are doing
      // Fm_CBo_W = Fm_CBc_W.Shift(p_BcB_W).

      // This is SpatialForce<T>::ShiftInPlace(&Fm_CBo_W, -p_BoBc_W) but
      // done with fixed sizes (no loop if kNv==1).
      // TODO(sherm1) Consider moving this to a templatized ShiftInPlace
      //  API in SpatialForce (and other spatial vector classes?).
      for (int col = 0; col < kNv; ++col) {
        // Ugly Eigen intermediate types; don't look!
        auto torque = Fm_CBo_W.template block<3, 1>(0, col);
        const auto force = Fm_CBo_W.template block<3, 1>(3, col);
        torque += p_BoBc_W.cross(force);  // + because we're negating p_BoBc
      }

      CalcMassMatrixOffDiagonalDispatcher<T, kNv>::Dispatch(
          *body_node, composite_start_in_v, H_PB_W_cache, Fm_CBo_W, M);

      child_node = body_node;                      // Update child node Bc.
      body_node = child_node->parent_body_node();  // Update node B.
    }
  }
}

// This is the inner loop of the CalcMassMatrix() algorithm. Rnv is the
// size of the outer-loop body node R's mobilizer, kNv is the size of the
// current body B's ConcreteMobilizer encountered on R's inboard sweep.
#define DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK(Rnv)                             \
  template <typename T, template <typename> class ConcreteMobilizer>           \
  void                                                                         \
      BodyNodeImpl<T, ConcreteMobilizer>::CalcMassMatrixOffDiagonalBlock##Rnv( \
          int R_start_in_v, const std::vector<Vector6<T>>& H_PB_W_cache,       \
          const Eigen::Matrix<T, 6, Rnv>& Fm_CBo_W, EigenPtr<MatrixX<T>> M)    \
          const {                                                              \
    if constexpr (kNv != 0) {                                                  \
      const auto H_PB_W = get_H(H_PB_W_cache); /* 6 x kNv fixed-size Map */    \
      const Eigen::Matrix<T, kNv, Rnv> HtFm = H_PB_W.transpose() * Fm_CBo_W;   \
      const int body_start_in_v = mobilizer().velocity_start_in_v();           \
      /* Update the appropriate block and its symmetric partner. */            \
      auto block = M->template block<kNv, Rnv>(body_start_in_v, R_start_in_v); \
      block += HtFm;                                                           \
      M->template block<Rnv, kNv>(R_start_in_v, body_start_in_v) =             \
          block.transpose();                                                   \
    }                                                                          \
  }

DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK(1)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK(2)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK(3)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK(4)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK(5)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK(6)

#undef DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK

// Macro used to explicitly instantiate implementations for every mobilizer.
#define EXPLICITLY_INSTANTIATE_IMPLS(T)                        \
  template class BodyNodeImpl<T, CurvilinearMobilizer>;        \
  template class BodyNodeImpl<T, PlanarMobilizer>;             \
  template class BodyNodeImpl<T, PrismaticMobilizer>;          \
  template class BodyNodeImpl<T, QuaternionFloatingMobilizer>; \
  template class BodyNodeImpl<T, RevoluteMobilizer>;           \
  template class BodyNodeImpl<T, RpyBallMobilizer>;            \
  template class BodyNodeImpl<T, RpyFloatingMobilizer>;        \
  template class BodyNodeImpl<T, ScrewMobilizer>;              \
  template class BodyNodeImpl<T, UniversalMobilizer>;          \
  template class BodyNodeImpl<T, WeldMobilizer>

// Explicitly instantiates on the supported scalar types.
// These should be kept in sync with the list in default_scalars.h.
EXPLICITLY_INSTANTIATE_IMPLS(double);
EXPLICITLY_INSTANTIATE_IMPLS(AutoDiffXd);
EXPLICITLY_INSTANTIATE_IMPLS(symbolic::Expression);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
