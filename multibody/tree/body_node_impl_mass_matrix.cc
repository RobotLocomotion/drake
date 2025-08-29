/* clang-format off */
// NOLINTNEXTLINE(build/include)
#include "drake/multibody/tree/body_node_impl.h"
/* clang-format on */

#include <vector>

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

template <typename T, class ConcreteMobilizer>
void BodyNodeImpl<T, ConcreteMobilizer>::
    CalcMassMatrixContributionViaWorld_TipToBase(
        const PositionKinematicsCache<T>& pc,
        const std::vector<SpatialInertia<T>>& K_BBo_W_cache,
        const std::vector<Vector6<T>>& H_PB_W_cache,
        EigenPtr<MatrixX<T>> M) const {
  // Welds only contribute to composite mass properties. Their effect on the
  // mass matrix gets included when we get to the composite's non-weld inboard
  // mobilizer.
  if constexpr (kNv != 0) {
    // This node's 6x6 composite body inertia.
    const SpatialInertia<T>& K_BBo_W = K_BBo_W_cache[mobod_index()];

    // Across-mobilizer 6 x kNv hinge matrix, from C's parent Cp to C.
    const auto H_PB_W = get_H(H_PB_W_cache);  // 6 x kNv fixed size Map.

    // The composite body algorithm considers the system at rest, when
    // generalized velocities are zero. Here m=kNv, this body's mobilizer's
    // number of dofs.
    // Now if we consider this node's generalized accelerations as the matrix
    // vm_dot = Iₘ, the identity matrix in ℝᵐˣᵐ, the spatial acceleration A_WB
    // is in ℝ⁶ˣᵐ. That is, we are considering each case in which all
    // generalized accelerations are zero but the m-th generalized
    // acceleration for this node equals one.
    // This node's spatial acceleration can be written as:
    //   A_WB = Φᵀ(p_PB) * A_WP + A_WB + Ab_PB_W + H_PB_W * vm_dot
    // where A_WP is the spatial acceleration of the parent node's body P. Φᵀ is
    // the shift operator from Abhi Jain's book, p_PB is the position vector
    // from Po to Bo expressed in P. A_WB includes the centrifugal and Coriolis
    // terms, and Ab_PC_W is the spatial acceleration bias of the hinge Jacobian
    // matrix H_PB_W. Now, since all generalized accelerations but vm_dot are
    // zero, then A_WP is zero.  Since the system is at rest, A_WB and Ab_PB_W
    // are also zero. Therefore, for vm_dot = Iₘ, we have that A_WB = H_PB_W.
    const auto& A_WB = H_PB_W;  // 6 x kNv, fixed-size Map.

    // If we consider the closed system composed of the composite body held by
    // its mobilizer, the Newton-Euler equations state:
    //   Fm_BBo_W = K_BBo_W * A_WB + Fb_BBo_W
    // where Fm_BBo_W is the spatial force at this node's mobilizer.
    // Since the system is at rest, we have Fb_BBo_W = 0 and thus:
    const Eigen::Matrix<T, 6, kNv> Fm_BBo_W = K_BBo_W * A_WB;

    const int B_start_in_v = mobilizer().velocity_start_in_v();

    // Diagonal block corresponding to current node (mobod_index).
    M->template block<kNv, kNv>(B_start_in_v, B_start_in_v) =
        H_PB_W.transpose() * Fm_BBo_W;  // kNv * kNv * 11 flops

    // We recurse the tree inwards from B all the way to the root. We define
    // the frames:
    //  - C: child composite body, initially B
    //  - P: parent composite body, initially B's inboard body
    const BodyNode<T>* child_node = this;
    const BodyNode<T>* parent_node = this->parent_body_node();
    Eigen::Matrix<T, 6, kNv> Fm_CPo_W = Fm_BBo_W;  // Initially Fm_CCo_W.

    while (parent_node->mobod_index() != world_mobod_index()) {
      const Vector3<T>& p_PC_W = pc.get_p_PoBo_W(child_node->mobod_index());

      // In place rigid shift of the spatial force in each column of Fm_CCo_W,
      // from Co to Po. Before this computation, Fm_CPo_W actually stores
      // Fm_CCo_W from the previous recursion. At the end of this computation,
      // Fm_CPo_W stores the spatial force on child composite body C, shifted to
      // parent origin Po, and expressed in the world W. That is, we are doing
      // Fm_CPo_W = Fm_CCo_W.Shift(p_CoPo_W).

      // This is SpatialForce<T>::ShiftInPlace(&Fm_CCo_W, -p_PoCo_W) but
      // done with fixed sizes (no loop if kNv==1). 12 * kNv flops
      for (int col = 0; col < kNv; ++col) {
        // Ugly Eigen intermediate types; don't look!
        auto torque = Fm_CPo_W.template block<3, 1>(0, col);
        const auto force = Fm_CPo_W.template block<3, 1>(3, col);
        torque += p_PC_W.cross(force);  // + because we're negating p_PC
      }

      CalcMassMatrixOffDiagonalViaWorldDispatcher<T, kNv>::Dispatch(
          *parent_node, B_start_in_v, H_PB_W_cache, Fm_CPo_W, M);

      child_node = parent_node;                      // Update child node C.
      parent_node = child_node->parent_body_node();  // Update parent node P.
    }
  }
}

// This is the inner loop of the CalcMassMatrix() algorithm. Bnv is the
// size of the outer-loop body node B's mobilizer, kNv is the size of the
// current inboard body P's ConcreteMobilizer encountered on B's inboard sweep.
// Cost is Bnv*kNv*11 flops.
#define DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(Bnv)                 \
  template <typename T, class ConcreteMobilizer>                             \
  void BodyNodeImpl<T, ConcreteMobilizer>::                                  \
      CalcMassMatrixOffDiagonalBlockViaWorld##Bnv(                           \
          int B_start_in_v, const std::vector<Vector6<T>>& H_PB_W_cache,     \
          const Eigen::Matrix<T, 6, Bnv>& Fm_CPo_W, EigenPtr<MatrixX<T>> M)  \
          const {                                                            \
    if constexpr (kNv != 0) {                                                \
      const auto H_PC_W = get_H(H_PB_W_cache); /* 6 x kNv fixed-size Map */  \
      const Eigen::Matrix<T, kNv, Bnv> HtFm = H_PC_W.transpose() * Fm_CPo_W; \
      const int P_start_in_v = mobilizer().velocity_start_in_v();            \
      /* Update the appropriate block and its symmetric partner. */          \
      auto block = M->template block<kNv, Bnv>(P_start_in_v, B_start_in_v);  \
      block = HtFm;                                                          \
      M->template block<Bnv, kNv>(B_start_in_v, P_start_in_v) =              \
          block.transpose();                                                 \
    }                                                                        \
  }

DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(1)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(2)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(3)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(4)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(5)
DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD(6)

#undef DEFINE_MASS_MATRIX_OFF_DIAGONAL_BLOCK_VIA_WORLD

// Macro used to explicitly instantiate implementations for every mobilizer.
#define EXPLICITLY_INSTANTIATE_IMPLS(T)                           \
  template class BodyNodeImpl<T, CurvilinearMobilizer<T>>;        \
  template class BodyNodeImpl<T, PlanarMobilizer<T>>;             \
  template class BodyNodeImpl<T, PrismaticMobilizerAxial<T, 0>>;  \
  template class BodyNodeImpl<T, PrismaticMobilizerAxial<T, 1>>;  \
  template class BodyNodeImpl<T, PrismaticMobilizerAxial<T, 2>>;  \
  template class BodyNodeImpl<T, QuaternionFloatingMobilizer<T>>; \
  template class BodyNodeImpl<T, RevoluteMobilizerAxial<T, 0>>;   \
  template class BodyNodeImpl<T, RevoluteMobilizerAxial<T, 1>>;   \
  template class BodyNodeImpl<T, RevoluteMobilizerAxial<T, 2>>;   \
  template class BodyNodeImpl<T, RpyBallMobilizer<T>>;            \
  template class BodyNodeImpl<T, RpyFloatingMobilizer<T>>;        \
  template class BodyNodeImpl<T, ScrewMobilizer<T>>;              \
  template class BodyNodeImpl<T, UniversalMobilizer<T>>;          \
  template class BodyNodeImpl<T, WeldMobilizer<T>>

// Explicitly instantiates on the supported scalar types.
// These should be kept in sync with the list in default_scalars.h.
EXPLICITLY_INSTANTIATE_IMPLS(double);
EXPLICITLY_INSTANTIATE_IMPLS(AutoDiffXd);
EXPLICITLY_INSTANTIATE_IMPLS(symbolic::Expression);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
