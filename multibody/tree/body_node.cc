#include "drake/multibody/tree/body_node.h"

#include <iostream>
#include <string>

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>&
    BodyNode<T>::CalcHingeMatrixFactorization(
    const systems::Context<T>& context,
    const MatrixUpTo6<T>& D_B,
    ArticulatedBodyInertiaCache<T>* abic) const {
  DRAKE_THROW_UNLESS(abic != nullptr);

  // Compute the LLT factorization of D_B as llt_D_B.
  // Note: Eigen benchmarks for various matrix factorizations are here:
  // https://eigen.tuxfamily.org/dox/group__DenseDecompositionBenchmark.html
  // As compared to factoring with LLT, the benchmark (as of October 2022)
  // report other methods of factoring small 8x8 matrices are slower:
  // LDLT 1.3x, PartialPivLU 1.5x, FullPivLU = 1.9x, HouseholderQR 3.5x,
  // CompleteOrthogonalDecomposition 4.3x, FullPivHouseholderQR 4.3x,
  // JacobiSVD 18.6x, BDCSVD 19.7x.
  math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>& llt_D_B =
      get_mutable_llt_D_B(abic);
  llt_D_B = math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>(
      MatrixUpTo6<T>(D_B.template selfadjointView<Eigen::Lower>()));

  // As will become useful momentarily, get the mobilizer associated with this
  // BodyNode and check if can rotate, translate, or both.
  const Mobilizer<T>& mobilizer = get_mobilizer();
  const bool can_rotate = mobilizer.can_rotate();
  const bool can_translate = mobilizer.can_translate();

  // Ensure D_B (the articulated body hinge inertia matrix) is positive definite
  // enough (which means that all its eigenvalues are positive enough).
  // If there is a singularity (or near-singularity), the articulated body
  // hinge matrix may be non-physical, e.g., it has a zero (or near-zero)
  // moment of inertia along an axis along which rotation is permitted.
  // Example: The 1x1 hinge matrix D_B = [1.2] is positive definite whereas
  // D_B = [0] or D_B = [-1E-22] are not positive definite. We also want to
  // avoid near-singular hinge matrices such as D_B = [+ε] where ε is a "small"
  // positive number (the choice of ε is discussed below). We then ensure
  // the LLT factorization of (D_B - ε I) is still positive definite (i.e.,
  // all its eigenvalues are positive). The following proof establishes
  // the mathematical validity for this test. By definition, the eigenvalues
  // λ and eigenvectors v of the matrix A are determined by A v = λ v.
  //   (A - ε I) v = A v - ε I v  where I is the identity matrix.
  //               = λ v - ε v    since ε I = ε
  //               = (λ - ε) v    which shows (λ - ε) are the eigenvalues of
  // (A - ε I). Since A is symmetric, (A - ε I) is symmetric. Hence if
  // (λ - ε) > 0 (positive eigenvalues), then λ > ε which means all the
  // eigenvalues of A are not only positive, but greater than ε (which means
  // that A is not super-close to singular (for the ε chosen here).
  bool is_failed = llt_D_B.eigen_linear_solver().info() != Eigen::Success;

  // If the matrix has not already failed, check (D_B - ε I).
  if (!is_failed) {
    // For rotational motion, ε is chosen from a "small" moment of inertia which
    // scales as mass * length². In robotics and for rotation, we regard a small
    // mass as 1E-3 kg (1 gram) and a small length as 1E-3 m (1 mm).
    // Hence, εᵣₒₜ ≈ small moment of inertia ≈ mass * length² ≈ 1E-9.
    // For translational motion, ε is chosen by only considering a "small" mass
    // (not needing to scale with length²) and we use a more cautious criteria
    // by regarding a small mass as 1 milligram, so εₜᵣₐₙₛ ≈ small mass ≈ 1E-6.
    const double epsilon = can_rotate ? 1.0E-9 : 1.0E-6;
    const int nv = get_num_mobilizer_velocities();
    MatrixUpTo6<T> D_B_minus_epsilon(nv, nv);
    D_B_minus_epsilon = D_B;
    for (int i = 0; i < nv; ++i) D_B_minus_epsilon(i, i) -= epsilon;

    // Check if LLT factorization on llt_D_B_minus_epsilon fails.
    math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>> llt_D_B_minus_epsilon =
        math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>(MatrixUpTo6<T>(
            D_B_minus_epsilon.template selfadjointView<Eigen::Lower>()));
    is_failed =
        llt_D_B_minus_epsilon.eigen_linear_solver().info() != Eigen::Success;
  }

  // If factorization fails on original or shifted matrix, issue an error.
  if (is_failed) {
    // Create a meaningful message the helps the user as much as possible.
    const Body<T>& inboard_body = mobilizer.inboard_body();
    const Body<T>& outboard_body = mobilizer.outboard_body();
    // DRAKE_DEMAND(&(body()) == &inboard_body);
    // DRAKE_DEMAND(&(parent_body()) == &outboard_body);
    const std::string& inboard_body_name = inboard_body.name();
    const std::string& outboard_body_name = outboard_body.name();
    const T outboard_body_mass = outboard_body.get_mass(context);
    const RotationalInertia<T> I_BBo_B = outboard_body.
        CalcSpatialInertiaInBodyFrame(context).CalcRotationalInertia();
    std::stringstream message;
    message << "Encountered singular (or near-singular) articulated body hinge "
               "inertia matrix for BodyNode index " << topology_.index << ", "
               "which is associated with the joint that connects body "
            << inboard_body_name << " to body " << outboard_body_name << ". "
               "The articulated body hinge inertia matrix is [" << D_B << "]. ";
    if (can_rotate) {
      message << "Since the joint allows rotation, ensure that body "
              << outboard_body_name << " has reasonable non-zero moments of "
                 "inertia around joint rotation axes. The body's inertia "
                 "matrix around its body origin is " << I_BBo_B << ". ";
    }
    if (can_translate) {
      message << "Since the joint allows translation, ensure that body "
              << outboard_body_name << " has a reasonable non-zero mass. "
                 "Note: The body's mass is " << outboard_body_mass << ". ";
    }
    throw std::runtime_error(message.str());
  }
  return llt_D_B;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::BodyNode)
