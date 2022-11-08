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
  // reports other methods of factoring small 8x8 matrices are slower by:
  // LDLT 1.3x, PartialPivLU 1.5x, FullPivLU = 1.9x, HouseholderQR 3.5x,
  // CompleteOrthogonalDecomposition 4.3x, FullPivHouseholderQR 4.3x,
  // JacobiSVD 18.6x, BDCSVD 19.7x.
  math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>& llt_D_B =
      get_mutable_llt_D_B(abic);
  llt_D_B = math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>(
      MatrixUpTo6<T>(D_B.template selfadjointView<Eigen::Lower>()));

  // Ensure D_B (the articulated body hinge inertia matrix) is positive definite
  // (which means that all its eigenvalues are positive). If it is not postive
  // definite, the articulated body hinge matrix may be non-physical, e.g., it
  // has a zero moment of inertia for an axis about which rotation is permitted.
  // Example: The 1x1 hinge matrix D_B = [3.3] is positive definite whereas
  // D_B = [0] or D_B = [-1E-22] are not positive definite.
  bool is_failure_with_epsilon = false;
  const bool is_failure_absolute =
      llt_D_B.eigen_linear_solver().info() != Eigen::Success;

  // As momentarily useful, get the mobilizer associated with this BodyNode .
  const Mobilizer<T>& mobilizer = get_mobilizer();

  // If no failure yet, check if the articulated body hinge matrix D_B is near
  // singular. If the matrix is near-singular, the articulated body hinge matrix
  // may be non-physical, e.g., it has a near-zero moment of inertia for an axis
  // about which rotation is permitted.
  // Example: We want to  avoid near-singular hinge matrices such as D_B = [+ε]
  // where ε is a "small" positive number (the choice of ε is discussed below).
  // To that end, we ensure the LLT factorization of (D_B - ε I) is positive
  // definite (i.e., all its eigenvalues are positive). What follows is a proof
  // of the mathematical validity of this test. By definition, the eigenvalues
  // λ and eigenvectors v of the matrix A are determined by A v = λ v.
  //   (A - ε I) v = A v - ε I v  where I is the identity matrix.
  //               = λ v - ε v    since ε I = ε
  //               = (λ - ε) v    which shows (λ - ε) are the eigenvalues of
  // (A - ε I). Since A is symmetric, (A - ε I) is symmetric. Hence if
  // (λ - ε) > 0 (positive eigenvalues), then λ > ε which means all the
  // eigenvalues of A are not only positive, but greater than ε (which means
  // that A is not super-close to singular (for the ε chosen here).
  if (is_failure_absolute == false) {
    // For rotational motion, ε is chosen from a "small" moment of inertia which
    // scales as mass * length². In robotics and for rotation, we regard a small
    // mass as 1E-3 kg (1 gram) and a small length as 1E-3 m (1 mm).
    // Hence, εᵣₒₜ ≈ small moment of inertia ≈ mass * length² ≈ 1E-9.
    // For translational motion, ε is chosen by only considering a "small" mass
    // (not needing to scale with length²) and we use a more cautious criteria
    // by regarding a small mass as 1 milligram, so εₜᵣₐₙₛ ≈ small mass ≈ 1E-6.
    const double epsilon = mobilizer.can_rotate() ? 1.0E-9 : 1.0E-6;
    const int nv = get_num_mobilizer_velocities();
    MatrixUpTo6<T> D_B_minus_epsilon(nv, nv);
    D_B_minus_epsilon = D_B;
    for (int i = 0; i < nv; ++i) D_B_minus_epsilon(i, i) -= epsilon;

    // Check if LLT factorization on llt_D_B_minus_epsilon fails.
    math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>> llt_D_B_minus_epsilon =
        math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>(MatrixUpTo6<T>(
            D_B_minus_epsilon.template selfadjointView<Eigen::Lower>()));
    is_failure_with_epsilon =
        llt_D_B_minus_epsilon.eigen_linear_solver().info() != Eigen::Success;
  }

  // If factorization fails on original or shifted matrix, issue an error.
  if (is_failure_absolute || is_failure_with_epsilon) {
    // Create a meaningful message the helps the user as much as possible.
    const Body<T>& inboard_body = mobilizer.inboard_body();
    const Body<T>& outboard_body = mobilizer.outboard_body();
    DRAKE_DEMAND(&(body()) == &outboard_body);
    DRAKE_DEMAND(&(parent_body()) == &inboard_body);
    const std::string& inboard_body_name = inboard_body.name();
    const std::string& outboard_body_name = outboard_body.name();

    const std::string fail_msg(is_failure_absolute ? "not positive-definite. "
                                                   : "nearly singular. ");
    std::stringstream message;
    message << "An internal mass matrix associated with the joint that "
               "connects body " << inboard_body_name << " to body "
               << outboard_body_name << " is " << fail_msg;
    if (mobilizer.can_rotate()) {
      const RotationalInertia<T> I_BBo_B = outboard_body.
        CalcSpatialInertiaInBodyFrame(context).CalcRotationalInertia();
      message << "Since the joint allows rotation, ensure body "
              << outboard_body_name << " (combined with other outboard bodies) "
                 "has reasonable non-zero moments of inertia about joint "
                 "rotation axes. Note: The inertia matrix of body "
              << outboard_body_name << " about its body origin is "
              << I_BBo_B << ". ";
    }
    if (mobilizer.can_translate()) {
      const T outboard_body_mass = outboard_body.get_mass(context);
      message << "Since the joint allows translation, ensure body "
              << outboard_body_name << " (combined with other outboard bodies) "
                 "has a reasonable non-zero mass. Note: The mass of body "
              << outboard_body_name << " is " << outboard_body_mass << ". ";
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
