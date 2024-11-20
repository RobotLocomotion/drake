#include "drake/multibody/tree/rotational_inertia.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace multibody {

namespace {

// Forms a canonical set of 3 principal moment of inertia directions from the
// non-canonical principal moment of inertia axes directions in @p R_EA.
// @param[in] principal_moments_inertia for `this` rotational inertia sorted
// from minimum to maximum.
// @param[in] R_EA 3x3 rotation matrix relating the expressed-in frame E for
// `this` rotational inertia to a frame A with unit vectors Ax, Ay, Az. The 1ˢᵗ
// column of R_EA is Ax_E (Ax expressed in frame E) which is parallel to the
// principal axis associated with Ixx (smallest principal moment of inertia).
// Similarly, the 2ⁿᵈ and 3ʳᵈ columns of R_EA are Ay_E and Az_E, which are
// parallel to principal axes associated with Iyy and Izz (the intermediate and
// largest principal moments of inertia).
// @pre The three values in principal_moments_inertia should be non-negative and
// finite and sorted from minimum to maximum.
// @returns 3x3 rotation matrix whose 3 canonical principal directions are
// deduced from R_EA and are closest to the identity matrix.
// @note Principal axes directions Ax, Ay, Az are not unique. Moments of inertia
// are associated with lines (not unit vectors), so it possible to reorient the
// principal directions in various ways, particularly if two or three of the
// principal moments of inertia are equal.
// @note The only case currently implemented is the triaxially symmetric case
// which is when all three principal moments of inertia are equal.
// TODO(Mitiguy) Implement the other two cases which are the axially symmetric
//  case (two principal moments of inertia are equal) and the case when each
//  principal moment of inertia is distinct.
math::RotationMatrix<double> CalcCanonicalPrincipalDirections(
    const Vector3<double>& principal_moments_inertia,
    const math::RotationMatrix<double>& R_EA, const double& inertia_tolerance) {
  const double& Imin = principal_moments_inertia(0);
  const double& Imax = principal_moments_inertia(2);
  math::RotationMatrix<double> canonical_principal_directions = R_EA;

  // Case: Triaxially symmetric principal moments of inertia (all 3 ≈ equal).
  // Physical example: uniform-density sphere.
  // For this case, body B's inertia matrix about point P expressed in the
  // right-handed orthonormal basis A of principal directions is
  //          ⎡Imin  0   0 ⎤     ⎡ 1  0  0 ⎤
  // I_BP_A ≈ | 0  Imed  0 | ≈ J | 0  1  0 |  where J ≈ Imin ≈ Imed ≈ Imax.
  //          ⎣ 0    0 Imax⎦     ⎣ 0  0  1 ⎦
  // We show: I_BP_E = I_BP_A, where E is any right-handed orthonormal basis.
  // Proof: To reexpress I_BP_A from basis A to basis E, use the rotation
  // matrix R_AE and its inverse R_EA as I_BP_E = R_EA * I_BP_A * R_AE.
  // Substitute for I_BP_A as I_BP_E = R_EA * J * IdentityMatrix * R_AE. Thus
  // I_BP_E = R_EA * J * R_AE = J * R_EA * R_AE = J * IdentityMatrix = I_BP_A.
  // Note: Before this function existed, the principal_directions could be:
  //                        ⎡ 0  1  0 ⎤                           ⎡ 1  0  0 ⎤
  // principal_directions = | 1  0  0 |   Now, this is changed to | 0  1  0 |
  //                        ⎣ 0  0 -1 ⎦.                          ⎣ 0  0  1 ⎦.
  if (Imax - Imin <= 2 * inertia_tolerance) {
    // Since rotation matrix is arbitrary, we choose the identity matrix.
    canonical_principal_directions = math::RotationMatrix<double>::Identity();
  }

  // TODO(Mitiguy) Return canonical principal directions for the other cases.
  //  Case: Axially symmetric moments of inertia (2 are equal, one differs).
  //  Case: Distinct principal moments of inertia (all 3 are different).

  return canonical_principal_directions;
}
}  // namespace

template <typename T>
Vector3<double> RotationalInertia<T>::CalcPrincipalMomentsAndMaybeAxesOfInertia(
    math::RotationMatrix<double>* principal_directions) const {
  Vector3<double> principal_moments;

  // 1. Eigen's SelfAdjointEigenSolver does not compile for AutoDiffXd.
  //    Therefore, convert `this` to a local copy of type Matrix3<double>.
  // 2. Eigen's SelfAdjointEigenSolver only uses the lower-triangular part
  //    of this symmetric matrix.
  const Matrix3<T>& I_BP_E = get_matrix();
  const Matrix3<double> I_double = ExtractDoubleOrThrow(I_BP_E);

  // Many calls to this function originate with a primitive such as a sphere,
  // ellipsoid, box, cylinder, or capsule that has zero products of inertia.
  // If all products of inertia are ≈ zero, no need to calculate eigenvalues or
  // eigenvectors as the principal moments of inertia are just the diagonal
  // elements and the principal directions are simple (e.g., [1, 0, 0]).
  // Note: the largest product of inertia is at most half the largest moment of
  // inertia, so compare products of inertia to a tolerance (rather than 0.0)
  // informed by machine epsilon multiplied by largest moment of inertia.
  // Note: It seems reasonable to surmise that an inertia tolerance based on
  // known properties of moments and products of inertia is generally better
  // than generic matrix tolerances used in Eigen's eigenvalue solver.
  const double inertia_tolerance =
      4 * std::numeric_limits<double>::epsilon() *
      ExtractDoubleOrThrow(CalcMaximumPossibleMomentOfInertia());
  const bool is_diagonal = (std::abs(I_double(1, 0)) <= inertia_tolerance &&
                            std::abs(I_double(2, 0)) <= inertia_tolerance &&
                            std::abs(I_double(2, 1)) <= inertia_tolerance);
  if (is_diagonal) {
    // Sort the principal moments of inertia (eigenvalues) and corresponding
    // principal directions (eigenvectors) in ascending order.
    using Pair = std::pair<double, int>;
    std::array I{Pair{I_double(0, 0), 0}, Pair{I_double(1, 1), 1},
                 Pair{I_double(2, 2), 2}};
    std::sort(I.begin(), I.end(), [](const Pair& l, const Pair& r) {
      return l < r;
    });
    const double Imin = I[0].first;  // Minimum principal moment of inertia.
    const double Imed = I[1].first;  // Intermediate principal moment of inertia
    const double Imax = I[2].first;  // Maximum principal moment of inertia.
    DRAKE_ASSERT(Imin <= Imed && Imed <= Imax);

    // If requested, also return the eigenvectors (principal directions).
    // The algorithm below ensures the first two columns of the returned 3x3
    // matrix are unit vectors with elements +1 or 0. The 3rd column is a unit
    // vector with elements ±1 or 0.
    if (principal_directions != nullptr) {
      math::RotationMatrix<double> identity_matrix =
          math::RotationMatrix<double>::Identity();
      const int index_min = I[0].second;  // Index associated with Imin.
      const int index_med = I[1].second;  // Index associated with Imed.
      const int index_max = I[2].second;  // Index associated with Imax.
      const Vector3<double> col_min = identity_matrix.col(index_min);
      const Vector3<double> col_med = identity_matrix.col(index_med);
      const Vector3<double> col_max = identity_matrix.col(index_max);
      const bool is_right_handed = (col_min.cross(col_med)).dot(col_max) > 0;
      *principal_directions =
          math::RotationMatrix<double>::MakeFromOrthonormalColumns(
              col_min, col_med, is_right_handed ? col_max : -col_max);
    }
    principal_moments = Vector3<double>(Imin, Imed, Imax);

  } else {
    // Calculate eigenvalues, possibly with eigenvectors.
    // TODO(Mitiguy) Since this is 3x3 matrix, consider Eigen's specialized
    //  compute_direct() method instead of compute() which uses QR.
    // compute_direct() calculates eigenvalues with a closed-form algorithm that
    // is usually signficantly faster than the QR iterative algorithm but may
    // be less accurate (e.g., for 3x3 matrix of doubles, accuracy ≈ 1.0E-8).
    Eigen::SelfAdjointEigenSolver<Matrix3<double>> eig_solve;
    const int compute_eigenvectors = principal_directions != nullptr
                                         ? Eigen::ComputeEigenvectors
                                         : Eigen::EigenvaluesOnly;
    eig_solve.compute(I_double, compute_eigenvectors);
    if (eig_solve.info() != Eigen::Success) {
      const std::string error_message = fmt::format(
          "{}(): Unable to calculate the eigenvalues or eigenvectors of the "
          "3x3 matrix associated with a RotationalInertia.",
          __func__);
      throw std::logic_error(error_message);
    }

    // The eigen solver orders the storage of eigenvalues in increasing value
    // and orders the storage of their corresponding eigenvectors similarly.
    // Note: The rotational inertia's 3x3 matrix should be positive
    // semi-definite, so all the eigenvalues should be non-negative.
    if (principal_directions != nullptr) {
      // Form a right-handed orthogonal set of unit vectors Bx, By, Bz with the
      // 1st and 2nd eigenvectors and their cross-product. Unit vectors Bx, By,
      // Bz are expressed in the same frame E as `this` UnitInertia.
      const Vector3<double>& Bx_E = eig_solve.eigenvectors().col(0);
      const Vector3<double>& By_E = eig_solve.eigenvectors().col(1);
      const Vector3<double> Bz_E = Bx_E.cross(By_E);  // Ensure right-handed set
      *principal_directions =
          math::RotationMatrix<double>::MakeFromOrthonormalColumns(Bx_E, By_E,
                                                                   Bz_E);
    }
    principal_moments = eig_solve.eigenvalues();
  }

  // Since moments of inertia are associated with lines (not unit vectors), the
  // rotation matrix stored in principal_directions does not uniquely describe
  // the principal axes directions. So, form "canonical" principal directions
  // that are closest to an identity rotation matrix.
  if (principal_directions != nullptr) {
    *principal_directions = CalcCanonicalPrincipalDirections(
        principal_moments, *principal_directions, inertia_tolerance);
  }
  return principal_moments;
}

template <typename T>
void RotationalInertia<T>::ThrowNotPhysicallyValid(
    const char* func_name) const {
  std::string error_message = fmt::format(
      "{}(): The rotational inertia\n"
      "{}did not pass the test CouldBePhysicallyValid().",
      func_name, *this);
  // Provide additional information if a moment of inertia is non-negative
  // or if moments of inertia do not satisfy the triangle inequality.
  if constexpr (scalar_predicate<T>::is_bool) {
    if (!IsNaN()) {
      const Vector3<double> p = CalcPrincipalMomentsOfInertia();
      if (!AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality(
              p(0), p(1), p(2))) {
        error_message += fmt::format(
            "\nThe associated principal moments of inertia:"
            "\n{}  {}  {}",
            p(0), p(1), p(2));
        if (p(0) < 0 || p(1) < 0 || p(2) < 0) {
          error_message += "\nare invalid since at least one is negative.";
        } else {
          error_message += "\ndo not satisfy the triangle inequality.";
        }
      }
    }
  }
  throw std::logic_error(error_message);
}

// TODO(Mitiguy) Consider using this code (or code similar to this) to write
//  most/all Drake matrices and consolidate other usages to use this.
// TODO(jwnimmer-tri) Obeying the formatting choices from `out` (via `copyfmt`
//  is a defect; we should always display full round-trip precision.  We should
//  not re-use this pattern elsewhere.
template <typename T>
std::ostream& operator<<(std::ostream& out, const RotationalInertia<T>& I) {
  int width = 0;
  // Computes largest width so that we can align columns for a prettier format.
  // Idea taken from: Eigen::internal::print_matrix() in Eigen/src/Core/IO.h
  for (int j = 0; j < I.cols(); ++j) {
    for (int i = 0; i < I.rows(); ++i) {
      std::stringstream sstr;
      sstr.copyfmt(out);
      sstr << I(i, j);
      width = std::max<int>(width, static_cast<int>(sstr.str().length()));
    }
  }

  // Outputs to stream.
  for (int i = 0; i < I.rows(); ++i) {
    out << "[";
    if (width) out.width(width);
    out << I(i, 0);
    for (int j = 1; j < I.cols(); ++j) {
      out << "  ";
      if (width) out.width(width);
      out << I(i, j);
    }
    out << "]\n";
  }
  return out;
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (static_cast<std::ostream& (*)(std::ostream&, const RotationalInertia<T>&)>(
        &operator<< )  // clang-format would remove space lint requires
));
// clang-format on

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::RotationalInertia);
