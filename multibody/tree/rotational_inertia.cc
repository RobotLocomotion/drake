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
boolean<T> RotationalInertia<
    T>::AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality() const {
  // We use a tiny multiple of max_possible_inertia_moment to guide the value
  // of ε. To avoid false negatives when max_possible_inertia_moment ≈ 0,
  // we also use a tiny absolute tolerance.
  // Note: A side effect of ε is that some inertias are incorrectly classified
  // as valid. We prefer to include some ever-so-slightly invalid inertias
  // rather than exclude ones that were definitely valid but were penalized by
  // finite precision during valid mathematical operations.
  using std::abs;
  using std::max;
  const double precision = 16 * std::numeric_limits<double>::epsilon();
  const T max_possible_inertia_moment = CalcMaximumPossibleMomentOfInertia();
  const T epsilon = precision * max(1.0, max_possible_inertia_moment);

  // TODO(Mitiguy) For now, this function is only used to test principal moments
  //  of inertia. A future PR will instead first perform the tests herein with
  //  `this` rotational inertia's diagonal moments of inertia e.g., with
  //  ExtractDoubleOrThrow(get_moments()) and also do the product of inertia
  //  inequality tests (e.g., is 2*abs(Ixy) ≤ Izz + ε).
  const Vector3<double> moments = CalcPrincipalMomentsOfInertia();

  const double Ixx = moments.x();
  const double Iyy = moments.y();
  const double Izz = moments.z();
  const auto are_moments_near_positive =
      AreMomentsOfInertiaNearPositive(Ixx, Iyy, Izz, epsilon);
  const auto is_triangle_inequality_satisfied = Ixx + Iyy + epsilon >= Izz &&
                                                Ixx + Iyy + epsilon >= Iyy &&
                                                Iyy + Izz + epsilon >= Ixx;
  return are_moments_near_positive && is_triangle_inequality_satisfied;
}

// (This implementation is adapted from Simbody's Rotation::reexpressSymMat33.)
// Use sneaky tricks from Featherstone to rotate a symmetric dyadic matrix.
// See Featherstone 2008, Appendix A.5, pg 254.
//
// Let the current rotational inertia matrix be I=I_EE, meaning it is a dyadic
// expressed in E (only the expressed-in frame matters here so we're switching
// notation to emphasize that). We're given a rotation matrix R_AE and would
// like to efficiently calculate I' = I_AA = R_AE I_EE R_AEᵀ, writing the result
// in place.
//
// If the calculation were actually performed by multiplying three generic 3x3
// matrices, it would take 90 flops (as used here a "flop" is a floating-point
// multiply or add), or 75 flops if we leveraged the symmetry of the I_AA and
// I_EE inertia matrices (each matrix only has 6 unique elements). However,
// with the clever algorithm below, the calculation cost is only 57 flops.
//
// We'll work with I's elements: I =[ a d e ]
//                                  [ d b f ]
//                                  [ e f c ]
//
// First, split I into I=L+D+v× with v=[-f e 0]ᵀ (× means cross product matrix):
//        [a-c   d   0]       [c 0 0]        [ 0  0  e]
//    L = [ d   b-c  0]   D = [0 c 0]   v× = [ 0  0  f]
//        [2e   2f   0]       [0 0 c]        [-e -f  0]
// (4 flops to calculate L)
//
// A cross product matrix identity says R v× Rᵀ=(Rv)×, so:
//    I' = R I Rᵀ = R L Rᵀ + D + (Rv)×.
// Let Y' = R L, Z = Y' Rᵀ. We only need the lower triangle of Z and a
// 2x2 square of Y'.
//
// Don't-care's below are marked "-". Below we'll use square bracket [i]
// index of a matrix to mean "row i", round bracket (j) means "column j".
//
//        [  -   -  0 ]
//   Y' = [ Y00 Y01 0 ]   Y = [ R[1]•L(0)  R[1]•L(1) ]     20 flops
//        [ Y10 Y11 0 ]       [ R[2]•L(0)  R[2]•L(1) ]
//
//   Z = [   Z00          -          -     ]
//       [ Y[0]•R[0]  Y[0]•R[1]      -     ]   15 flops (use only 2
//       [ Y[1]•R[0]  Y[1]•R[1]  Y[1]•R[2] ]   elements of R's rows)
//
//   Z00 = (L00+L11)-(Z11+Z22)  3 flops (because rotation preserves trace)
//
//         [e R01 - f R00]           [  0       -    -  ]
//    Rv = [e R11 - f R10]   (Rv)× = [ Rv[2]    0    -  ]
//         [e R21 - f R20]           [-Rv[1]  Rv[0]  0  ]
// (Rv is 9 flops)
//
//        [ Z00 + c          -           -    ]
//   I' = [ Z10 + Rv[2]  Z11 + c         -    ]
//        [ Z20 - Rv[1]  Z21 + Rv[0]  Z22 + c ]
//
// which takes 6 more flops. Total 6 + 9Rv + 18Z + 20Y + 4L = 57.
//
// (Looking at the generated assembly code for gcc 7.5 -O2 in Drake showed
// exactly 57 flops. clang 9.0 -O2 generated a few extra flops but in many fewer
// instructions since it was able to make much better use of SSE2 packed
// instructions. Both compilers were fully able to inline the Eigen methods here
// so there are no loops or function calls.)
template <typename T>
void RotationalInertia<T>::ReExpressInPlace(
    const math::RotationMatrix<T>& R_AE) {
  const Matrix3<T>& R = R_AE.matrix();

  // We're going to write back into this lower triangle.
  T& a = I_SP_E_(0, 0);
  T& d = I_SP_E_(1, 0);
  T& b = I_SP_E_(1, 1);
  T& e = I_SP_E_(2, 0);
  T& f = I_SP_E_(2, 1);
  T& c = I_SP_E_(2, 2);

  // Avoid use of Eigen's comma initializer since the compilers don't
  // reliably inline it.
  Eigen::Matrix<T, 3, 2> L;
  L(0, 0) = a - c;
  L(0, 1) = d;
  L(1, 0) = d;
  L(1, 1) = b - c;
  L(2, 0) = 2 * e;
  L(2, 1) = 2 * f;

  // For convenience below, the first two rows of Rᵀ.
  Eigen::Matrix<T, 2, 3> Rt;
  Rt(0, 0) = R(0, 0);
  Rt(0, 1) = R(1, 0);
  Rt(0, 2) = R(2, 0);
  Rt(1, 0) = R(0, 1);
  Rt(1, 1) = R(1, 1);
  Rt(1, 2) = R(2, 1);

  const Vector3<T> Rv(e * Rt.row(1) - f * Rt.row(0));

  Matrix2<T> Y;
  Y(0, 0) = R.row(1).dot(L.col(0));
  Y(0, 1) = R.row(1).dot(L.col(1));
  Y(1, 0) = R.row(2).dot(L.col(0));
  Y(1, 1) = R.row(2).dot(L.col(1));

  // We'll do Z elementwise due to element interdependence.
  const T Z11 = Y.row(0) * Rt.col(1);
  const T Z22 = Y.row(1) * Rt.col(2);
  const T Z00 = (L(0, 0) + L(1, 1)) - (Z11 + Z22);
  const T Z10 = Y.row(0) * Rt.col(0);
  const T Z20 = Y.row(1) * Rt.col(0);
  const T Z21 = Y.row(1) * Rt.col(1);

  // Assign result back into lower triangle. Don't set c until we're done
  // with the previous value!
  a = Z00 + c;
  d = Z10 + Rv[2];
  b = Z11 + c;
  e = Z20 - Rv[1];
  f = Z21 + Rv[0];
  c += Z22;

  // If both `this` and `R_AE` were valid upon entry to this method, the
  // returned rotational inertia should be valid.  Otherwise, it may not be.
  DRAKE_ASSERT_VOID(ThrowIfNotPhysicallyValid(__func__));
}

template <typename T>
std::optional<std::string> RotationalInertia<T>::CreateInvalidityReport()
    const {
  // Default return value is an empty string (this RotationalInertia is valid).
  std::string error_message;
  if (!IsFinite()) {
    error_message = fmt::format(
        "\nNon-finite moment or product of inertia "
        "detected in RotationalInertia.");
  } else if constexpr (scalar_predicate<T>::is_bool) {
    if (!AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality()) {
      const Vector3<double> p = CalcPrincipalMomentsOfInertia();
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
  if (error_message.empty()) return std::nullopt;
  return error_message;
}

template <typename T>
void RotationalInertia<T>::ThrowIfNotPhysicallyValidImpl(
    const char* func_name) const {
  DRAKE_DEMAND(func_name != nullptr);
  const std::optional<std::string> invalidity_report = CreateInvalidityReport();
  if (invalidity_report.has_value()) {
    const std::string error_message = fmt::format(
        "{}(): The rotational inertia\n"
        "{}did not pass the test CouldBePhysicallyValid().{}",
        func_name, *this, *invalidity_report);
    throw std::logic_error(error_message);
  }
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
      const std::string element = fmt::to_string(I(i, j));
      width = std::max<int>(width, static_cast<int>(element.length()));
    }
  }

  // Outputs to stream.
  for (int i = 0; i < I.rows(); ++i) {
    out << "[";
    if (width) out.width(width);
    out << fmt::to_string(I(i, 0));
    for (int j = 1; j < I.cols(); ++j) {
      out << "  ";
      if (width) out.width(width);
      out << fmt::to_string(I(i, j));
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
