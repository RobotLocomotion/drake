#include "drake/multibody/tree/rotational_inertia.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace multibody {

template <typename T>
Vector3<double> RotationalInertia<T>::CalcPrincipalMomentsAndAxesOfInertia(
      math::RotationMatrix<double>* principal_directions) const {
  // 1. Eigen's SelfAdjointEigenSolver does not compile for AutoDiffXd.
  //    Therefore, convert `this` to a local copy of type Matrix3<double>.
  // 2. Eigen's SelfAdjointEigenSolver only uses the lower-triangular part
  //    of this symmetric matrix.

  const Matrix3<T>& I_BP_E = get_matrix();
  Matrix3<double> I_double = Matrix3<double>::Constant(NAN);
  I_double(0, 0) = ExtractDoubleOrThrow(I_BP_E(0, 0));
  I_double(1, 1) = ExtractDoubleOrThrow(I_BP_E(1, 1));
  I_double(2, 2) = ExtractDoubleOrThrow(I_BP_E(2, 2));

  // Since the inertia matrix is symmetric, only the lower-triangular part of
  // the matrix is used (its upper-triangular elements are NaN).
  I_double(1, 0) = ExtractDoubleOrThrow(I_BP_E(1, 0));  // (1, 0) not (0, 1).
  I_double(2, 0) = ExtractDoubleOrThrow(I_BP_E(2, 0));  // (2, 0) not (0, 2).
  I_double(2, 1) = ExtractDoubleOrThrow(I_BP_E(2, 1));  // (2, 1) not (1, 2).

  // If all products of inertia are zero, no need to calculate eigenvalues.
  // The eigenvalues are diagonal elements and eigenvectors are identity matrix.
  const bool is_diagonal = (I_double(1, 0) == 0 && I_double(2, 0) == 0 &&
                            I_double(2, 1) == 0);
  if (is_diagonal) {
    // Sort the eigenvalues and eigenvectors in ascending order.
    double Imin = I_double(0, 0);
    double Imed = I_double(1, 1);
    double Imax = I_double(2, 2);
    int imin = 0, imed = 1, imax = 2;  // Indices.
    if (Imin > Imed) {
      std::swap(Imin, Imed);  // Imin gets lower value, Imed higher value.
      std::swap(imin, imed);  // imin = 1 and imed = 0.
    }
    if (Imin > Imax) {
      std::swap(Imin, Imax);  // Imin gets lower value, Imax higher value.
      std::swap(imin, imed);  // imin = 2 and imed = 0 or 1 (depends).
    }
    if (Imed > Imax) {
      std::swap(Imed, Imax);  // Imed gets lower value, Imax higher value
      std::swap(imed, imax);  // imed = 1 or 2 and imax = 0 or 1.
    }
    DRAKE_ASSERT(Imin <= Imed && Imed <= Imax);

    if (principal_directions != nullptr) {
      math::RotationMatrix<double> identity_matrix =
          math::RotationMatrix<double>::Identity();
      const Vector3<double> col_min = identity_matrix.col(imin);
      const Vector3<double> col_med = identity_matrix.col(imed);
      const Vector3<double> col_max = identity_matrix.col(imax);
      const bool is_right_handed = (col_min.cross(col_med)).dot(col_max) > 0;
      *principal_directions =
          math::RotationMatrix<double>::MakeFromOrthonormalColumns(
              col_min, col_med, is_right_handed ? col_max : -col_max);
    }

    return Vector3<double>(Imin, Imed, Imax);
  }

  // Calculate eigenvalues, possibly with eigenvectors.
  Eigen::SelfAdjointEigenSolver<Matrix3<double>> eig_solve;
  const int compute_Eigenvectors = principal_directions != nullptr ?
      Eigen::ComputeEigenvectors : Eigen::EigenvaluesOnly;
  eig_solve.compute(I_double, compute_Eigenvectors);
  if (eig_solve.info() != Eigen::Success) {
    const std::string error_message = fmt::format(
        "{}(): Unable to calculate the eigenvalues or eigenvectors of the "
        "3x3 matrix associated with a RotationalInertia.", __func__);
    throw std::logic_error(error_message);
  }

  // The eigen solver orders the storage of eigenvalues in increasing value and
  // orders the storage of their corresponding eigenvectors similarly.
  // Note: The rotational inertia's 3x3 matrix should be positive semi-definite,
  // so all the eigenvalues should be non-negative.
  if (principal_directions != nullptr) {
    // Form a right-handed orthogonal set of unit vectors Bx, By, Bz with the
    // 1st and 2nd eigenvectors and their cross-product. Unit vectors Bx, By, Bz
    // are expressed in the same frame E as `this` UnitInertia.
    const Vector3<double>& Bx_E = eig_solve.eigenvectors().col(0);
    const Vector3<double>& By_E = eig_solve.eigenvectors().col(1);
    const Vector3<double> Bz_E = Bx_E.cross(By_E);  // Ensure right-handed set.
    *principal_directions =
        math::RotationMatrix<double>::MakeFromOrthonormalColumns(Bx_E, By_E,
                                                                 Bz_E);
  }
  return eig_solve.eigenvalues();
}

template <typename T>
void RotationalInertia<T>::ThrowNotPhysicallyValid(const char* func_name)
    const {
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
              p(0), p(1), p(2), /* epsilon = */ 0.0)) {
        error_message += fmt::format(
            "\nThe associated principal moments of inertia:"
            "\n{}  {}  {}", p(0), p(1), p(2));
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

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<std::ostream&(*)(std::ostream&, const RotationalInertia<T>&)>(
        &operator<< )
))

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::RotationalInertia)
