#pragma once

/** @file
Declarations for fast, low-level functions for handling objects stored in small
matrices with known memory layouts. Ideally these are implemented using
platform-specific SIMD instructions for speed. There is always a straight
C++ fallback. */

/* Do not include code from drake/common here because this file will be
included by a compilation unit that may have a different opinion about whether
SIMD instructions are enabled than Eigen does in the rest of Drake. */

namespace drake {
namespace math {

/* We do not have access to the declaration for RotationMatrix
here. Instead, these functions depend on knowledge of the internal storage
format of this class, which is guaranteed and enforced by the class
declaration: Drake RotationMatrix objects are stored as 3x3 column-ordered
matrices in nine consecutive doubles.*/
template <typename>
class RotationMatrix;

namespace internal {

/* Composes two drake::math::RotationMatrix<double> objects as quickly as
possible, resulting in a new RotationMatrix.

Here we calculate `R_AC = R_AB * R_BC`. It is OK for R_AC to overlap
with one or both inputs. */
void ComposeRR(const RotationMatrix<double>& R_AB,
               const RotationMatrix<double>& R_BC,
               RotationMatrix<double>* R_AC);

/* Composes the inverse of a drake::math::RotationMatrix<double> object with
another (non-inverted) drake::math::RotationMatrix<double> as quickly as
possible, resulting in a new RotationMatrix.

@note A valid RotationMatrix is orthonormal, and the inverse of an orthonormal
matrix is just its transpose. This function assumes orthonormality and hence
simply multiplies the transpose of its first argument by the second.

Here we calculate `R_AC = R_BA⁻¹ * R_BC`. It is OK for R_AC to overlap
with one or both inputs. */
void ComposeRinvR(const RotationMatrix<double>& R_BA,
                  const RotationMatrix<double>& R_BC,
                  RotationMatrix<double>* R_AC);

/** Composes two drake::math::RigidTransform<double> objects as quickly as
possible. Drake RigidTransforms are stored as 3x4 column-ordered matrices in 12
consecutive doubles. The first nine elements comprise a 3x3 RotationMatrix and
the last three are a translation vector.

Note that this is a function specialized for RigidTransforms and is not just a
matrix multiply.

Here we calculate `X_AC = X_AB * X_BC`. It is OK for X_AC to overlap
with one or both inputs. */
void ComposeXX(const double* X_AB, const double* X_BC, double* X_AC);

/** Composes the inverse of a drake::math::RigidTransform<double> object with
another (non-inverted) drake::math::RigidTransform<double> as quickly as
possible. Drake RigidTransforms are stored as 3x4 column-ordered matrices in 12
consecutive doubles. The first nine elements comprise a 3x3 RotationMatrix and
the last three are a translation vector.

Note that this is a function specialized for RigidTransforms and is not just a
matrix multiply.

Here we calculate `X_AC = X_BA⁻¹ * X_BC`. It is OK for X_AC to overlap
with one or both inputs. */
void ComposeXinvX(const double* X_BA, const double* X_BC, double* X_AC);

/* Returns `true` if we are using the portable fallback implementations for
the above functions. */
bool IsUsingPortableCompositionFunctions();

/* These portable implementations are exposed so they can be unit tested.
Call IsUsingPortableCompositionFunctions() to determine whether these are being
used to implement the above functions. */

void ComposeRRPortable(const RotationMatrix<double>& R_AB,
                       const RotationMatrix<double>& R_BC,
                       RotationMatrix<double>* R_AC);
void ComposeRinvRPortable(const RotationMatrix<double>& R_BA,
                          const RotationMatrix<double>& R_BC,
                          RotationMatrix<double>* R_AC);

void ComposeXXPortable(const double* X_AB, const double* X_BC, double* X_AC);
void ComposeXinvXPortable(const double* X_BA, const double* X_BC, double* X_AC);

}  // namespace internal
}  // namespace math
}  // namespace drake
