#pragma once

/** @file
Internal use only. */

/* Declarations for fast, low-level functions for handling objects stored in
small matrices with known memory layouts, implemented using platform-specific
SIMD instructions for speed. */

/* N.B. Do not include any other drake headers here because this file will be
included by a compilation unit that may have a different opinion about whether
SIMD instructions are enabled than Eigen does in the rest of Drake. */

namespace drake {
namespace math {

/* We do not have access to the declarations for RotationMatrix and
RigidTransform here. Instead, the functions below depend on knowledge of the
internal storage format of these classes, which is guaranteed and enforced by
the class declarations:
 - Drake RotationMatrix objects are stored as 3x3 column-ordered matrices in
   nine consecutive doubles.
 - Drake RigidTransform objects are stored as 3x4 column-ordered matrices in
   twelve consecutive doubles. The first nine elements comprise a 3x3
   RotationMatrix and the last three are the translation vector. */

#ifndef DRAKE_DOXYGEN_CXX
template <typename>
class RotationMatrix;
template <typename>
class RigidTransform;
#endif

namespace internal {

/* Detects if the AVX2 implementations below are supported. Supported means that
both (1) AVX2 was enabled at built time, and (2) the processor executing this
code supports AVX2 instructions. */
bool AvxSupported();

/* Composes two drake::math::RotationMatrix<double> objects as quickly as
possible, resulting in a new RotationMatrix.

Here we calculate `R_AC = R_AB * R_BC`. It is OK for R_AC to overlap
with one or both inputs.

Note: if AVX2 is not supported, calling this function will crash the program. */
void ComposeRRAvx(
    const RotationMatrix<double>& R_AB,
    const RotationMatrix<double>& R_BC,
    RotationMatrix<double>* R_AC);

/* Composes the inverse of a drake::math::RotationMatrix<double> object with
another (non-inverted) drake::math::RotationMatrix<double> as quickly as
possible, resulting in a new RotationMatrix.

@note A valid RotationMatrix is orthonormal, and the inverse of an orthonormal
matrix is just its transpose. This function assumes orthonormality and hence
simply multiplies the transpose of its first argument by the second.

Here we calculate `R_AC = R_BA⁻¹ * R_BC`. It is OK for R_AC to overlap
with one or both inputs.

Note: if AVX2 is not supported, calling this function will crash the program. */
void ComposeRinvRAvx(
    const RotationMatrix<double>& R_BA,
    const RotationMatrix<double>& R_BC,
    RotationMatrix<double>* R_AC);

/** Composes two drake::math::RigidTransform<double> objects as quickly as
possible, resulting in a new RigidTransform.

@note This function is specialized for RigidTransforms and is not just a
matrix multiply.

Here we calculate `X_AC = X_AB * X_BC`. It is OK for X_AC to overlap
with one or both inputs.

Note: if AVX2 is not supported, calling this function will crash the program. */
void ComposeXXAvx(
    const RigidTransform<double>& X_AB,
    const RigidTransform<double>& X_BC,
    RigidTransform<double>* X_AC);

/** Composes the inverse of a drake::math::RigidTransform<double> object with
another (non-inverted) drake::math::RigidTransform<double> as quickly as
possible, resulting in a new RigidTransform.

@note This function is specialized for RigidTransforms and is not just a
matrix multiply.

Here we calculate `X_AC = X_BA⁻¹ * X_BC`. It is OK for X_AC to overlap
with one or both inputs.

Note: if AVX2 is not supported, calling this function will crash the program. */
void ComposeXinvXAvx(
    const RigidTransform<double>& X_BA,
    const RigidTransform<double>& X_BC,
    RigidTransform<double>* X_AC);

}  // namespace internal
}  // namespace math
}  // namespace drake
